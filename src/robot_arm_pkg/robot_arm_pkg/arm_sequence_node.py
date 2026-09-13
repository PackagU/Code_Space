#!/usr/bin/env python3
"""Mission-commanded arm node with controller-response gating and cancellation.

The safe default opens no serial port, performs no startup homing, and ignores the
legacy floor-ready trigger. Completion means only that four PRAD responses matched
each commanded pose; it is not a physical-success claim. The optional simulation
mode is reported as ``simulated_complete``.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_arm_pkg import servo_protocol as sp
from robot_arm_pkg.arm_execution import ArmExecutionContract
from robot_arm_pkg.arm_sequence import FloorReadyTrigger


class ArmSequenceNode(Node):
    def __init__(self):
        super().__init__("packagu_arm_sequence")
        self.declare_parameter("command_topic", "/packagu_arm/command")
        self.declare_parameter("status_topic", "/packagu_arm/status")
        self.declare_parameter("legacy_floor_status_topic", "/floor_orchestrator/status")
        self.declare_parameter("enable_floor_trigger", False)
        self.declare_parameter("command_estimate_topic", "/packagu_arm/joint_cmd_estimate")
        self.declare_parameter("publish_command_estimate", False)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("initial_floor", "F1")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("home_on_start", False)
        self.declare_parameter("simulation_mode", False)
        self.declare_parameter("require_homed", True)
        self.declare_parameter("feedback_timeout_sec", 1.0)
        self.declare_parameter("position_tolerance_pwm", 30)
        self.declare_parameter("stow_verified", False)
        self.declare_parameter("press_cycle", 1)
        self.declare_parameter("self_test", False)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), status_qos
        )
        self._estimate_pub = None
        if bool(self.get_parameter("publish_command_estimate").value):
            self._estimate_pub = self.create_publisher(
                JointState, str(self.get_parameter("command_estimate_topic").value), 10
            )

        simulation_mode = bool(self.get_parameter("simulation_mode").value)
        port = str(self.get_parameter("serial_port").value).strip()
        if simulation_mode and port:
            raise RuntimeError("simulation_mode must not receive a physical serial port")

        driver = None
        serial_error = ""
        if port:
            try:
                driver = sp.SerialPoseDriver(
                    port,
                    int(self.get_parameter("serial_baud").value),
                    logger=self.get_logger(),
                )
            except Exception as exc:  # noqa: BLE001
                serial_error = f"serial_open_failed:{type(exc).__name__}"
                self.get_logger().error(f"arm serial open failed: {exc}")

        self._contract = ArmExecutionContract(
            driver,
            simulation_mode=simulation_mode,
            require_homed=bool(self.get_parameter("require_homed").value),
            feedback_timeout_ms=1000.0
            * float(self.get_parameter("feedback_timeout_sec").value),
            position_tolerance_pwm=int(self.get_parameter("position_tolerance_pwm").value),
            stow_verified=bool(self.get_parameter("stow_verified").value),
        )
        self._driver = driver
        self._default_cycle = int(self.get_parameter("press_cycle").value)
        sp.get_cycle(self._default_cycle)
        self._trigger = FloorReadyTrigger(str(self.get_parameter("initial_floor").value))
        self._self_test_phase = "disabled"

        self._command_sub = self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self._on_command,
            10,
        )
        self._legacy_sub = None
        if bool(self.get_parameter("enable_floor_trigger").value):
            self._legacy_sub = self.create_subscription(
                String,
                str(self.get_parameter("legacy_floor_status_topic").value),
                self._on_floor_status,
                10,
            )
            self.get_logger().warn(
                "legacy floor-ready trigger enabled; explicit mission commands are recommended"
            )

        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate_hz, self._on_tick)

        if serial_error:
            self._contract.state = "failed"
            self._contract.phase = "hardware_unavailable"
            self._publish(self._contract.snapshot("failed", error=serial_error))
        else:
            self._publish(self._contract.snapshot("ready"))

        if bool(self.get_parameter("home_on_start").value):
            self._submit_dict(
                {"request_id": "startup-home", "action": "home"},
                source="startup",
            )

        if bool(self.get_parameter("self_test").value):
            if not simulation_mode:
                raise RuntimeError("self_test is permitted only with simulation_mode:=true")
            self._self_test_phase = "home_pending"
            self._submit_dict(
                {"request_id": "self-test-home", "action": "home"},
                source="self_test",
            )

        self.get_logger().info(
            "arm contract ready: driver=%s simulation=%s home_on_start=%s legacy_trigger=%s"
            % (
                "serial" if driver else "none",
                simulation_mode,
                bool(self.get_parameter("home_on_start").value),
                bool(self.get_parameter("enable_floor_trigger").value),
            )
        )

    @staticmethod
    def _now_ms():
        return time.monotonic() * 1000.0

    def _publish(self, status):
        msg = String()
        msg.data = json.dumps(status, sort_keys=True, separators=(",", ":"))
        self._status_pub.publish(msg)
        event = status.get("event", "")
        if event in ("failed", "rejected", "cancelled", "completed"):
            if event == "failed":
                self.get_logger().error(msg.data)
            else:
                self.get_logger().info(msg.data)
        self._publish_estimate(status)

    def _publish_estimate(self, status):
        if self._estimate_pub is None or "target_pwm" not in status:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"servo_{sid}_command_pwm_estimate" for sid in sp.SERVO_IDS]
        msg.position = [float(status["target_pwm"][sid]) for sid in sp.SERVO_IDS]
        self._estimate_pub.publish(msg)

    def _submit_dict(self, command, source):
        try:
            status = self._contract.submit(command, self._now_ms())
        except ValueError as exc:
            status = self._contract.snapshot(
                "rejected", error=f"invalid_command:{exc}", request=command
            )
        status["source"] = source
        self._publish(status)
        return status

    def _on_command(self, msg):
        try:
            status = self._contract.submit(msg.data, self._now_ms())
        except ValueError as exc:
            status = self._contract.snapshot("rejected", error=f"invalid_command:{exc}")
        status["source"] = "mission"
        self._publish(status)

    def _on_floor_status(self, msg):
        floor = self._trigger.observe(msg.data)
        if floor is None:
            return
        self._submit_dict(
            {
                "request_id": f"legacy-floor-{floor}",
                "action": "press",
                "target": "destination",
                "button": floor,
                "press_cycle": self._default_cycle,
            },
            source="legacy_floor_trigger",
        )

    def _on_tick(self):
        status = self._contract.tick(self._now_ms())
        if status is None:
            return
        status["source"] = "executor"
        self._publish(status)
        if self._self_test_phase == "home_pending" and status["event"] == "completed":
            self._self_test_phase = "press_pending"
            self._submit_dict(
                {
                    "request_id": "self-test-press",
                    "action": "press",
                    "target": "call",
                    "button": "TEST",
                    "press_cycle": self._default_cycle,
                },
                source="self_test",
            )
        elif self._self_test_phase == "press_pending" and status["event"] == "completed":
            self._self_test_phase = "done"

    def shutdown(self):
        if self._contract.state == "busy":
            status = self._contract.submit(
                {
                    "request_id": f"shutdown-{time.monotonic_ns()}",
                    "action": "cancel",
                    "target_request_id": self._contract.active["request_id"],
                },
                self._now_ms(),
            )
            status["source"] = "shutdown"
            self._publish(status)
        if self._driver is not None:
            self._driver.close()


def main(args=None):
    rclpy.init(args=args)
    node = ArmSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
