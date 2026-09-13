#!/usr/bin/env python3
"""ROS wrapper that interposes a fail-closed gate before the OpenCR bridge."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from drive_pkg.safety_gate import SafetyGate


class NavSafetyGateNode(Node):
    def __init__(self):
        super().__init__("nav_safety_gate")
        self.declare_parameter("input_cmd_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("drive_ready_topic", "/drive/ready")
        self.declare_parameter("software_stop_topic", "/nav_safety/stop")
        self.declare_parameter("sensor_timeout_sec", 0.5)
        self.declare_parameter("drive_ready_timeout_sec", 0.5)
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.35)

        p = self.get_parameter
        publish_rate = float(p("publish_rate_hz").value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.gate = SafetyGate(
            sensor_timeout_sec=float(p("sensor_timeout_sec").value),
            drive_ready_timeout_sec=float(p("drive_ready_timeout_sec").value),
            command_timeout_sec=float(p("command_timeout_sec").value),
            max_linear_speed=float(p("max_linear_speed").value),
            max_angular_speed=float(p("max_angular_speed").value),
        )

        input_cmd = str(p("input_cmd_topic").value)
        output_cmd = str(p("output_cmd_topic").value)
        if input_cmd == output_cmd:
            raise ValueError("input_cmd_topic and output_cmd_topic must differ")

        self.command = Twist()
        self.command_time = None
        self.last_reason = None
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_pub = self.create_publisher(Twist, output_cmd, 10)
        self.ready_pub = self.create_publisher(Bool, "/nav_safety/ready", state_qos)
        self.stopped_pub = self.create_publisher(Bool, "/nav_safety/stopped", state_qos)
        self.status_pub = self.create_publisher(String, "/nav_safety/status", state_qos)

        self.create_subscription(Twist, input_cmd, self._on_command, 10)
        self.create_subscription(
            LaserScan, str(p("scan_topic").value), lambda _msg: self.gate.observe_scan(self._now()), 10
        )
        self.create_subscription(
            Odometry, str(p("odom_topic").value), lambda _msg: self.gate.observe_odom(self._now()), 10
        )
        self.create_subscription(
            Bool,
            str(p("drive_ready_topic").value),
            lambda msg: self.gate.observe_drive_ready(msg.data, self._now()),
            10,
        )
        self.create_subscription(
            Bool,
            str(p("software_stop_topic").value),
            self._on_software_stop,
            10,
        )
        self.create_timer(1.0 / publish_rate, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_command(self, msg):
        self.command = msg
        self.command_time = self._now()

    def _on_software_stop(self, msg):
        self.gate.set_software_stop(msg.data)

    def _tick(self):
        result = self.gate.filter_command(
            self.command.linear.x,
            self.command.angular.z,
            self.command_time,
            self._now(),
        )
        output = Twist()
        output.linear.x = result.linear_x
        output.angular.z = result.angular_z
        self.command_pub.publish(output)

        ready = Bool()
        ready.data = result.system_ready
        self.ready_pub.publish(ready)
        stopped = Bool()
        stopped.data = self.gate.software_stop
        self.stopped_pub.publish(stopped)
        if result.reason != self.last_reason:
            status = String()
            status.data = result.reason
            self.status_pub.publish(status)
            if result.command_forwarded:
                self.get_logger().info(result.reason)
            else:
                self.get_logger().warning(result.reason)
            self.last_reason = result.reason

    def publish_zero(self):
        self.command_pub.publish(Twist())


def main():
    rclpy.init()
    node = NavSafetyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
