#!/usr/bin/env python3
"""엘베 층 전환 완료 시 4 DOF 하드코딩 시퀀스를 실행하는 노드.

- 구독: /floor_orchestrator/status (std_msgs/String, JSON)
- 발행: /packagu_arm/joint_cmd (sensor_msgs/JointState) — rate_hz 주기 보간 명령
- 드라이버: serial_port 파라미터가 비어 있으면 토픽 발행만(시뮬/드라이런),
  지정되면 시리얼로도 전송 (프로토콜은 Kim 실기 코드 확정 후 교체 — 아래 SerialDriver).

벤치 단독 테스트: ros2 run robot_arm_pkg arm_sequence --ros-args -p self_test:=true
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_arm_pkg.arm_sequence import (
    BUTTON_PRESS_SEQUENCE,
    JOINT_NAMES,
    FloorReadyTrigger,
    interpolate,
    sequence_duration,
)


class SerialDriver:
    """실서보 시리얼 백엔드. TODO(Kim): 실기 스크립트의 명령 포맷으로 send() 교체."""

    def __init__(self, port, baud, logger):
        import serial  # python3-serial — 이미지에 포함

        self._logger = logger
        self._conn = serial.Serial(port, baud, timeout=0.05)
        self._logger.info(f"arm serial open: {port} @ {baud}")

    def send(self, angles):
        line = "A " + " ".join(f"{a:.4f}" for a in angles) + "\n"
        self._conn.write(line.encode("ascii"))

    def close(self):
        self._conn.close()


class ArmSequenceNode(Node):
    def __init__(self):
        super().__init__("packagu_arm_sequence")
        self.declare_parameter("status_topic", "/floor_orchestrator/status")
        self.declare_parameter("cmd_topic", "/packagu_arm/joint_cmd")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("initial_floor", "F1")
        self.declare_parameter("serial_port", "")  # 예: /dev/arm_servo (launch 인자로만 지정)
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("self_test", False)

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._trigger = FloorReadyTrigger(str(self.get_parameter("initial_floor").value))
        self._elapsed = None  # None = 대기, float = 시퀀스 진행 중
        self._tick_count = 0

        self._driver = None
        port = str(self.get_parameter("serial_port").value).strip()
        if port:
            try:
                self._driver = SerialDriver(port, int(self.get_parameter("serial_baud").value), self.get_logger())
            except Exception as exc:  # noqa: BLE001 — 시리얼 실패는 부하테스트를 막지 않는다
                self.get_logger().error(f"arm serial open failed ({exc}) — topic-only로 계속")

        self._pub = self.create_publisher(JointState, str(self.get_parameter("cmd_topic").value), 10)
        self._sub = self.create_subscription(
            String, str(self.get_parameter("status_topic").value), self._on_status, 10
        )
        self._timer = self.create_timer(1.0 / self._rate_hz, self._on_tick)

        if bool(self.get_parameter("self_test").value):
            self.get_logger().info("self_test: 2초 후 시퀀스 1회 실행")
            self._self_test_timer = self.create_timer(2.0, self._start_self_test)

        self.get_logger().info(
            f"packagu_arm_sequence ready — trigger={self._trigger.last_floor} 이후 층 전환, "
            f"duration={sequence_duration():.1f}s @ {self._rate_hz:.0f}Hz, "
            f"driver={'serial' if self._driver else 'topic-only'}"
        )

    def _start_self_test(self):
        self._self_test_timer.cancel()
        self._start_sequence("self_test")

    def _on_status(self, msg):
        floor = self._trigger.observe(msg.data)
        if floor is None:
            return
        self._start_sequence(floor)

    def _start_sequence(self, reason):
        if self._elapsed is not None:
            self.get_logger().warn(f"시퀀스 진행 중 — {reason} 트리거 무시")
            return
        self._elapsed = 0.0
        self._tick_count = 0
        self.get_logger().info(f"버튼 시퀀스 시작 ({reason}) — {sequence_duration():.1f}s")

    def _on_tick(self):
        if self._elapsed is None:
            return
        angles = interpolate(self._elapsed, BUTTON_PRESS_SEQUENCE)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(a) for a in angles]
        self._pub.publish(msg)
        if self._driver is not None:
            try:
                self._driver.send(angles)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"arm serial send failed ({exc}) — driver 비활성화")
                self._driver = None
        self._tick_count += 1
        self._elapsed += 1.0 / self._rate_hz
        if self._elapsed >= sequence_duration():
            self.get_logger().info(f"버튼 시퀀스 완료 — {self._tick_count} ticks")
            self._elapsed = None


def main(args=None):
    rclpy.init(args=args)
    node = ArmSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._driver is not None:
            node._driver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
