#!/usr/bin/env python3
"""브리지 노드 dry-run: FakeSerial 주입, ROS 필요 (호스트에서는 러너가 SKIP).

컨테이너 실행:
  ROS_DOMAIN_ID=89 python3 scripts/test_opencr_bridge_dryrun.py
"""
import math
import sys
from pathlib import Path

import rclpy  # 호스트에 없으면 ModuleNotFoundError -> 러너 SKIP 규약

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "drive_pkg"))

from drive_pkg.opencr_bridge_node import OpencrBridgeNode  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402


class FakeSerial:
    def __init__(self, lines=None):
        self.lines = list(lines or [])
        self.written = []

    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        return b""

    def write(self, data):
        self.written.append(bytes(data))


def approx(a, b, tol, label):
    if abs(a - b) > tol:
        raise AssertionError(f"{label}: expected {b}, got {a}")


def main():
    rclpy.init()
    fake = FakeSerial([
        b"HELLO opencr 0.1\n",                                 # 무시되어야 함
        b"F 60.0 60.0 0.0 0.0 0.0 0.0 0.0 9.81 1.0 0.0 0.0 0.0\n",
    ])
    node = OpencrBridgeNode(transport=fake)
    try:
        # 1) 신선한 cmd_vel -> V 프레임 인코딩 확인
        twist = Twist()
        twist.linear.x = 0.2
        node.on_cmd_vel(twist)
        node.send_command_tick()
        assert node.last_command_bytes.startswith(b"V "), "V frame not sent"
        assert node.last_command_bytes != b"V 0.00 0.00\n", "fresh cmd must not be zeroed"
        assert fake.written and fake.written[-1] == node.last_command_bytes, "command not written"

        # 2) watchdog: cmd_vel 0.6s 경과 시 0 명령
        node.force_last_cmd_age_for_test(0.6)
        node.send_command_tick()
        assert node.last_command_bytes == b"V 0.00 0.00\n", "watchdog zero command missing"

        # 3) 피드백 처리: 첫 tick이 큐를 소진(HELLO 무시 + F 적분 1.0s), 둘째 tick은 no-op
        node.poll_feedback_tick(dt_override=1.0)
        node.poll_feedback_tick(dt_override=1.0)
        v_expected = 60.0 * 2.0 * math.pi / 60.0 * 0.033
        assert node.last_odom_msg is not None, "odom not published"
        approx(node.last_odom_msg.pose.pose.position.x, v_expected, 1e-6, "odom x")
        approx(node.last_odom_msg.twist.twist.linear.x, v_expected, 1e-6, "odom v")
        assert node.last_odom_msg.header.frame_id == "odom"
        assert node.last_odom_msg.child_frame_id == "base_footprint"
        assert node.last_imu_msg is not None, "imu not published"
        approx(node.last_imu_msg.linear_acceleration.z, 9.81, 1e-6, "imu az")
        print("opencr_bridge dry-run tests passed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
