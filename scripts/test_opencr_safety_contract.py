#!/usr/bin/env python3
"""OpenCR bridge 오류 입력·정지·복구 계약의 FakeSerial 회귀 시험."""

import math
from pathlib import Path
import sys

import rclpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "drive_pkg"))

from drive_pkg.opencr_bridge_node import OpencrBridgeNode  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402


VALID_STOPPED = b"F 0 0 0 0 0 0 0 9.81 1 0 0 0\n"


class FakeSerial:
    def __init__(self):
        self.lines = []
        self.written = []
        self.fail_read = False
        self.fail_write = False

    def readline(self):
        if self.fail_read:
            raise OSError("injected read failure")
        return self.lines.pop(0) if self.lines else b""

    def write(self, data):
        if self.fail_write:
            raise OSError("injected write failure")
        self.written.append(bytes(data))


def feed_valid(node, fake, dt=0.02):
    fake.lines.append(VALID_STOPPED)
    node.poll_feedback_tick(dt_override=dt)
    assert node.last_odom_msg is not None


def command(node, linear=0.1, angular=0.0):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    node.on_cmd_vel(msg)


def main():
    rclpy.init()
    fake = FakeSerial()
    node = OpencrBridgeNode(transport=fake)
    try:
        # startup과 피드백 단절에서는 항상 0.
        command(node)
        node.send_command_tick()
        assert node.last_command_bytes == b"V 0.00 0.00\n"
        feed_valid(node, fake)

        # 정상 명령은 가속 제한을 지키며 0이 아닌 프레임을 만든다.
        command(node)
        node.send_command_tick()
        left = float(node.last_command_bytes.split()[1])
        assert 0.0 < left <= node.max_wheel_accel_rpm_s / node.cmd_rate_hz + 0.01

        # NaN/Inf와 설정 범위 밖 명령은 즉시 0·not ready.
        for linear, angular in ((math.nan, 0.0), (0.0, math.inf), (1.0, 0.0), (0.0, 2.0)):
            feed_valid(node, fake)
            command(node, linear, angular)
            node.send_command_tick()
            assert node.last_command_bytes == b"V 0.00 0.00\n"
            assert not node.motion_ready

        # 개별 선·각속도 한계 안이어도 합성 wheel RPM 한계를 넘으면 정지한다.
        feed_valid(node, fake)
        command(node, node.max_linear_speed, node.max_angular_speed)
        node.send_command_tick()
        assert node.last_command_bytes == b"V 0.00 0.00\n"
        assert not node.motion_ready

        # 유효 피드백이 timeout되면 저장된 명령을 폐기하고 자동 재개하지 않는다.
        feed_valid(node, fake)
        command(node)
        node._last_feedback_time = node._now_sec() - node.feedback_timeout - 0.1
        node.send_command_tick()
        assert node.last_command_bytes == b"V 0.00 0.00\n"
        assert node._last_cmd_time is None and not node.motion_ready

        # 잘못된 프레임과 범위 밖 RPM은 odom/imu를 갱신하지 않는다.
        feed_valid(node, fake)
        odom_before = node.last_odom_msg
        for line in (
            b"F nan 0 0 0 0 0 0 0 1 0 0 0\n",
            b"F 999 0 0 0 0 0 0 0 1 0 0 0\n",
            b"F 0 0 0 0 0 0 0 0 0 0 0 0\n",
            b"garbage\n",
        ):
            fake.lines.append(line)
        node.poll_feedback_tick(dt_override=0.02)
        assert node.last_odom_msg is odom_before
        assert not node.motion_ready

        # 큰 dt와 시각 역행은 적분하지 않고, 다음 정상 프레임에서 복구한다.
        x_before = node.odometry.x
        fake.lines.append(VALID_STOPPED)
        node.poll_feedback_tick(dt_override=node.max_feedback_dt + 1.0)
        assert node.odometry.x == x_before and not node.motion_ready
        fake.lines.append(VALID_STOPPED)
        node.poll_feedback_tick(dt_override=-0.01)
        assert node.odometry.x == x_before and not node.motion_ready
        feed_valid(node, fake)

        # 큐에 쌓인 피드백은 마지막 프레임 하나만 반영한다.
        fake.lines.extend((
            b"F 30 30 0 0 0 0 0 9.81 1 0 0 0\n",
            VALID_STOPPED,
        ))
        x_before = node.odometry.x
        node.poll_feedback_tick(dt_override=0.02)
        assert node.odometry.x == x_before

        # read/write 예외는 process 성공이나 ready로 바뀌지 않는다.
        fake.fail_read = True
        node.poll_feedback_tick()
        assert not node.motion_ready
        fake.fail_read = False
        feed_valid(node, fake)
        command(node)
        fake.fail_write = True
        node.send_command_tick()
        assert node.last_command_bytes == b"V 0.00 0.00\n"
        assert not node.motion_ready
        fake.fail_write = False

        before = len(fake.written)
        node.send_zero_command_safe()
        assert fake.written[before:] == [b"V 0.00 0.00\n"] * 3
        assert not node.motion_ready
        print("opencr_safety_contract tests passed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
