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

        # 기본 피드백 상한은 max_wheel_rpm 그대로이며, 거부 사유가 원인별로 남는다.
        assert node.feedback_max_abs_rpm == node.max_wheel_rpm
        for line, reason in (
            (b"F 30.46 12.0\n", "feedback rpm over limit"),
            (b"E dynamixel_write 3\n", "firmware error: dynamixel_write"),
            (b"HELLO opencr 0.2-minimal\n", "firmware hello line"),
        ):
            feed_valid(node, fake)
            command(node)
            node.send_command_tick()
            count_before = node.rejected_feedback_count
            odom_before = node.last_odom_msg
            fake.lines.append(line)
            node.poll_feedback_tick(dt_override=0.02)
            assert node.last_odom_msg is odom_before, line
            assert not node.motion_ready and node._ready_reason == reason, (line, node._ready_reason)
            assert node.rejected_feedback_count == count_before + 1

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

        # 센서 전용 I/G 프레임은 wheel freshness나 odom을 위조하지 않는다.
        node._last_feedback_time = None
        odom_before = node.last_odom_msg
        fake.lines.append(b"I 0.1 0.2 0.3 0 0 9.81 1 0 0 0\n")
        node.poll_feedback_tick(dt_override=0.02)
        assert node.last_odom_msg is odom_before
        assert node._last_feedback_time is None and node.imu_ready
        assert node.last_imu_msg.header.frame_id == "imu_link"
        assert node.last_imu_msg.orientation.w == 1.0

        fake.lines.append(b"G 0 0 0.4\n")
        node.poll_feedback_tick(dt_override=0.02)
        assert node.last_odom_msg is odom_before
        assert node.last_imu_msg.orientation_covariance[0] == -1.0
        assert node.last_imu_msg.linear_acceleration_covariance[0] == -1.0
        assert node.last_imu_msg.angular_velocity.z == 0.4

        # 잘못된 IMU 프레임은 IMU만 not-ready로 만들고 drive 상태 이유는 덮지 않는다.
        feed_valid(node, fake)
        command(node)
        node.send_command_tick()
        drive_reason = node._ready_reason
        fake.lines.append(b"I 0 0\n")
        node.poll_feedback_tick(dt_override=0.02)
        assert not node.imu_ready
        assert node._ready_reason == drive_reason

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

        # 현장 A/B용 피드백 타당성 상한은 명시 설정 때만 넓어지고 명령 상한은 그대로다.
        for value, accepted in ((33.0, True), (29.0, False), (61.0, False), (math.nan, False)):
            try:
                wide = _bridge_with_feedback_limit(value)
            except ValueError:
                assert not accepted, value
                continue
            try:
                assert accepted, value
                assert wide.feedback_max_abs_rpm == value and wide.max_wheel_rpm == 30.0
                command(wide)
                wide.transport.lines.append(b"F 30.46 12.0\n")
                wide.poll_feedback_tick(dt_override=0.02)
                assert wide.last_odom_msg is not None and wide.motion_ready
            finally:
                wide.destroy_node()
        print("opencr_safety_contract tests passed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _bridge_with_feedback_limit(limit):
    class LimitedBridge(OpencrBridgeNode):
        def declare_parameter(self, name, value=None, *args, **kwargs):
            if name == "feedback_max_abs_rpm":
                value = limit
            return super().declare_parameter(name, value, *args, **kwargs)

    return LimitedBridge(transport=FakeSerial())


if __name__ == "__main__":
    main()
