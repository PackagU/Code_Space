#!/usr/bin/env python3
"""OpenCR 시리얼 브리지: /cmd_vel -> V 프레임, F 피드백 -> /odom + TF + /imu.

프로토콜: docs/deployment/02_opencr_serial_protocol.md (v0.2)
테스트: scripts/test_opencr_bridge_dryrun.py (FakeSerial 주입)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from drive_pkg.opencr_protocol import (
    encode_velocity_command,
    parse_feedback_line,
    twist_to_wheel_rpm,
)
from drive_pkg.diff_drive_odometry import DiffDriveOdometry, yaw_to_quaternion

MAX_LINES_PER_TICK = 20


class OpencrBridgeNode(Node):
    def __init__(self, transport=None):
        super().__init__("packagu_opencr_bridge")
        self.declare_parameter("serial_port", "/dev/opencr")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.51324)
        self.declare_parameter("left_sign", 1.0)
        self.declare_parameter("right_sign", 1.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("feedback_timeout_sec", 0.5)
        self.declare_parameter("max_feedback_dt_sec", 0.25)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_wheel_rpm", 30.0)
        self.declare_parameter("max_wheel_accel_rpm_s", 60.0)
        self.declare_parameter("require_feedback_before_motion", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        p = self.get_parameter
        self.wheel_radius = p("wheel_radius").value
        self.wheel_separation = p("wheel_separation").value
        self.left_sign = p("left_sign").value
        self.right_sign = p("right_sign").value
        self.cmd_timeout = p("cmd_timeout_sec").value
        self.cmd_rate_hz = p("cmd_rate_hz").value
        self.feedback_timeout = p("feedback_timeout_sec").value
        self.max_feedback_dt = p("max_feedback_dt_sec").value
        self.max_linear_speed = p("max_linear_speed").value
        self.max_angular_speed = p("max_angular_speed").value
        self.max_wheel_rpm = p("max_wheel_rpm").value
        self.max_wheel_accel_rpm_s = p("max_wheel_accel_rpm_s").value
        self.require_feedback_before_motion = p("require_feedback_before_motion").value
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.imu_frame = p("imu_frame").value
        self.publish_tf = p("publish_tf").value

        positive_parameters = {
            "wheel_radius": self.wheel_radius,
            "wheel_separation": self.wheel_separation,
            "cmd_timeout_sec": self.cmd_timeout,
            "cmd_rate_hz": self.cmd_rate_hz,
            "feedback_timeout_sec": self.feedback_timeout,
            "max_feedback_dt_sec": self.max_feedback_dt,
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed,
            "max_wheel_rpm": self.max_wheel_rpm,
            "max_wheel_accel_rpm_s": self.max_wheel_accel_rpm_s,
        }
        invalid_parameters = [
            name for name, value in positive_parameters.items()
            if not math.isfinite(value) or value <= 0.0
        ]
        if invalid_parameters:
            raise ValueError(f"parameters must be finite and positive: {invalid_parameters}")
        if not all(math.isfinite(value) and value != 0.0 for value in (self.left_sign, self.right_sign)):
            raise ValueError("left_sign and right_sign must be finite and non-zero")

        self.transport = transport if transport is not None else self._open_serial()
        self.odometry = DiffDriveOdometry(
            self.wheel_radius, self.wheel_separation, self.left_sign, self.right_sign
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.ready_pub = self.create_publisher(Bool, "/drive/ready", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist, str(p("cmd_vel_topic").value), self.on_cmd_vel, 10
        )

        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._last_feedback_time = None
        self._last_sent_left_rpm = 0.0
        self._last_sent_right_rpm = 0.0
        self.motion_ready = False
        self._ready_reason = "startup"
        self.last_command_bytes = b""
        self.last_odom_msg = None
        self.last_imu_msg = None

        cmd_period = 1.0 / self.cmd_rate_hz
        self.create_timer(cmd_period, self.send_command_tick)
        self.create_timer(0.01, self.poll_feedback_tick)  # 100Hz 폴링
        self._publish_ready()

    def _open_serial(self):
        import serial  # 지연 import: 오프라인 테스트는 transport 주입으로 우회

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baudrate").value
        self.get_logger().info(f"opening serial {port} @ {baud}")
        return serial.Serial(port, baud, timeout=0.0)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd_vel(self, msg):
        values = (msg.linear.x, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self._invalidate_command("non-finite cmd_vel")
            return
        if abs(msg.linear.x) > self.max_linear_speed or abs(msg.angular.z) > self.max_angular_speed:
            self._invalidate_command("cmd_vel exceeds configured limit")
            return
        self._cmd_v = msg.linear.x
        self._cmd_w = msg.angular.z
        self._last_cmd_time = self._now_sec()

    def _invalidate_command(self, reason):
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._set_motion_ready(False, reason)

    def _publish_ready(self):
        msg = Bool()
        msg.data = self.motion_ready
        self.ready_pub.publish(msg)

    def _set_motion_ready(self, ready, reason):
        changed = ready != self.motion_ready or reason != self._ready_reason
        self.motion_ready = ready
        self._ready_reason = reason
        if changed:
            if ready:
                self.get_logger().info(f"drive ready=True: {reason}")
            else:
                self.get_logger().warning(f"drive ready=False: {reason}")
            self._publish_ready()

    @staticmethod
    def _slew(current, target, max_step):
        return max(current - max_step, min(current + max_step, target))

    def force_last_cmd_age_for_test(self, age_sec):
        self._last_cmd_time = self._now_sec() - age_sec

    def send_command_tick(self):
        now = self._now_sec()
        stale = (
            self._last_cmd_time is None
            or (now - self._last_cmd_time) < 0.0
            or (now - self._last_cmd_time) > self.cmd_timeout
        )
        feedback_stale = (
            self._last_feedback_time is None
            or (now - self._last_feedback_time) < 0.0
            or (now - self._last_feedback_time) > self.feedback_timeout
        )
        if stale or (self.require_feedback_before_motion and feedback_stale):
            left_rpm, right_rpm = 0.0, 0.0
            self._last_sent_left_rpm = 0.0
            self._last_sent_right_rpm = 0.0
            reason = "cmd_vel stale" if stale else "feedback stale"
            if feedback_stale:
                # 피드백이 복구돼도 오래 저장된 명령이 자동 재개되지 않게 한다.
                self._last_cmd_time = None
            self._set_motion_ready(False, reason)
        else:
            try:
                target_left, target_right = twist_to_wheel_rpm(
                    self._cmd_v, self._cmd_w, self.wheel_radius, self.wheel_separation
                )
                target_left *= self.left_sign
                target_right *= self.right_sign
                if max(abs(target_left), abs(target_right)) > self.max_wheel_rpm:
                    raise ValueError("wheel RPM exceeds configured limit")
                max_step = self.max_wheel_accel_rpm_s / self.cmd_rate_hz
                left_rpm = self._slew(self._last_sent_left_rpm, target_left, max_step)
                right_rpm = self._slew(self._last_sent_right_rpm, target_right, max_step)
                self._last_sent_left_rpm = left_rpm
                self._last_sent_right_rpm = right_rpm
                self._set_motion_ready(True, "fresh command and feedback")
            except ValueError as exc:
                left_rpm, right_rpm = 0.0, 0.0
                self._last_sent_left_rpm = 0.0
                self._last_sent_right_rpm = 0.0
                self._set_motion_ready(False, str(exc))
        try:
            self.last_command_bytes = encode_velocity_command(left_rpm, right_rpm)
            self.transport.write(self.last_command_bytes)
        except Exception as exc:  # noqa: BLE001
            self.last_command_bytes = b"V 0.00 0.00\n"
            self._last_sent_left_rpm = 0.0
            self._last_sent_right_rpm = 0.0
            self._set_motion_ready(False, f"serial write failed: {exc}")

    def poll_feedback_tick(self, dt_override=None):
        latest_feedback = None
        for _ in range(MAX_LINES_PER_TICK):
            try:
                raw = self.transport.readline()
            except Exception as exc:  # noqa: BLE001
                self._set_motion_ready(False, f"serial read failed: {exc}")
                return
            if not raw:
                break
            feedback = parse_feedback_line(
                raw.decode("ascii", errors="replace"), max_abs_rpm=self.max_wheel_rpm
            )
            if feedback is None:
                self.get_logger().debug(f"ignored line: {raw!r}")
                self._set_motion_ready(False, "invalid feedback frame")
                continue
            latest_feedback = feedback
        if latest_feedback is None:
            return

        now = self._now_sec()
        if dt_override is not None:
            dt = dt_override
        elif self._last_feedback_time is None:
            dt = 0.0
        else:
            dt = now - self._last_feedback_time
        self._last_feedback_time = now
        if not math.isfinite(dt) or dt < 0.0 or dt > self.max_feedback_dt:
            self._set_motion_ready(False, f"invalid feedback dt={dt}")
            return
        self.odometry.update(latest_feedback["left_rpm"], latest_feedback["right_rpm"], dt)
        self._publish_odom()
        if latest_feedback["quat"] is not None:
            self._publish_imu(latest_feedback)
        command_age = None if self._last_cmd_time is None else now - self._last_cmd_time
        if command_age is not None and 0.0 <= command_age <= self.cmd_timeout:
            self._set_motion_ready(True, "fresh command and feedback")
        else:
            self._set_motion_ready(False, "waiting for fresh cmd_vel")

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.odometry.x
        msg.pose.pose.position.y = self.odometry.y
        qx, qy, qz, qw = yaw_to_quaternion(self.odometry.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.odometry.v
        msg.twist.twist.angular.z = self.odometry.w
        msg.pose.covariance = _diagonal_covariance(
            (0.10, 0.10, 1e6, 1e6, 1e6, 0.20), 6
        )
        msg.twist.covariance = _diagonal_covariance(
            (0.10, 0.10, 1e6, 1e6, 1e6, 0.20), 6
        )
        self.odom_pub.publish(msg)
        self.last_odom_msg = msg
        if self.publish_tf:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odometry.x
            tf.transform.translation.y = self.odometry.y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def _publish_imu(self, feedback):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation.w = feedback["quat"][0]
        msg.orientation.x = feedback["quat"][1]
        msg.orientation.y = feedback["quat"][2]
        msg.orientation.z = feedback["quat"][3]
        msg.angular_velocity.x = feedback["gyro"][0]
        msg.angular_velocity.y = feedback["gyro"][1]
        msg.angular_velocity.z = feedback["gyro"][2]
        msg.linear_acceleration.x = feedback["accel"][0]
        msg.linear_acceleration.y = feedback["accel"][1]
        msg.linear_acceleration.z = feedback["accel"][2]
        msg.orientation_covariance = _diagonal_covariance((0.10, 0.10, 0.20), 3)
        msg.angular_velocity_covariance = _diagonal_covariance((0.02, 0.02, 0.02), 3)
        msg.linear_acceleration_covariance = _diagonal_covariance((0.10, 0.10, 0.10), 3)
        self.imu_pub.publish(msg)
        self.last_imu_msg = msg

    def send_zero_command_safe(self):
        """종료 경로 전용 — 실패해도 진행 (시리얼이 이미 닫혔을 수 있음)."""
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._last_sent_left_rpm = 0.0
        self._last_sent_right_rpm = 0.0
        self._set_motion_ready(False, "shutdown")
        for _ in range(3):
            try:
                self.transport.write(encode_velocity_command(0.0, 0.0))
            except Exception:  # noqa: BLE001
                break


def _diagonal_covariance(values, width):
    matrix = [0.0] * (width * width)
    for index, value in enumerate(values):
        matrix[index * width + index] = value
    return matrix


def main():
    rclpy.init()
    node = OpencrBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_command_safe()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
