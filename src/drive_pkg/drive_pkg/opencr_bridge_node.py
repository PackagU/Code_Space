#!/usr/bin/env python3
"""OpenCR 시리얼 브리지: /cmd_vel -> V, wheel/IMU feedback -> ROS topics.

v0.2 wheel-only와 v0.3 IMU/gyro-only 프레임을 함께 수용한다.
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
    classify_rejected_feedback,
    classify_rejected_imu,
    encode_velocity_command,
    parse_feedback_line,
    parse_imu_line,
    twist_to_wheel_rpm,
)
from drive_pkg.diff_drive_odometry import DiffDriveOdometry, yaw_to_quaternion

MAX_LINES_PER_TICK = 20
# 2026-09-13: 관측용 로그 간격일 뿐 안전 동작과 무관하다 [제안값].
REJECTED_FEEDBACK_LOG_PERIOD_SEC = 1.0
# 사용자 진술(2026-09-13) 바퀴 모터 하드웨어 최대 60 rpm. 피드백 타당성 상한은 이 값을 넘지 않는다.
FEEDBACK_PLAUSIBILITY_CEILING_RPM = 60.0


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
        self.declare_parameter("feedback_poll_hz", 50.0)
        self.declare_parameter("feedback_timeout_sec", 0.5)
        self.declare_parameter("imu_timeout_sec", 0.5)
        self.declare_parameter("max_feedback_dt_sec", 0.25)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_wheel_rpm", 30.0)
        self.declare_parameter("max_wheel_accel_rpm_s", 60.0)
        # 0.0이면 max_wheel_rpm과 같다(2026-09-12 동작 그대로). 명령 상한은 바꾸지 않는다.
        self.declare_parameter("feedback_max_abs_rpm", 0.0)
        self.declare_parameter("require_feedback_before_motion", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)

        p = self.get_parameter
        self.wheel_radius = p("wheel_radius").value
        self.wheel_separation = p("wheel_separation").value
        self.left_sign = p("left_sign").value
        self.right_sign = p("right_sign").value
        self.cmd_timeout = p("cmd_timeout_sec").value
        self.cmd_rate_hz = p("cmd_rate_hz").value
        self.feedback_poll_hz = p("feedback_poll_hz").value
        if not (self.feedback_poll_hz and self.feedback_poll_hz > 0.0):
            raise ValueError("feedback_poll_hz must be positive")
        self.feedback_timeout = p("feedback_timeout_sec").value
        self.imu_timeout = p("imu_timeout_sec").value
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
            "feedback_poll_hz": self.feedback_poll_hz,
            "feedback_timeout_sec": self.feedback_timeout,
            "imu_timeout_sec": self.imu_timeout,
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
        feedback_limit = float(p("feedback_max_abs_rpm").value)
        self.feedback_max_abs_rpm = self.max_wheel_rpm if feedback_limit == 0.0 else feedback_limit
        if not (
            math.isfinite(self.feedback_max_abs_rpm)
            and self.max_wheel_rpm <= self.feedback_max_abs_rpm <= FEEDBACK_PLAUSIBILITY_CEILING_RPM
        ):
            raise ValueError(
                "feedback_max_abs_rpm must be 0 or between max_wheel_rpm and "
                f"{FEEDBACK_PLAUSIBILITY_CEILING_RPM} rpm"
            )

        self.transport = transport if transport is not None else self._open_serial()
        self.odometry = DiffDriveOdometry(
            self.wheel_radius, self.wheel_separation, self.left_sign, self.right_sign
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.ready_pub = self.create_publisher(Bool, "/drive/ready", 10)
        self.imu_ready_pub = self.create_publisher(Bool, "/imu/ready", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Twist, str(p("cmd_vel_topic").value), self.on_cmd_vel, 10
        )

        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._last_feedback_time = None
        self._last_imu_time = None
        self._last_sent_left_rpm = 0.0
        self._last_sent_right_rpm = 0.0
        self.motion_ready = False
        self.imu_ready = False
        self._ready_reason = "startup"
        self.last_command_bytes = b""
        self.last_odom_msg = None
        self.last_imu_msg = None
        self.rejected_feedback_count = 0
        self.last_rejected_feedback_reason = None
        self._last_rejected_log_time = None
        # Buffered nonblocking feedback; readiness heartbeat.
        self._rx_buf = bytearray()
        self._rx_started_at = None
        self._rx_discarding = False

        cmd_period = 1.0 / self.cmd_rate_hz
        self.create_timer(cmd_period, self.send_command_tick)
        # 2026-09-12: 고정 100Hz -> 파라미터화. 펌웨어 FEEDBACK_PERIOD_MS=20 (50Hz) 이라
        # 100Hz 폴링은 절반이 빈 깨움이었다. 기본 [제안값] 50Hz.
        # poll_feedback_tick 은 버퍼에 쌓인 줄을 모두 비우므로 느린 폴링에서도 프레임을 잃지 않는다.
        self.create_timer(1.0 / self.feedback_poll_hz, self.poll_feedback_tick)
        self._publish_ready()
        self._publish_imu_ready()

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

    def _publish_imu_ready(self):
        msg = Bool()
        msg.data = self.imu_ready
        self.imu_ready_pub.publish(msg)

    def _set_imu_ready(self, ready, reason):
        changed = ready != self.imu_ready
        self.imu_ready = ready
        if changed:
            if ready:
                self.get_logger().info(f"imu ready=True: {reason}")
            else:
                self.get_logger().warning(f"imu ready=False: {reason}")
        self._publish_imu_ready()

    def _refresh_imu_ready(self, now=None):
        now = self._now_sec() if now is None else now
        stale = (
            self._last_imu_time is None
            or (now - self._last_imu_time) < 0.0
            or (now - self._last_imu_time) > self.imu_timeout
        )
        if stale:
            self._set_imu_ready(False, "imu stale")

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

        # Heartbeat after freshness checks and the serial write attempt.
        self._publish_ready()

    def _read_feedback_line(self):
        # readline() can return a fragment with timeout=0. Keep it until LF.
        now = self._now_sec()
        if self._rx_started_at is not None:
            age = now - self._rx_started_at
            if not math.isfinite(age) or age < 0.0 or age > self.feedback_timeout:
                self._rx_buf.clear()
                self._rx_started_at = None
                self._rx_discarding = True
                raise ValueError("partial feedback frame expired")

        raw = self.transport.readline()
        if self._rx_discarding:
            # readline() ends at the first LF, so this restores a boundary.
            if raw.endswith(b"\n"):
                self._rx_discarding = False
            return b""
        if not raw:
            return b""
        if not self._rx_buf:
            self._rx_started_at = now
        self._rx_buf.extend(raw)
        if len(self._rx_buf) > 1024:  # Proposed limit; protocol frames are smaller.
            self._rx_buf.clear()
            self._rx_started_at = None
            self._rx_discarding = not raw.endswith(b"\n")
            raise ValueError("feedback frame exceeds 1024 bytes")
        if not self._rx_buf.endswith(b"\n"):
            return b""
        line = bytes(self._rx_buf)
        self._rx_buf.clear()
        self._rx_started_at = None
        return line

    def poll_feedback_tick(self, dt_override=None):
        latest_feedback = None
        latest_imu = None
        for _ in range(MAX_LINES_PER_TICK):
            try:
                raw = self._read_feedback_line()
            except Exception as exc:  # noqa: BLE001
                if self._rx_buf:
                    self._rx_discarding = True
                self._rx_buf.clear()
                self._rx_started_at = None
                self._last_feedback_time = None
                self._invalidate_command(f"serial read failed: {exc}")
                return
            if not raw:
                break
            text = raw.decode("ascii", errors="replace")
            feedback = parse_feedback_line(text, max_abs_rpm=self.feedback_max_abs_rpm)
            if feedback is not None:
                latest_feedback = feedback
                if feedback["gyro"] is not None:
                    latest_imu = feedback
                continue

            imu = parse_imu_line(text)
            if imu is not None:
                latest_imu = imu
                continue

            imu_reason = classify_rejected_imu(text)
            stripped = text.strip()
            if imu_reason is not None or stripped.startswith("E imu_"):
                reason = imu_reason or classify_rejected_feedback(text)
                self._note_rejected_feedback(reason, text)
                self._last_imu_time = None
                self._set_imu_ready(False, reason)
                continue

            reason = classify_rejected_feedback(text, max_abs_rpm=self.feedback_max_abs_rpm)
            self._note_rejected_feedback(reason, text)
            self._set_motion_ready(False, reason)

        now = self._now_sec()
        if latest_imu is not None:
            self._publish_imu(latest_imu)
            self._last_imu_time = now
            self._set_imu_ready(True, "fresh valid imu frame")
        else:
            self._refresh_imu_ready(now)

        if latest_feedback is None:
            return
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
        command_age = None if self._last_cmd_time is None else now - self._last_cmd_time
        if command_age is not None and 0.0 <= command_age <= self.cmd_timeout:
            self._set_motion_ready(True, "fresh command and feedback")
        else:
            self._set_motion_ready(False, "waiting for fresh cmd_vel")

    def _note_rejected_feedback(self, reason, text):
        """Count every rejected line and log its content at most once per period."""
        self.rejected_feedback_count += 1
        self.last_rejected_feedback_reason = reason
        now = self._now_sec()
        last = self._last_rejected_log_time
        if last is None or now - last >= REJECTED_FEEDBACK_LOG_PERIOD_SEC or now < last:
            self._last_rejected_log_time = now
            self.get_logger().warning(
                f"rejected feedback ({reason}); total={self.rejected_feedback_count}; "
                f"line={text.strip()[:80]!r}"
            )

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
        if feedback["quat"] is not None:
            msg.orientation.w = feedback["quat"][0]
            msg.orientation.x = feedback["quat"][1]
            msg.orientation.y = feedback["quat"][2]
            msg.orientation.z = feedback["quat"][3]
            msg.orientation_covariance = _diagonal_covariance((0.10, 0.10, 0.20), 3)
        else:
            # REP-145/SensorMsgs convention: first element -1 means no estimate.
            msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = feedback["gyro"][0]
        msg.angular_velocity.y = feedback["gyro"][1]
        msg.angular_velocity.z = feedback["gyro"][2]
        if feedback["accel"] is not None:
            msg.linear_acceleration.x = feedback["accel"][0]
            msg.linear_acceleration.y = feedback["accel"][1]
            msg.linear_acceleration.z = feedback["accel"][2]
            msg.linear_acceleration_covariance = _diagonal_covariance((0.10, 0.10, 0.10), 3)
        else:
            msg.linear_acceleration_covariance[0] = -1.0
        msg.angular_velocity_covariance = _diagonal_covariance((0.02, 0.02, 0.02), 3)
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
