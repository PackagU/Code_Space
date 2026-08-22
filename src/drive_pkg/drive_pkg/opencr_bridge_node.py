#!/usr/bin/env python3
"""OpenCR 시리얼 브리지: /cmd_vel -> V 프레임, F 피드백 -> /odom + TF + /imu.

프로토콜: docs/deployment/02_opencr_serial_protocol.md (v0.1)
테스트: scripts/test_opencr_bridge_dryrun.py (FakeSerial 주입)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.51324)
        self.declare_parameter("left_sign", 1.0)
        self.declare_parameter("right_sign", 1.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("cmd_rate_hz", 20.0)
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
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.imu_frame = p("imu_frame").value
        self.publish_tf = p("publish_tf").value

        self.transport = transport if transport is not None else self._open_serial()
        self.odometry = DiffDriveOdometry(
            self.wheel_radius, self.wheel_separation, self.left_sign, self.right_sign
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._last_feedback_time = None
        self.last_command_bytes = b""
        self.last_odom_msg = None
        self.last_imu_msg = None

        cmd_period = 1.0 / p("cmd_rate_hz").value
        self.create_timer(cmd_period, self.send_command_tick)
        self.create_timer(0.01, self.poll_feedback_tick)  # 100Hz 폴링

    def _open_serial(self):
        import serial  # 지연 import: 오프라인 테스트는 transport 주입으로 우회

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baudrate").value
        self.get_logger().info(f"opening serial {port} @ {baud}")
        return serial.Serial(port, baud, timeout=0.0)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd_vel(self, msg):
        self._cmd_v = msg.linear.x
        self._cmd_w = msg.angular.z
        self._last_cmd_time = self._now_sec()

    def force_last_cmd_age_for_test(self, age_sec):
        self._last_cmd_time = self._now_sec() - age_sec

    def send_command_tick(self):
        stale = (
            self._last_cmd_time is None
            or (self._now_sec() - self._last_cmd_time) > self.cmd_timeout
        )
        if stale:
            left_rpm, right_rpm = 0.0, 0.0
        else:
            left_rpm, right_rpm = twist_to_wheel_rpm(
                self._cmd_v, self._cmd_w, self.wheel_radius, self.wheel_separation
            )
            left_rpm *= self.left_sign
            right_rpm *= self.right_sign
        self.last_command_bytes = encode_velocity_command(left_rpm, right_rpm)
        self.transport.write(self.last_command_bytes)

    def poll_feedback_tick(self, dt_override=None):
        for _ in range(MAX_LINES_PER_TICK):
            raw = self.transport.readline()
            if not raw:
                return
            feedback = parse_feedback_line(raw.decode("ascii", errors="replace"))
            if feedback is None:
                self.get_logger().debug(f"ignored line: {raw!r}")
                continue
            now = self._now_sec()
            if dt_override is not None:
                dt = dt_override
            elif self._last_feedback_time is None:
                dt = 0.0
            else:
                dt = now - self._last_feedback_time
            self._last_feedback_time = now
            self.odometry.update(feedback["left_rpm"], feedback["right_rpm"], dt)
            self._publish_odom()
            self._publish_imu(feedback)

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
        self.imu_pub.publish(msg)
        self.last_imu_msg = msg

    def send_zero_command_safe(self):
        """종료 경로 전용 — 실패해도 진행 (시리얼이 이미 닫혔을 수 있음)."""
        try:
            self.transport.write(encode_velocity_command(0.0, 0.0))
        except Exception:  # noqa: BLE001
            pass


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
