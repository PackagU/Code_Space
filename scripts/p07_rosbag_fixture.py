#!/usr/bin/env python3
"""Synthetic ROS fixture for P07 rosbag late-join and safe replay tests."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


STATIC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def run_publisher(duration_sec: float, publish_command: bool):
    rclpy.init()
    node = rclpy.create_node("packagu_p07_fixture_publisher")
    scan_pub = node.create_publisher(LaserScan, "/scan", 20)
    odom_pub = node.create_publisher(Odometry, "/odom", 20)
    imu_pub = node.create_publisher(Imu, "/imu", 20)
    tf_pub = node.create_publisher(TFMessage, "/tf", 20)
    static_pub = node.create_publisher(TFMessage, "/tf_static", STATIC_QOS)
    command_pub = node.create_publisher(Twist, "/cmd_vel", 20) if publish_command else None

    static = TransformStamped()
    static.header.stamp = node.get_clock().now().to_msg()
    static.header.frame_id = "base_footprint"
    static.child_frame_id = "laser"
    static.transform.translation.z = 0.5
    static.transform.rotation.w = 1.0
    static_pub.publish(TFMessage(transforms=[static]))

    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        stamp = node.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.785
        scan.range_min = 0.12
        scan.range_max = 12.0
        scan.ranges = [2.0] * 5
        scan_pub.publish(scan)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu_pub.publish(imu)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        tf_pub.publish(TFMessage(transforms=[transform]))

        if command_pub is not None:
            command = Twist()
            command.linear.x = 0.08
            command_pub.publish(command)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


def run_verifier(duration_sec: float, output: Path):
    rclpy.init()
    node = rclpy.create_node("packagu_p07_fixture_verifier")
    counts = {"scan": 0, "tf_static": 0, "cmd_vel": 0}
    subscriptions = [
        node.create_subscription(LaserScan, "/scan", lambda _msg: counts.__setitem__("scan", counts["scan"] + 1), 20),
        node.create_subscription(TFMessage, "/tf_static", lambda _msg: counts.__setitem__("tf_static", counts["tf_static"] + 1), STATIC_QOS),
        node.create_subscription(Twist, "/cmd_vel", lambda _msg: counts.__setitem__("cmd_vel", counts["cmd_vel"] + 1), 20),
    ]
    _ = subscriptions
    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(counts, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(counts, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)
    publisher = sub.add_parser("publish")
    publisher.add_argument("--duration-sec", type=float, default=6.0)
    publisher.add_argument(
        "--publish-command",
        action="store_true",
        help="include /cmd_vel only in an isolated replay-safety test domain",
    )
    verifier = sub.add_parser("verify")
    verifier.add_argument("--duration-sec", type=float, default=4.0)
    verifier.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.mode == "publish":
        run_publisher(args.duration_sec, args.publish_command)
    else:
        run_verifier(args.duration_sec, args.output)


if __name__ == "__main__":
    main()
