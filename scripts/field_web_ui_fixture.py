#!/usr/bin/env python3
"""Isolated sensor/map fixture for the local field web UI runtime contract."""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=7.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Node("packagu_field_web_ui_fixture")
    scan_pub = node.create_publisher(LaserScan, "/scan", 10)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    ready_pub = node.create_publisher(Bool, "/drive/ready", 10)
    map_pub = node.create_publisher(OccupancyGrid, "/map", MAP_QOS)
    tf_pub = TransformBroadcaster(node)
    static_pub = StaticTransformBroadcaster(node)
    counts = {
        "total": 0,
        "nonzero": 0,
        "zero_after_nonzero": 0,
        "max_abs_linear": 0.0,
        "max_abs_angular": 0.0,
    }
    saw_nonzero = False

    def on_command(msg):
        nonlocal saw_nonzero
        counts["total"] += 1
        counts["max_abs_linear"] = max(counts["max_abs_linear"], abs(msg.linear.x))
        counts["max_abs_angular"] = max(counts["max_abs_angular"], abs(msg.angular.z))
        moving = abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6
        if moving:
            counts["nonzero"] += 1
            saw_nonzero = True
        elif saw_nonzero:
            counts["zero_after_nonzero"] += 1

    subscription = node.create_subscription(Twist, "/cmd_vel", on_command, 20)
    _ = subscription

    static = TransformStamped()
    static.header.stamp = node.get_clock().now().to_msg()
    static.header.frame_id = "base_footprint"
    static.child_frame_id = "laser"
    static.transform.translation.z = 0.5
    static.transform.rotation.w = 1.0
    static_pub.sendTransform(static)

    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = 80
    grid.info.height = 48
    grid.info.resolution = 0.05
    grid.info.origin.position.x = -2.0
    grid.info.origin.position.y = -1.2
    grid.info.origin.orientation.w = 1.0
    data = []
    for y in range(grid.info.height):
        for x in range(grid.info.width):
            wall = y in (4, grid.info.height - 5) or x in (4, grid.info.width - 5)
            data.append(100 if wall else 0)
    grid.data = data

    start = time.monotonic()
    while time.monotonic() - start < args.duration_sec:
        stamp = node.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / 36
        scan.range_min = 0.12
        scan.range_max = 8.0
        scan.ranges = [2.0] * 37
        scan_pub.publish(scan)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        tf_pub.sendTransform(transform)
        ready = Bool(); ready.data = True; ready_pub.publish(ready)
        grid.header.stamp = stamp; map_pub.publish(grid)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(counts, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(counts, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
