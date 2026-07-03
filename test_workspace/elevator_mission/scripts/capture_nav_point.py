#!/usr/bin/env python3
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseCapture:
    def __init__(self, node):
        self.node = node
        self.message = None
        self.subscription = node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_pose,
            10,
        )

    def _on_pose(self, message):
        self.message = message


def main():
    parser = argparse.ArgumentParser(
        description="Print a kku_nav_points.yaml entry from the current /amcl_pose."
    )
    parser.add_argument("--floor", required=True, help="Floor id such as F1, F2, or F3")
    parser.add_argument("--point", required=True, help="Point id such as 303 or elevator_exit")
    parser.add_argument("--type", required=True, help="Point type such as room, elevator, or parcel")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("capture_nav_point")
    capture = PoseCapture(node)

    deadline = node.get_clock().now().nanoseconds + int(args.timeout_sec * 1e9)
    while rclpy.ok() and capture.message is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > deadline:
            node.destroy_node()
            rclpy.shutdown()
            print("ERROR: timed out waiting for /amcl_pose", file=sys.stderr)
            return 1

    pose = capture.message.pose.pose
    yaw_deg = math.degrees(yaw_from_quaternion(pose.orientation))

    print(f"# Paste under floors.{args.floor}.points in kku_nav_points.yaml")
    print(f'"{args.point}": {{type: {args.type}, x: {pose.position.x:.3f}, y: {pose.position.y:.3f}, yaw_deg: {yaw_deg:.1f}}}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
