#!/usr/bin/env python3
"""Isolated-test-only stationary base publishers; never connects to hardware."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def main():
    rclpy.init()
    lidar = Node("rplidar")
    drive = Node("packagu_opencr_bridge")
    safety = Node("nav_safety_gate")
    scan_pub = lidar.create_publisher(LaserScan, "/scan", 10)
    odom_pub = drive.create_publisher(Odometry, "/odom", 10)
    drive_ready_pub = drive.create_publisher(Bool, "/drive/ready", 10)
    safety_ready_pub = safety.create_publisher(Bool, "/nav_safety/ready", 10)
    tf_broadcaster = TransformBroadcaster(drive)
    static_broadcaster = StaticTransformBroadcaster(drive)

    static = TransformStamped()
    static.header.stamp = drive.get_clock().now().to_msg()
    static.header.frame_id = "base_footprint"
    static.child_frame_id = "laser"
    static.transform.translation.z = 0.5
    static.transform.rotation.w = 1.0
    static_broadcaster.sendTransform(static)

    def publish_stationary_state():
        stamp = drive.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.scan_time = 0.1
        scan.range_min = 0.12
        scan.range_max = 8.0
        scan.ranges = [2.0] * 360
        scan_pub.publish(scan)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        tf_broadcaster.sendTransform(transform)

        ready = Bool()
        ready.data = True
        drive_ready_pub.publish(ready)
        safety_ready_pub.publish(ready)

    drive.create_timer(0.1, publish_stationary_state)
    executor = SingleThreadedExecutor()
    for node in (lidar, drive, safety):
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in (lidar, drive, safety):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
