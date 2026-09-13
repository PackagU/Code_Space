#!/usr/bin/env python3
"""Synthetic command-chain publisher for the probe runtime test only.

It publishes motion topic names, so it refuses to run outside an explicitly
isolated test domain or when a real base node is visible.
"""

import argparse
import os
import sys
import time

ALLOWED_DOMAINS = range(200, 233)
REAL_BASE_NODES = {"packagu_opencr_bridge", "nav_safety_gate", "rplidar", "controller_server"}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=3.5)
    args = parser.parse_args(argv)
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if os.environ.get("PACKAGU_ISOLATED_TEST_DOMAIN") != "1" or not domain.isdigit() or int(domain) not in ALLOWED_DOMAINS:
        print("error: fixture requires PACKAGU_ISOLATED_TEST_DOMAIN=1 and ROS_DOMAIN_ID 200..232", file=sys.stderr)
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String

    rclpy.init()
    node = rclpy.create_node("packagu_cmd_chain_fixture")
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    visible = set(node.get_node_names())
    if visible & REAL_BASE_NODES:
        print(f"error: real base nodes visible in domain {domain}: {sorted(visible & REAL_BASE_NODES)}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 3

    bridge_logger = rclpy.create_node("packagu_opencr_bridge")  # rosout name only; no serial port
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {topic: node.create_publisher(Twist, topic, 10) for topic in ("/cmd_vel_nav", "/cmd_vel", "/cmd_vel_safe")}
    ready_pub = node.create_publisher(Bool, "/drive/ready", 10)
    status_pub = node.create_publisher(String, "/nav_safety/status", latched)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    scan_pub = node.create_publisher(LaserScan, "/scan", 10)

    deadline = time.monotonic() + 8.0
    while pubs["/cmd_vel_safe"].get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if pubs["/cmd_vel_safe"].get_subscription_count() == 0:
        print("error: probe subscription not discovered", file=sys.stderr)
        return 4
    settle = time.monotonic() + 2.0
    while time.monotonic() < settle:
        rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(bridge_logger, timeout_sec=0.0)

    def status(text):
        msg = String()
        msg.data = text
        status_pub.publish(msg)

    status("command forwarded")
    start = time.monotonic()
    tick = 0
    logged = False
    recovered = False
    while True:
        now = time.monotonic() - start
        if now >= args.duration_sec:
            break
        if tick % 2 == 0:
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.twist.twist.linear.x = 0.01
            odom_pub.publish(odom)
        if tick % 10 == 0:
            scan = LaserScan()
            scan.header.stamp = node.get_clock().now().to_msg()
            scan.header.frame_id = "laser"
            scan_pub.publish(scan)
        blip = 1.50 <= now < 1.65
        ready_false = 1.48 <= now < 1.66
        if ready_false and not logged:
            bridge_logger.get_logger().warning("drive ready=False: feedback rpm over limit")
            status("drive not ready")
            logged = True
        if now >= 1.70 and not recovered:
            status("command forwarded")
            recovered = True
        if tick % 5 == 0:
            command = Twist()
            command.linear.x = 0.01
            pubs["/cmd_vel_nav"].publish(command)
            pubs["/cmd_vel"].publish(command)
            pubs["/cmd_vel_safe"].publish(Twist() if blip else command)
            ready = Bool()
            ready.data = not ready_false
            ready_pub.publish(ready)
        tick += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(max(0.0, start + tick * 0.01 - time.monotonic()))

    time.sleep(0.5)
    bridge_logger.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
    print("fixture done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
