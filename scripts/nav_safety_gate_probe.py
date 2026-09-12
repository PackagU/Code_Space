#!/usr/bin/env python3
"""No-device ROS runtime probe for nav_safety_gate."""

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from drive_pkg.nav_safety_gate import NavSafetyGateNode


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def main():
    rclpy.init()
    gate = NavSafetyGateNode()
    probe = Node("nav_safety_gate_contract_probe")
    command_pub = probe.create_publisher(Twist, "/cmd_vel", 10)
    scan_pub = probe.create_publisher(LaserScan, "/scan", 10)
    odom_pub = probe.create_publisher(Odometry, "/odom", 10)
    drive_pub = probe.create_publisher(Bool, "/drive/ready", 10)
    stop_pub = probe.create_publisher(Bool, "/nav_safety/stop", 10)
    outputs = []
    probe.create_subscription(Twist, "/cmd_vel_safe", lambda msg: outputs.append(msg), 10)

    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(probe)
    try:
        # Let DDS discovery match local publishers/subscribers before assertions.
        spin_for(executor, 0.8)
        command = Twist()
        command.linear.x = 0.1
        command_pub.publish(command)
        spin_for(executor, 0.4)
        assert outputs and outputs[-1].linear.x == 0.0, "command passed before sensors"

        ready = Bool()
        ready.data = True
        for _ in range(5):
            scan_pub.publish(LaserScan())
            odom_pub.publish(Odometry())
            drive_pub.publish(ready)
            command_pub.publish(command)
            spin_for(executor, 0.08)
        assert any(abs(msg.linear.x - 0.1) < 1e-9 for msg in outputs), "fresh command not passed"

        stopped = Bool()
        stopped.data = True
        stop_pub.publish(stopped)
        spin_for(executor, 0.15)
        assert outputs[-1].linear.x == 0.0, "software stop did not zero output"
        print("nav safety gate ROS runtime probe passed")
    finally:
        gate.publish_zero()
        executor.remove_node(probe)
        executor.remove_node(gate)
        probe.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
