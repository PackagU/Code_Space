#!/usr/bin/env python3
"""P03 DDS 계약 프로브: 늦은 구독, QoS, 서비스, tf_static, map을 검사한다."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Publisher(Node):
    def __init__(self, token: str) -> None:
        super().__init__(f"p03_publisher_{token.lower()}")
        self.token = token
        self.string_pub = self.create_publisher(String, "/p03/string", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/p03/scan", qos_profile_sensor_data)
        self.tf_pub = self.create_publisher(TFMessage, "/p03/tf_static", LATCHED_QOS)
        self.map_pub = self.create_publisher(OccupancyGrid, "/p03/map", LATCHED_QOS)
        self.create_service(Trigger, "/p03/ping", self.on_ping)
        self.create_timer(0.1, self.publish_volatile)
        self.publish_latched()

    def on_ping(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        response.message = self.token
        return response

    def publish_volatile(self) -> None:
        text = String()
        text.data = self.token
        self.string_pub.publish(text)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f"p03_laser_{self.token}"
        scan.angle_min = 0.0
        scan.angle_max = 0.1
        scan.angle_increment = 0.1
        scan.range_min = 0.1
        scan.range_max = 8.0
        scan.ranges = [1.0, 1.1]
        self.scan_pub.publish(scan)

    def publish_latched(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = f"p03_parent_{self.token}"
        transform.child_frame_id = f"p03_child_{self.token}"
        transform.transform.rotation.w = 1.0
        self.tf_pub.publish(TFMessage(transforms=[transform]))

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = f"p03_map_{self.token}"
        grid.info.resolution = 0.05
        grid.info.width = 1
        grid.info.height = 1
        grid.info.origin.orientation.w = 1.0
        grid.data = [0]
        self.map_pub.publish(grid)


class Subscriber(Node):
    def __init__(self, token: str) -> None:
        super().__init__(f"p03_subscriber_{token.lower()}")
        self.token = token
        self.received = {"string": False, "scan": False, "tf_static": False, "map": False}
        self.create_subscription(String, "/p03/string", self.on_string, 10)
        self.create_subscription(LaserScan, "/p03/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/p03/tf_static", self.on_tf, LATCHED_QOS)
        self.create_subscription(OccupancyGrid, "/p03/map", self.on_map, LATCHED_QOS)
        self.client = self.create_client(Trigger, "/p03/ping")

    def on_string(self, msg: String) -> None:
        self.received["string"] |= msg.data == self.token

    def on_scan(self, msg: LaserScan) -> None:
        self.received["scan"] |= msg.header.frame_id == f"p03_laser_{self.token}"

    def on_tf(self, msg: TFMessage) -> None:
        self.received["tf_static"] |= any(
            item.child_frame_id == f"p03_child_{self.token}" for item in msg.transforms
        )

    def on_map(self, msg: OccupancyGrid) -> None:
        self.received["map"] |= msg.header.frame_id == f"p03_map_{self.token}"


def run_publisher(token: str) -> int:
    rclpy.init()
    node = Publisher(token)
    print(f"publisher_ready token={token}", flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def run_subscriber(token: str, timeout_sec: float) -> int:
    rclpy.init()
    node = Subscriber(token)
    deadline = time.monotonic() + timeout_sec
    future = None
    try:
        while time.monotonic() < deadline and not node.client.wait_for_service(timeout_sec=0.2):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.client.service_is_ready():
            print("FAIL service unavailable", file=sys.stderr)
            return 1
        future = node.client.call_async(Trigger.Request())
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            service_ok = (
                future.done()
                and future.exception() is None
                and future.result().success
                and future.result().message == token
            )
            if service_ok and all(node.received.values()):
                print(f"subscriber_PASS token={token} received={node.received} service=True")
                return 0
        service_result = None if future is None or not future.done() else future.result()
        print(
            f"FAIL token={token} received={node.received} service={service_result}",
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("role", choices=("publisher", "subscriber"))
    parser.add_argument("--token", required=True)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    args = parser.parse_args()
    if args.role == "publisher":
        return run_publisher(args.token)
    return run_subscriber(args.token, args.timeout_sec)


if __name__ == "__main__":
    raise SystemExit(main())
