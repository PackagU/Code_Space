#!/usr/bin/env python3
"""Probe the P05 safe ROS runtime without opening or touching a serial device."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__("arm_node_safe_probe")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.events = []
        self.create_subscription(String, "/packagu_arm/status", self._on_status, qos)
        self.publisher = self.create_publisher(String, "/packagu_arm/command", 10)

    def _on_status(self, msg):
        self.events.append(json.loads(msg.data))


def spin_until(node, predicate, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError("arm status condition timed out")


def main():
    rclpy.init()
    node = Probe()
    try:
        spin_until(node, lambda: any(event["event"] == "ready" for event in node.events), 5.0)
        ready = node.events[-1]
        assert ready["state"] == "idle"
        assert ready["hardware_connected"] is False
        assert ready["homed"] is False
        command = String()
        command.data = json.dumps({"request_id": "offline-home", "action": "home"})
        node.publisher.publish(command)
        spin_until(
            node,
            lambda: any(event.get("request_id") == "offline-home" for event in node.events),
            5.0,
        )
        result = next(event for event in node.events if event.get("request_id") == "offline-home")
        assert result["event"] == "failed" and result["state"] == "failed"
        assert result["error"] == "hardware_unavailable"
        assert result["completion_basis"] == "none"
        assert not any(event["state"] == "completed" for event in node.events)
        print("PASS arm ROS safe runtime: no driver, no startup home, hardware request failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
