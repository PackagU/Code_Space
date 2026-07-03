#!/usr/bin/env python3
"""Headless verifier for the map + Gazebo world swap smoke.

기본값은 F1 -> F2 전환 검증. --from-floor / --floor 로 임의의 전환
(예: F2 -> F3)을 검증할 수 있다. 맵 크기/원점 기대값은 하드코딩하지 않고
map_expectations.py(커밋된 맵 yaml + pgm 헤더)에서 읽는다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from gazebo_msgs.srv import GetModelList
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from map_expectations import load_expectation


def building_model(floor: str) -> str:
    return f"kku_{floor.lower()}_building"


class WorldSwapVerifier(Node):
    def __init__(self, target_floor: str, expected_map: dict):
        super().__init__("gazebo_world_swap_verifier")
        self.target_floor = target_floor
        self.expected_map = expected_map
        self.floor_status = None
        self.swap_status = None
        self.map_info = None
        self.scan_finite = False
        transient_qos = QoSProfile(depth=10)
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, "/floor_orchestrator/status", self._on_floor_status, 10
        )
        self.create_subscription(
            String, "/gazebo_world_swap/status", self._on_swap_status, transient_qos
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, transient_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.model_list_client = self.create_client(GetModelList, "/get_model_list")

    def _on_floor_status(self, msg):
        try:
            self.floor_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_swap_status(self, msg):
        try:
            self.swap_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_map(self, msg):
        self.map_info = msg.info

    def _on_scan(self, msg):
        self.scan_finite = any(
            math.isfinite(value) and msg.range_min <= value <= msg.range_max
            for value in msg.ranges
        )

    def floor_ready(self):
        status = self.floor_status or {}
        return (
            status.get("phase") == "ready"
            and status.get("pending") is False
            and status.get("current_floor") == self.target_floor
            and status.get("map_loaded") is True
        )

    def swap_ready(self):
        status = self.swap_status or {}
        return (
            status.get("state") == "swapped"
            and status.get("target_floor") == self.target_floor
            and status.get("method") == "model_swap"
        )

    def map_matches_floor(self):
        info = self.map_info
        if info is None:
            return False
        expected = self.expected_map
        return (
            info.width == expected["width"]
            and info.height == expected["height"]
            and abs(info.origin.position.x - expected["origin_x"]) < 0.02
            and abs(info.origin.position.y - expected["origin_y"]) < 0.02
        )

    def model_names(self, timeout_sec=5.0):
        if not self.model_list_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/get_model_list service unavailable")
        future = self.model_list_client.call_async(GetModelList.Request())
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                result = future.result()
                if result is None:
                    return []
                return list(result.model_names)
        raise RuntimeError("/get_model_list timeout")


def wait_for_conditions(node, timeout_sec):
    end = time.monotonic() + timeout_sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (
            node.floor_ready()
            and node.swap_ready()
            and node.map_matches_floor()
            and node.scan_finite
        ):
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--floor", default="F2", help="전환 목표 층 (기본 F2)")
    parser.add_argument(
        "--from-floor", default="F1", help="전환 전 층 — 이 층의 건물 모델이 사라져야 함"
    )
    args = parser.parse_args()

    expected = load_expectation(args.floor)

    rclpy.init()
    node = WorldSwapVerifier(args.floor, expected)
    try:
        if not wait_for_conditions(node, args.timeout_sec):
            print("FAIL world swap conditions not met", file=sys.stderr)
            print(f"floor_status={node.floor_status}", file=sys.stderr)
            print(f"swap_status={node.swap_status}", file=sys.stderr)
            print(f"map_info={node.map_info}", file=sys.stderr)
            print(f"scan_finite={node.scan_finite}", file=sys.stderr)
            return 1

        models = set(node.model_names())
        target_model = building_model(args.floor)
        previous_model = building_model(args.from_floor)
        target_present = target_model in models
        previous_present = previous_model in models
        if not target_present or previous_present:
            print(
                "FAIL Gazebo entity state mismatch: "
                f"{target_model}={target_present}, {previous_model}={previous_present}",
                file=sys.stderr,
            )
            return 1

        print("PASS world swap smoke")
        print(
            f"status phase=ready pending=false current_floor={args.floor} map_loaded=true"
        )
        print(
            f"map width={expected['width']} height={expected['height']} "
            f"origin={expected['origin_x']},{expected['origin_y']}"
        )
        print(f"entity {target_model} present")
        print(f"entity {previous_model} absent")
        print("/scan has finite ranges")
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
