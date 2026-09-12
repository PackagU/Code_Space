#!/usr/bin/env python3
"""Measure ROS sensor arrival gaps and header age without publishing commands."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time


class ArrivalSeries:
    def __init__(self):
        self.arrivals_ns = []
        self.ages_ms = []
        self.header_regressions = 0
        self._last_header_ns = None

    def add(self, arrival_ns: int, header_ns: int | None = None):
        if self.arrivals_ns and arrival_ns < self.arrivals_ns[-1]:
            raise ValueError("arrival clock regressed")
        self.arrivals_ns.append(arrival_ns)
        if header_ns and header_ns > 0:
            if self._last_header_ns is not None and header_ns < self._last_header_ns:
                self.header_regressions += 1
            self._last_header_ns = header_ns
            age_ms = (arrival_ns - header_ns) / 1_000_000
            if math.isfinite(age_ms):
                self.ages_ms.append(age_ms)

    @staticmethod
    def _percentile(values, percentile):
        if not values:
            return None
        ordered = sorted(values)
        index = (len(ordered) - 1) * percentile / 100
        lower = math.floor(index)
        upper = math.ceil(index)
        if lower == upper:
            return ordered[lower]
        return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)

    def report(self):
        gaps_ms = [
            (right - left) / 1_000_000
            for left, right in zip(self.arrivals_ns, self.arrivals_ns[1:])
        ]
        span_sec = (
            (self.arrivals_ns[-1] - self.arrivals_ns[0]) / 1_000_000_000
            if len(self.arrivals_ns) > 1
            else 0.0
        )
        return {
            "count": len(self.arrivals_ns),
            "measured_rate_hz": (len(gaps_ms) / span_sec) if span_sec > 0 else 0.0,
            "gap_ms_p50": self._percentile(gaps_ms, 50),
            "gap_ms_p95": self._percentile(gaps_ms, 95),
            "gap_ms_p99": self._percentile(gaps_ms, 99),
            "gap_ms_max": max(gaps_ms) if gaps_ms else None,
            "age_ms_p95": self._percentile(self.ages_ms, 95),
            "age_ms_p99": self._percentile(self.ages_ms, 99),
            "age_ms_max": max(self.ages_ms) if self.ages_ms else None,
            "header_regressions": self.header_regressions,
        }


def _header_ns(message):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def run_ros(duration_sec: float, topics: list[str], output: Path):
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu, LaserScan

    topic_types = {
        "/scan": LaserScan,
        "/scan_raw": LaserScan,
        "/odom": Odometry,
        "/imu": Imu,
        "/camera/image_raw": Image,
    }
    unknown = [topic for topic in topics if topic not in topic_types]
    if unknown:
        raise ValueError("unsupported metric topics: " + ", ".join(unknown))

    rclpy.init()
    node = rclpy.create_node("packagu_field_topic_metrics")
    series = {topic: ArrivalSeries() for topic in topics}

    def callback(topic):
        def receive(message):
            series[topic].add(node.get_clock().now().nanoseconds, _header_ns(message))

        return receive

    subscriptions = [
        node.create_subscription(topic_types[topic], topic, callback(topic), 50)
        for topic in topics
    ]
    _ = subscriptions
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        report = {
            "duration_sec": time.monotonic() - start,
            "topics": {topic: metric.report() for topic, metric in series.items()},
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--topics", default="/scan,/odom,/imu")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0:
        parser.error("duration-sec must be positive")
    topics = [topic.strip() for topic in args.topics.split(",") if topic.strip()]
    report = run_ros(args.duration_sec, topics, args.output)
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
