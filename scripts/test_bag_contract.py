#!/usr/bin/env python3
"""Offline tests for rosbag inspection and safe replay selection."""

import importlib.util
from pathlib import Path
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "scripts/bag_contract.py"


def load_module():
    spec = importlib.util.spec_from_file_location("bag_contract", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_bag(root: Path, *, duration=2_000_000_000, escape=False):
    topics = {
        "/scan": ("sensor_msgs/msg/LaserScan", 20),
        "/odom": ("nav_msgs/msg/Odometry", 20),
        "/imu": ("sensor_msgs/msg/Imu", 20),
        "/tf": ("tf2_msgs/msg/TFMessage", 20),
        "/tf_static": ("tf2_msgs/msg/TFMessage", 1),
        "/cmd_vel": ("geometry_msgs/msg/Twist", 5),
    }
    storage = root / "bag_0.db3"
    storage.write_bytes(b"sqlite-placeholder")
    metadata = {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "sqlite3",
            "duration": {"nanoseconds": duration},
            "starting_time": {"nanoseconds_since_epoch": 1},
            "message_count": sum(count for _type, count in topics.values()),
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": name,
                        "type": msg_type,
                        "serialization_format": "cdr",
                        "offered_qos_profiles": "",
                    },
                    "message_count": count,
                }
                for name, (msg_type, count) in topics.items()
            ],
            "relative_file_paths": ["../outside.db3" if escape else storage.name],
        }
    }
    (root / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")


def main():
    mod = load_module()
    with tempfile.TemporaryDirectory() as temp_dir:
        bag = Path(temp_dir) / "bag"
        bag.mkdir()
        write_bag(bag)
        summary = mod.load_summary(bag)
        mod.require_topics(summary, ["/scan", "/odom", "/tf", "/tf_static"])
        selected = mod.replay_topics(summary, "reslam")
        assert "/cmd_vel" not in selected
        assert selected == ["/scan", "/odom", "/imu", "/tf", "/tf_static"]
        assert summary["duration_sec"] == 2.0 and summary["actual_bytes"] > 0

        missing = Path(temp_dir) / "missing"
        missing.mkdir()
        write_bag(missing)
        data = yaml.safe_load((missing / "metadata.yaml").read_text(encoding="utf-8"))
        data["rosbag2_bagfile_information"]["topics_with_message_count"] = []
        data["rosbag2_bagfile_information"]["message_count"] = 0
        (missing / "metadata.yaml").write_text(yaml.safe_dump(data), encoding="utf-8")
        try:
            mod.require_topics(mod.load_summary(missing), ["/scan"])
        except mod.BagContractError:
            pass
        else:
            raise AssertionError("empty required topic accepted")

        unsafe = Path(temp_dir) / "unsafe"
        unsafe.mkdir()
        write_bag(unsafe, escape=True)
        try:
            mod.load_summary(unsafe)
        except mod.BagContractError:
            pass
        else:
            raise AssertionError("path traversal accepted")

    print("P07 bag contract tests passed")


if __name__ == "__main__":
    main()
