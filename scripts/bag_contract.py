#!/usr/bin/env python3
"""Validate rosbag2 metadata and select a motion-safe replay topic set."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml


REPLAY_PROFILES = {
    "reslam": ("/scan", "/scan_raw", "/odom", "/imu", "/tf", "/tf_static"),
    "inspect": (
        "/scan",
        "/scan_raw",
        "/odom",
        "/imu",
        "/tf",
        "/tf_static",
        "/diagnostics",
        "/camera/image_raw",
        "/camera/camera_info",
    ),
}
FORBIDDEN_REPLAY_PREFIXES = (
    "/cmd_vel",
    "/arm",
    "/lift",
    "/servo",
    "/drive/command",
    "/navigate_to_pose",
    "/goal_pose",
)


class BagContractError(ValueError):
    pass


def _bag_root(raw_path: str | Path) -> Path:
    root = Path(raw_path).expanduser().resolve()
    if not root.is_dir():
        raise BagContractError(f"bag directory does not exist: {root}")
    return root


def load_summary(raw_path: str | Path) -> dict:
    root = _bag_root(raw_path)
    metadata_path = root / "metadata.yaml"
    if not metadata_path.is_file():
        raise BagContractError(f"metadata.yaml missing: {root}")
    try:
        raw = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        info = raw["rosbag2_bagfile_information"]
    except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise BagContractError(f"invalid rosbag metadata: {exc}") from exc

    duration_ns = int((info.get("duration") or {}).get("nanoseconds", 0))
    if duration_ns <= 0:
        raise BagContractError("bag duration must be greater than zero")

    topics = {}
    for item in info.get("topics_with_message_count") or []:
        metadata = item.get("topic_metadata") or {}
        name = str(metadata.get("name", ""))
        msg_type = str(metadata.get("type", ""))
        count = int(item.get("message_count", 0))
        if not name.startswith("/") or not msg_type:
            raise BagContractError("topic metadata is missing name or type")
        if name in topics:
            raise BagContractError(f"duplicate topic metadata: {name}")
        topics[name] = {"type": msg_type, "count": count}

    files = []
    actual_bytes = metadata_path.stat().st_size
    for relative in info.get("relative_file_paths") or []:
        rel = Path(str(relative))
        if rel.is_absolute() or ".." in rel.parts:
            raise BagContractError(f"unsafe relative file path: {relative}")
        candidate = (root / rel).resolve()
        try:
            candidate.relative_to(root)
        except ValueError as exc:
            raise BagContractError(f"bag file escapes directory: {relative}") from exc
        if not candidate.is_file() or candidate.stat().st_size <= 0:
            raise BagContractError(f"bag storage file missing or empty: {relative}")
        files.append(str(rel))
        actual_bytes += candidate.stat().st_size
    if not files:
        raise BagContractError("no rosbag storage files listed")

    declared_count = int(info.get("message_count", 0))
    summed_count = sum(topic["count"] for topic in topics.values())
    if declared_count != summed_count:
        raise BagContractError(
            f"message count mismatch: declared={declared_count}, topics={summed_count}"
        )

    return {
        "bag": str(root),
        "duration_sec": duration_ns / 1_000_000_000,
        "message_count": declared_count,
        "actual_bytes": actual_bytes,
        "storage_identifier": str(info.get("storage_identifier", "")),
        "files": files,
        "topics": topics,
    }


def require_topics(summary: dict, required: list[str]) -> None:
    missing = [name for name in required if summary["topics"].get(name, {}).get("count", 0) <= 0]
    if missing:
        raise BagContractError("required topics missing or empty: " + ", ".join(missing))


def replay_topics(summary: dict, profile: str) -> list[str]:
    if profile not in REPLAY_PROFILES:
        raise BagContractError(f"unknown replay profile: {profile}")
    selected = [name for name in REPLAY_PROFILES[profile] if summary["topics"].get(name, {}).get("count", 0) > 0]
    if any(name.startswith(FORBIDDEN_REPLAY_PREFIXES) for name in selected):
        raise BagContractError("internal error: control topic entered replay allowlist")
    for mandatory in ("/scan", "/odom", "/tf", "/tf_static"):
        if mandatory not in selected:
            raise BagContractError(f"safe replay topic missing: {mandatory}")
    return selected


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    inspect_parser = sub.add_parser("inspect")
    inspect_parser.add_argument("bag")
    inspect_parser.add_argument("--require", action="append", default=[])
    replay_parser = sub.add_parser("replay-topics")
    replay_parser.add_argument("bag")
    replay_parser.add_argument("--profile", choices=tuple(REPLAY_PROFILES), default="reslam")
    args = parser.parse_args()
    try:
        summary = load_summary(args.bag)
        if args.command == "inspect":
            require_topics(summary, args.require)
            print(json.dumps(summary, indent=2, sort_keys=True))
        else:
            print(" ".join(replay_topics(summary, args.profile)))
    except BagContractError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
