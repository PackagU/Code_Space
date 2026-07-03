#!/usr/bin/env python3
"""Offline checks for common_pkg gazebo.launch.py spawn-point selection."""

from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_gazebo_launch():
    repo_root = Path(__file__).resolve().parents[3]
    launch_path = repo_root / "src" / "common_pkg" / "launch" / "gazebo.launch.py"
    spec = importlib.util.spec_from_file_location("gazebo_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    gazebo_launch = _load_gazebo_launch()

    assert gazebo_launch.resolve_spawn_pose("F1", "elevator_exit") == (1.6, 0.0, 0.0)
    assert gazebo_launch.resolve_spawn_pose("F1", "charge_station") == (1.6, 0.0, 0.0)
    assert gazebo_launch.resolve_spawn_pose("f2", "elevator_inside") == (0.0, 0.0, 0.0)

    try:
        gazebo_launch.resolve_spawn_pose("F4", "elevator_exit")
    except RuntimeError as exc:
        assert "floor must be one of" in str(exc)
    else:
        raise AssertionError("unknown floor should fail")

    try:
        gazebo_launch.resolve_spawn_pose("F1", "unknown_point")
    except RuntimeError as exc:
        assert "spawn_point must be one of" in str(exc)
    else:
        raise AssertionError("unknown spawn_point should fail")

    print("PASS gazebo launch spawn_point resolver")


if __name__ == "__main__":
    main()
