#!/usr/bin/env python3
"""Offline contract checks for world_swap_node helper functions."""

from __future__ import annotations

import json
import sys
from pathlib import Path


def _add_source_path():
    repo_root = Path(__file__).resolve().parents[3]
    package_root = repo_root / "test_workspace" / "gazebo_world_swap" / "src" / "gazebo_world_swap_pkg"
    sys.path.insert(0, str(package_root))


def main():
    _add_source_path()

    from gazebo_world_swap_pkg.world_swap_node import format_restart_command, status_json

    template = (
        "ros2 launch common_pkg gazebo.launch.py "
        "floor:={floor} spawn_point:=elevator_inside use_sim_time:=true"
    )
    assert format_restart_command(template, "f2") == [
        "ros2",
        "launch",
        "common_pkg",
        "gazebo.launch.py",
        "floor:=F2",
        "spawn_point:=elevator_inside",
        "use_sim_time:=true",
    ]

    payload = json.loads(
        status_json(
            state="swapped",
            method="model_swap",
            source_floor="F1",
            target_floor="F2",
            detail="spawned kku_f2_building",
        )
    )
    assert payload == {
        "detail": "spawned kku_f2_building",
        "method": "model_swap",
        "source_floor": "F1",
        "state": "swapped",
        "target_floor": "F2",
    }

    print("PASS world swap node contract")


if __name__ == "__main__":
    main()
