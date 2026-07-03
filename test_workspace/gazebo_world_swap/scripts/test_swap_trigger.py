#!/usr/bin/env python3
"""Offline checks for world-swap trigger selection and dedupe."""

from __future__ import annotations

import json
import sys
from pathlib import Path


def _add_source_path():
    repo_root = Path(__file__).resolve().parents[3]
    package_root = repo_root / "test_workspace" / "gazebo_world_swap" / "src" / "gazebo_world_swap_pkg"
    sys.path.insert(0, str(package_root))


def _status(**overrides):
    payload = {
        "phase": "ready",
        "pending": False,
        "map_loaded": True,
        "current_floor": "F2",
        "target_floor": "F2",
        "spawn_point_id": "elevator_inside",
    }
    payload.update(overrides)
    return json.dumps(payload)


def main():
    _add_source_path()

    from gazebo_world_swap_pkg.swap_trigger import WorldSwapTrigger

    trigger = WorldSwapTrigger(initial_floor="F1")

    assert trigger.observe(_status(phase="waiting_elevator", pending=True)) is None
    assert trigger.observe(_status(map_loaded=False)) is None
    assert trigger.observe(_status(phase="failed", pending=True, map_loaded=False)) is None
    assert trigger.observe("not-json") is None

    request = trigger.observe(_status())
    assert request is not None
    assert request.source_floor == "F1"
    assert request.target_floor == "F2"
    assert request.target_model == "kku_f2_building"
    assert request.spawn_point_id == "elevator_inside"

    assert trigger.observe(_status()) is None

    next_request = trigger.observe(_status(current_floor="F3", target_floor="F3"))
    assert next_request is not None
    assert next_request.source_floor == "F2"
    assert next_request.target_floor == "F3"
    assert next_request.target_model == "kku_f3_building"

    print("PASS world swap trigger")


if __name__ == "__main__":
    main()
