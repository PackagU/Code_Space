#!/usr/bin/env python3
"""Saved-map Nav2 launch contract for simulation and physical use."""

import ast
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src/slam_pkg/launch/kku_navigation.launch.py"
PARAMS = ROOT / "src/slam_pkg/config/nav2_params.yaml"
PACKAGE = ROOT / "src/slam_pkg/package.xml"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def main():
    source = LAUNCH.read_text(encoding="utf-8")
    ast.parse(source, filename=str(LAUNCH))
    for needle in (
        "nav2_bringup", "bringup_launch.py", '"map",',
        "validate_map_yaml", "physical Nav2 requires map", 'default_value="false"',
        '"use_composition": "False"', '"slam": "False"',
    ):
        require(needle in source, f"navigation launch missing {needle!r}")
    for forbidden in ("rplidar_ros", "opencr_bridge", "robot_arm_pkg", "async_slam_toolbox_node"):
        require(forbidden not in source, f"navigation launch must not start {forbidden}")

    params = PARAMS.read_text(encoding="utf-8")
    for needle in (
        "desired_linear_vel: 0.10", "max_velocity: [0.10, 0.0, 0.35]",
        "allow_unknown: false", "robot_base_frame: base_footprint",
    ):
        require(needle in params, f"safe Nav2 params missing {needle!r}")
    package = PACKAGE.read_text(encoding="utf-8")
    require("<exec_depend>nav2_bringup</exec_depend>" in package, "nav2 dependency missing")
    require("<exec_depend>python3-yaml</exec_depend>" in package, "YAML dependency missing")
    print("saved-map navigation launch contract passed")


if __name__ == "__main__":
    main()
