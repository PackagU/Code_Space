#!/usr/bin/env python3
"""Static contract: persistent base is separated from SLAM and saved-map Nav2."""

import ast
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "src/slam_pkg/launch/field_base.launch.py"
MAPPING = ROOT / "src/slam_pkg/launch/field_mapping_only.launch.py"
DRIVE = ROOT / "src/drive_pkg/launch/drive_bringup.launch.py"
URDF = ROOT / "src/common_pkg/urdf/delivery_robot.urdf.xacro"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def main():
    for path in (BASE, MAPPING, DRIVE):
        require(path.is_file(), f"missing {path}")
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    base = BASE.read_text(encoding="utf-8")
    for needle in (
        "robot_state_publisher", "rplidar_composition", "drive_bringup.launch.py",
        "nav_safety_gate", "/cmd_vel_safe", 'default_value="false"',
        "laser_x", "laser_y", "laser_z", "enable_drive",
    ):
        require(needle in base, f"field base missing {needle!r}")
    for forbidden in ("robot_arm_pkg", "arm_sequence", "lift_node"):
        require(forbidden not in base, f"field base must exclude {forbidden}")

    mapping = MAPPING.read_text(encoding="utf-8")
    require("async_slam_toolbox_node" in mapping, "mapping-only launch must start SLAM")
    for forbidden in ("rplidar_ros", "drive_pkg", "opencr_bridge", "robot_state_publisher", "robot_arm_pkg"):
        require(forbidden not in mapping, f"mapping-only launch must not start {forbidden}")

    drive = DRIVE.read_text(encoding="utf-8")
    require('"cmd_vel_topic"' in drive, "drive launch must expose cmd_vel_topic")

    urdf = URDF.read_text(encoding="utf-8")
    for needle in ('name="laser_x"', 'name="laser_z"', '$(arg laser_yaw)'):
        require(needle in urdf, f"URDF missing LiDAR override {needle}")
    print("field mapping launch contract passed")


if __name__ == "__main__":
    main()
