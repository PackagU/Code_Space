#!/usr/bin/env python3
"""Contract checks for the saved-map Nav2 launch file."""
from pathlib import Path
import ast
import sys


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = ROOT / "src/slam_pkg/launch/kku_navigation.launch.py"
PACKAGE_XML = ROOT / "src/slam_pkg/package.xml"
NAV2_PARAMS = ROOT / "src/slam_pkg/config/nav2_params.yaml"
MAPS = {
    "F1": ROOT / "src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml",
    "F2": ROOT / "src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml",
    "F3": ROOT / "src/slam_pkg/maps/kku_virtual/f3/kku_f3.yaml",
}


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def main():
    require(LAUNCH_FILE.exists(), f"missing {LAUNCH_FILE}")
    require(NAV2_PARAMS.exists(), f"missing {NAV2_PARAMS}")
    for floor, path in MAPS.items():
        require(path.exists(), f"missing {floor} map yaml: {path}")

    source = LAUNCH_FILE.read_text(encoding="utf-8")
    ast.parse(source, filename=str(LAUNCH_FILE))

    for needle in [
        "nav2_bringup",
        "bringup_launch.py",
        "kku_f1.yaml",
        "kku_f2.yaml",
        "kku_f3.yaml",
        "nav2_params.yaml",
        "use_sim_time",
        "rviz",
    ]:
        require(needle in source, f"{LAUNCH_FILE} does not contain {needle!r}")

    package_xml = PACKAGE_XML.read_text(encoding="utf-8")
    require(
        "<exec_depend>nav2_bringup</exec_depend>" in package_xml,
        "package.xml is missing nav2_bringup exec_depend",
    )

    print("PASS: kku_navigation.launch.py contract")


if __name__ == "__main__":
    main()
