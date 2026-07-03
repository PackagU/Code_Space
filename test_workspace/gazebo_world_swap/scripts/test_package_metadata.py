#!/usr/bin/env python3
"""Checks for gazebo_world_swap_pkg package metadata."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


def main():
    repo_root = Path(__file__).resolve().parents[3]
    package_root = repo_root / "test_workspace" / "gazebo_world_swap" / "src" / "gazebo_world_swap_pkg"
    package_xml = package_root / "package.xml"
    setup_py = package_root / "setup.py"
    launch_file = package_root / "launch" / "world_swap.launch.py"

    assert package_xml.exists(), "package.xml missing"
    assert setup_py.exists(), "setup.py missing"
    assert launch_file.exists(), "world_swap.launch.py missing"

    package = ET.parse(package_xml).getroot()
    assert package.findtext("name") == "gazebo_world_swap_pkg"
    deps = {node.text for node in package.findall("depend")}
    assert {"rclpy", "std_msgs", "gazebo_msgs", "geometry_msgs"}.issubset(deps)

    setup_text = setup_py.read_text(encoding="utf-8")
    assert "world_swap_node = gazebo_world_swap_pkg.world_swap_node:main" in setup_text

    print("PASS gazebo world swap package metadata")


if __name__ == "__main__":
    main()
