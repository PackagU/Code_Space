#!/usr/bin/env python3
"""장치 없이 Jetson 배포 이미지의 런타임 의존성을 검증한다."""

from __future__ import annotations

import importlib
import os
import shutil
import subprocess


def main() -> None:
    modules = ("cv2", "numpy", "serial", "rclpy", "yaml")
    for name in modules:
        module = importlib.import_module(name)
        version = getattr(module, "__version__", "available")
        print(f"module {name}={version}")

    required_directories = (
        "/ros2_ws/src",
        "/ros2_ws/maps",
        "/ros2_ws/logs",
        "/ros2_ws/test_workspace",
        "/ros2_ws/scripts",
        "/opt/floor_reader/data",
    )
    for path in required_directories:
        if not os.path.isdir(path):
            raise RuntimeError(f"required directory missing: {path}")

    if shutil.which("rviz2") is None:
        raise RuntimeError("rviz2 executable missing")

    for package in ("rplidar_ros", "slam_toolbox", "nav2_bringup"):
        subprocess.run(
            ("ros2", "pkg", "prefix", package),
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        print(f"ros_package {package}=available")

    serial_devices = ("/dev/rplidar", "/dev/opencr", "/dev/arm_servo", "/dev/motor_nano")
    present = [path for path in serial_devices if os.path.exists(path)]
    if present:
        raise RuntimeError(f"device-less probe unexpectedly received devices: {present}")

    print("jetson_image_runtime=PASS")


if __name__ == "__main__":
    main()
