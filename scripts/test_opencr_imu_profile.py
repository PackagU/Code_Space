#!/usr/bin/env python3
"""P2 IMU candidate static contract; no serial port or ROS graph is opened."""

import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVE = ROOT / "src" / "drive_pkg"
SLAM = ROOT / "src" / "slam_pkg"
FIRMWARE = DRIVE / "firmware" / "opencr_imu" / "opencr_drive_bridge_imu" / "opencr_drive_bridge_imu.ino"


def main():
    firmware = FIRMWARE.read_text(encoding="utf-8")
    for required in (
        "HELLO opencr 0.3-imu",
        "IMU.bConnected",
        "IMU.update()",
        "GYRO_FACTOR = 0.0010642",
        "ACCEL_FACTOR = 0.000598550415",
        "getLastLibErrCode()",
        "left_error != DXL_LIB_OK",
        "USB-only/motor-fault mode: never fabricate F 0 0",
        "readHostCommands();",
        "COMMAND_TIMEOUT_MS = 500",
        "FEEDBACK_PERIOD_MS = 20",
    ):
        assert required in firmware, required
    assert "while (true)" not in firmware
    assert firmware.index("readHostCommands();") < firmware.index("publishFeedback();")

    # The field default remains the old wheel-only bringup.
    field_launch = (SLAM / "launch" / "field_base.launch.py").read_text(encoding="utf-8")
    ast.parse(field_launch)
    assert 'DeclareLaunchArgument("odometry_profile", default_value="wheel_only")' in field_launch
    assert "wheel_imu requires imu_mount_verified:=true" in field_launch
    assert 'drive_launch = "drive_bringup.launch.py"' in field_launch

    imu_launch = (DRIVE / "launch" / "drive_imu_bringup.launch.py").read_text(encoding="utf-8")
    ast.parse(imu_launch)
    assert '("/odom", "/wheel/odom")' in imu_launch
    assert '("/odometry/filtered", "/odom")' in imu_launch
    assert '"publish_tf": False' in imu_launch
    assert '"--child-frame-id", "imu_link"' in imu_launch

    config = yaml.safe_load((DRIVE / "config" / "drive_ekf.yaml").read_text(encoding="utf-8"))
    params = config["ekf_filter_node"]["ros__parameters"]
    assert params["two_d_mode"] is True and params["publish_tf"] is True
    assert params["world_frame"] == "odom" and params["base_link_frame"] == "base_footprint"
    assert params["odom0"] == "/wheel/odom" and params["imu0"] == "/imu"
    assert params["odom0_config"] == [False] * 6 + [True, False, False, False, False, True] + [False] * 3
    assert params["imu0_config"] == [False] * 11 + [True] + [False] * 3

    # docker/ is not bind-mounted into the runtime container; host execution
    # still enforces image persistence when that source tree is available.
    dockerfile_path = ROOT / "docker" / "Dockerfile.jetson"
    if dockerfile_path.exists():
        dockerfile = dockerfile_path.read_text(encoding="utf-8")
        assert "ros-humble-robot-localization" in dockerfile
    print("opencr_imu_profile tests passed")


if __name__ == "__main__":
    main()
