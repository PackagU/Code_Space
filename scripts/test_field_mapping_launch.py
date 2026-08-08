#!/usr/bin/env python3
"""실기 매핑 launch 계약 검사 (스펙 §5.4 '매핑 launch 정비')."""
from pathlib import Path
import ast
import sys

ROOT = Path(__file__).resolve().parents[1]
SLAM_LAUNCH = ROOT / "src/slam_pkg/launch/slam_toolbox.launch.py"
SIM_LAUNCH = ROOT / "src/slam_pkg/launch/kku_simulation.launch.py"
DRIVE_LAUNCH = ROOT / "src/drive_pkg/launch/drive_bringup.launch.py"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def main():
    for path in (SLAM_LAUNCH, SIM_LAUNCH, DRIVE_LAUNCH):
        require(path.exists(), f"missing {path}")
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    slam_src = SLAM_LAUNCH.read_text(encoding="utf-8")
    for needle in [
        "robot_state_publisher",          # 실기 TF 체인 (RSP)
        "delivery_robot.urdf.xacro",      # URDF 로드
        "UnlessCondition",                # 시뮬에서는 RSP 중복 기동 금지
        '"/dev/rplidar"',                 # udev 별칭 기본값
        '"rviz"',                         # RViz 조건부 인자
        '"enable_lidar"',
        '"enable_drive"',
        "drive_bringup.launch.py",        # 드라이브 include
    ]:
        require(needle in slam_src, f"slam_toolbox.launch.py missing: {needle}")
    require('default_value="false"' in slam_src, "rviz must default to false")

    sim_src = SIM_LAUNCH.read_text(encoding="utf-8")
    require('"rviz": "true"' in sim_src, "sim must keep RViz on (regression guard)")

    drive_src = DRIVE_LAUNCH.read_text(encoding="utf-8")
    require("DeclareLaunchArgument" in drive_src and '"serial_port"' in drive_src,
            "drive serial port must be a launch argument (portability R3)")

    print("field mapping launch contract passed")


if __name__ == "__main__":
    main()
