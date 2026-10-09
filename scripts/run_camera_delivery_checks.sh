#!/usr/bin/env bash
# Hardware-free regression checks, with a real ROS graph when ROS is installed.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python3 scripts/test_camera_delivery.py
python3 scripts/test_elevator_camera.py
python3 scripts/test_floor_arrival_probe.py
python3 scripts/test_arm_execution_contract.py
python3 -m unittest discover -s tools/button_arm_test -p 'test_*.py'
python3 -m unittest discover -s tools/floor_reader -p 'test_*.py'
python3 scripts/check_portability.py
if python3 -c 'import rclpy' >/dev/null 2>&1; then
  python3 scripts/test_camera_delivery_ros.py
fi
