#!/usr/bin/env bash
# Official A1/A2M8-style standalone driver probe. No drive/arm nodes are started.
set -eo pipefail
source /opt/ros/humble/setup.bash
timeout 15 ros2 run rplidar_ros rplidar_node --ros-args \
  -r __node:=rplidar_probe \
  -p serial_port:=/dev/rplidar \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p inverted:=false \
  -p angle_compensate:=true
