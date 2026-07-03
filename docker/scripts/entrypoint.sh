#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[entrypoint] ROS2 Humble ready. Domain ID: ${ROS_DOMAIN_ID}"
echo "[entrypoint] DISPLAY: ${DISPLAY:-unset}"

exec "$@"
