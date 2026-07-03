#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "error: ${CONTAINER_NAME} is not running. Start it with ./scripts/run_kku_sim.sh F1 first." >&2
  exit 1
fi

docker exec -it -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run drive_pkg keyboard_teleop'
