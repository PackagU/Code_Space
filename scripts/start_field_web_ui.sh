#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
PORT="${PORT:-8080}"

[[ "${PORT}" =~ ^[0-9]+$ ]] && (( PORT >= 1024 && PORT <= 65535 )) || {
  echo "error: PORT must be 1024..65535" >&2
  exit 2
}
docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || {
  echo "error: ${CONTAINER_NAME} is not running" >&2
  exit 1
}
if docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 node list' | grep -Fxq /packagu_field_web_ui; then
  echo "error: /packagu_field_web_ui is already running" >&2
  exit 1
fi

echo "[field-ui] laptop tunnel: ssh -N -L ${PORT}:127.0.0.1:${PORT} hsm@192.168.0.7"
echo "[field-ui] browser: http://127.0.0.1:${PORT}"
echo "[field-ui] Ctrl+C stops the UI and publishes stop; arm/lift nodes are not started"
docker exec -it -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run slam_pkg field_web_ui --ros-args -p bind_host:=127.0.0.1 -p port:='${PORT}'"
