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
nodes="$(docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 node list' 2>/dev/null || true)"
if grep -Fxq /packagu_field_web_ui <<<"${nodes}"; then
  echo "error: /packagu_field_web_ui is already running" >&2
  exit 1
fi
# 2026-09-13: UI는 자기가 띄운 Nav2가 아니면 idle로 보고 /cmd_vel에 0을 20 Hz로 보낸다.
# 터미널 Nav2나 teleop과 섞이면 명령이 0과 교대하므로 먼저 거부한다.
for conflicting in /controller_server /packagu_keyboard_teleop; do
  if grep -Fxq "${conflicting}" <<<"${nodes}"; then
    echo "error: ${conflicting} already owns /cmd_vel; stop it before the web UI (command ownership)" >&2
    exit 1
  fi
done

echo "[field-ui] laptop tunnel: ssh -N -L ${PORT}:127.0.0.1:${PORT} hsm@192.168.0.7"
echo "[field-ui] browser: http://127.0.0.1:${PORT}"
echo "[field-ui] Ctrl+C stops the UI and publishes stop; arm/lift nodes are not started"
docker exec -it -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run slam_pkg field_web_ui --ros-args -p bind_host:=127.0.0.1 -p port:='${PORT}'"
