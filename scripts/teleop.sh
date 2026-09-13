#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "error: ${CONTAINER_NAME} is not running. Start it with ./scripts/run_kku_sim.sh F1 first." >&2
  exit 1
fi

# 2026-09-13: /cmd_vel 소유자는 하나다. Nav2, 웹 UI(idle 0 발행), 다른 teleop이 있으면 명령이 섞인다.
nodes="$(docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 node list' 2>/dev/null || true)"
for conflicting in /controller_server /packagu_field_web_ui /packagu_keyboard_teleop; do
  if grep -Fxq "${conflicting}" <<<"${nodes}"; then
    echo "error: ${conflicting} already owns /cmd_vel; stop it before teleop (command ownership)" >&2
    exit 1
  fi
done

docker exec -it -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run drive_pkg keyboard_teleop'
