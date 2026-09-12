#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
BAG_PATH="${1:-}"
PROFILE="${PROFILE:-reslam}"
REPLAY_DOMAIN_ID="${REPLAY_DOMAIN_ID:-227}"
PHYSICAL_DOMAIN_ID="${PHYSICAL_DOMAIN_ID:-0}"
RATE="${RATE:-1.0}"

[[ "${BAG_PATH}" =~ ^/ros2_ws/logs/[A-Za-z0-9_./-]+$ ]] || {
  echo "error: bag path must be an absolute directory below /ros2_ws/logs" >&2
  exit 2
}
[[ "${PROFILE}" == "reslam" || "${PROFILE}" == "inspect" ]] || { echo "error: invalid PROFILE" >&2; exit 2; }
[[ "${REPLAY_DOMAIN_ID}" =~ ^[0-9]+$ && "${PHYSICAL_DOMAIN_ID}" =~ ^[0-9]+$ ]] || {
  echo "error: ROS domain IDs must be integers" >&2
  exit 2
}
(( REPLAY_DOMAIN_ID >= 1 && REPLAY_DOMAIN_ID <= 232 )) || { echo "error: replay domain must be 1..232" >&2; exit 2; }
[[ "${REPLAY_DOMAIN_ID}" != "${PHYSICAL_DOMAIN_ID}" ]] || {
  echo "error: replay domain equals physical domain" >&2
  exit 1
}
[[ "${RATE}" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "error: RATE must be positive" >&2; exit 2; }

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}
docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || {
  echo "error: ${CONTAINER_NAME} is not running" >&2
  exit 1
}

replay_nodes="$(in_container "ROS_DOMAIN_ID='${REPLAY_DOMAIN_ID}' ros2 node list" 2>/dev/null || true)"
if grep -Eiq '/.*(opencr|arm|lift|servo|controller_server|nav_safety_gate)' <<<"${replay_nodes}"; then
  echo "error: hardware/control node detected in replay domain ${REPLAY_DOMAIN_ID}" >&2
  exit 1
fi
topics="$(in_container "python3 scripts/bag_contract.py replay-topics '${BAG_PATH}' --profile '${PROFILE}'")"
grep -Eq '(^| )/(cmd_vel|arm|lift|servo)' <<<"${topics}" && {
  echo "error: control topic entered replay list" >&2
  exit 1
}

echo "[field-replay] isolated ROS_DOMAIN_ID=${REPLAY_DOMAIN_ID}; use_sim_time=true consumers only"
echo "[field-replay] topics=${topics}"
in_container "ROS_DOMAIN_ID='${REPLAY_DOMAIN_ID}' ros2 bag play '${BAG_PATH}' --clock 100 --rate '${RATE}' --topics ${topics}"
