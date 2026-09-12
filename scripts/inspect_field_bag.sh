#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
BAG_PATH="${1:-}"
REQUIRE_IMU="${REQUIRE_IMU:-0}"
[[ "${BAG_PATH}" =~ ^/ros2_ws/logs/[A-Za-z0-9_./-]+$ ]] || {
  echo "error: bag path must be an absolute directory below /ros2_ws/logs" >&2
  exit 2
}
[[ "${REQUIRE_IMU}" == "0" || "${REQUIRE_IMU}" == "1" ]] || {
  echo "error: REQUIRE_IMU must be 0 or 1" >&2
  exit 2
}

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}
docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || {
  echo "error: ${CONTAINER_NAME} is not running" >&2
  exit 1
}
required_args="--require /scan --require /odom --require /tf --require /tf_static"
[[ "${REQUIRE_IMU}" == "1" ]] && required_args+=" --require /imu"
in_container "python3 scripts/bag_contract.py inspect '${BAG_PATH}' ${required_args}"
in_container "ros2 bag info '${BAG_PATH}'"
in_container "cd '${BAG_PATH}' && sha256sum -c SHA256SUMS"
