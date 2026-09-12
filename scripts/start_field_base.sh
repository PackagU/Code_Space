#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
ENABLE_DRIVE="${ENABLE_DRIVE:-0}"
LIDAR_PORT="${LIDAR_PORT:-/dev/rplidar}"
OPENCR_PORT="${OPENCR_PORT:-/dev/opencr}"
LASER_X="${LASER_X:--0.1015}"
LASER_Y="${LASER_Y:-0.0}"
LASER_Z="${LASER_Z:-0.750}"
LASER_ROLL="${LASER_ROLL:-0.0}"
LASER_PITCH="${LASER_PITCH:-0.0}"
LASER_YAW="${LASER_YAW:-0.0}"

[[ "${ENABLE_DRIVE}" == "0" || "${ENABLE_DRIVE}" == "1" ]] || {
  echo "error: ENABLE_DRIVE must be 0 or 1" >&2
  exit 2
}
for value in "${LASER_X}" "${LASER_Y}" "${LASER_Z}" "${LASER_ROLL}" "${LASER_PITCH}" "${LASER_YAW}"; do
  [[ "${value}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || {
    echo "error: LiDAR pose values must be decimal numbers" >&2
    exit 2
  }
done
docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || {
  echo "error: ${CONTAINER_NAME} is not running" >&2
  exit 1
}

# 2026-09-12: 중복 실행 가드. 같은 스택을 두 번 띄우면 노드가 이중으로 떠서
# odom->base_footprint TF 를 두 발행자가 동시에 쏘고 CPU 가 포화된다.
# [측정값] 현장에서 field_base 2벌일 때 load average 6.18 -> 13.14, map->base_footprint 조회 실패.
_existing="$(docker exec "${CONTAINER_NAME}" pgrep -f 'field_base\.launch\.py' 2>/dev/null | tr '\n' ' ' || true)"
if [[ -n "${_existing// /}" ]]; then
  echo "error: field_base 가 이미 ${CONTAINER_NAME} 에서 실행 중이다 (컨테이너 PID: ${_existing})" >&2
  echo "       먼저 정리하라: docker exec ${CONTAINER_NAME} pkill -INT -f 'field_base.launch'" >&2
  echo "       정리 확인: docker exec ${CONTAINER_NAME} pgrep -af 'field_base.launch'" >&2
  exit 1
fi

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}

in_container "test -c '${LIDAR_PORT}' && test \"\$(stat -c '%t:%T' '${LIDAR_PORT}')\" != '1:3'" || {
  echo "error: ${LIDAR_PORT} is not a real character device in the container" >&2
  exit 1
}
if [[ "${ENABLE_DRIVE}" == "1" ]]; then
  in_container "test -c '${OPENCR_PORT}' && test \"\$(stat -c '%t:%T' '${OPENCR_PORT}')\" != '1:3'" || {
    echo "error: ${OPENCR_PORT} is not a real character device in the container" >&2
    exit 1
  }
fi

echo "[field-base] LiDAR=${LIDAR_PORT}; drive=${ENABLE_DRIVE}; arm/lift excluded"
echo "[field-base] laser xyz=${LASER_X},${LASER_Y},${LASER_Z} rpy=${LASER_ROLL},${LASER_PITCH},${LASER_YAW}"
enable_drive_arg=false
[[ "${ENABLE_DRIVE}" == "1" ]] && enable_drive_arg=true
in_container "ros2 launch slam_pkg field_base.launch.py use_sim_time:=false enable_lidar:=true enable_drive:=${enable_drive_arg} lidar_port:='${LIDAR_PORT}' opencr_port:='${OPENCR_PORT}' laser_x:='${LASER_X}' laser_y:='${LASER_Y}' laser_z:='${LASER_Z}' laser_roll:='${LASER_ROLL}' laser_pitch:='${LASER_PITCH}' laser_yaw:='${LASER_YAW}'"
