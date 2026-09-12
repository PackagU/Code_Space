#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
WITH_RVIZ="${WITH_RVIZ:-0}"
[[ "${WITH_RVIZ}" == "0" || "${WITH_RVIZ}" == "1" ]] || {
  echo "error: WITH_RVIZ must be 0 or 1" >&2
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

# 2026-09-12: 중복 실행 가드. 같은 스택을 두 번 띄우면 노드가 이중으로 떠서
# odom->base_footprint TF 를 두 발행자가 동시에 쏘고 CPU 가 포화된다.
# [측정값] 현장에서 field_base 2벌일 때 load average 6.18 -> 13.14, map->base_footprint 조회 실패.
_existing="$(docker exec "${CONTAINER_NAME}" pgrep -f 'field_mapping_only\.launch\.py' 2>/dev/null | tr '\n' ' ' || true)"
if [[ -n "${_existing// /}" ]]; then
  echo "error: mapping 가 이미 ${CONTAINER_NAME} 에서 실행 중이다 (컨테이너 PID: ${_existing})" >&2
  echo "       먼저 정리하라: docker exec ${CONTAINER_NAME} pkill -INT -f 'field_mapping_only.launch'" >&2
  echo "       정리 확인: docker exec ${CONTAINER_NAME} pgrep -af 'field_mapping_only.launch'" >&2
  exit 1
fi
nodes="$(in_container 'ros2 node list')"
for required in /robot_state_publisher /rplidar /packagu_opencr_bridge /nav_safety_gate; do
  grep -Fxq "${required}" <<<"${nodes}" || {
    echo "error: required base node missing: ${required}" >&2
    exit 1
  }
done
topics="$(in_container 'ros2 topic list')"
for required in /scan /odom /drive/ready /cmd_vel_safe; do
  grep -Fxq "${required}" <<<"${topics}" || {
    echo "error: required base topic missing: ${required}" >&2
    exit 1
  }
done
grep -Fxq /slam_toolbox <<<"${nodes}" && {
  echo "error: slam_toolbox is already running" >&2
  exit 1
}

rviz=false
[[ "${WITH_RVIZ}" == "1" ]] && rviz=true
echo "[field-mapping] starting SLAM only; base stays alive after Ctrl+C"
in_container "ros2 launch slam_pkg field_mapping_only.launch.py rviz:=${rviz}"
