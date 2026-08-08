#!/usr/bin/env bash
# 현장 실측 원커맨드 (스펙 §5.4, AGENTS E2E 정책).
# 흐름: 컨테이너 확인 -> 실기 매핑 launch -> 토픽 대기 -> rosbag 기록(전경)
#       -> Ctrl+C -> 맵+posegraph 저장 -> 산출물 검증 -> cleanup
# 사용:
#   FLOOR=F1 ./scripts/run_field_mapping.sh
#   WITH_RVIZ=1  : (데스크톱 검증용) RViz 켜기 — Jetson에서는 0 유지
#   BAG=0        : rosbag 생략 (권장 안 함 — bag은 재SLAM 보험)
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${FLOOR:-F1}"
WITH_RVIZ="${WITH_RVIZ:-0}"
BAG="${BAG:-1}"
TS="$(date +%Y-%m-%d_%H-%M-%S)"
BAG_DIR="/ros2_ws/logs/field_${TS}"
RVIZ_ARG="false"
[[ "${WITH_RVIZ}" == "1" ]] && RVIZ_ARG="true"

case "${FLOOR}" in F1|F2|F3) ;; *) echo "error: FLOOR must be F1|F2|F3" >&2; exit 2 ;; esac

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "error: ${CONTAINER_NAME} is not running (docker compose -f docker/compose/docker-compose.jetson.yml up -d)" >&2
  exit 1
fi

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}

cleanup() {
  echo "[field] cleanup: stopping launch/bag processes in container"
  docker exec -i "${CONTAINER_NAME}" bash -lc \
    "pkill -INT -f 'ros2 bag record' 2>/dev/null; pkill -INT -f slam_toolbox 2>/dev/null; pkill -INT -f rplidar 2>/dev/null; pkill -INT -f opencr_bridge 2>/dev/null; pkill -INT -f robot_state_publisher 2>/dev/null" || true
}
trap cleanup EXIT

echo "[field] launching mapping stack (floor=${FLOOR}, rviz=${RVIZ_ARG})"
docker exec -d -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=false enable_drive:=true rviz:=${RVIZ_ARG} serial_port:=/dev/rplidar"

echo "[field] waiting for /scan and /odom (max 60s)"
READY=0
for _ in $(seq 1 30); do
  if in_container "ros2 topic list" | grep -q "^/scan$" \
     && in_container "ros2 topic list" | grep -q "^/odom$"; then
    READY=1; break
  fi
  sleep 2
done
if [[ "${READY}" != "1" ]]; then
  echo "error: /scan or /odom not available — 철수 기준 확인 (스펙 §6.4)" >&2
  exit 1
fi

# Ctrl+C는 전경 자식(bag record)만 멈추고 스크립트는 저장 단계로 진행해야 한다.
# trap 없이는 bash가 SIGINT로 함께 종료되어 맵 저장을 건너뛴다 (bash WCE 규칙).
trap ':' INT
if [[ "${BAG}" == "1" ]]; then
  echo "[field] recording rosbag to ${BAG_DIR} — 맵핑 종료 시 Ctrl+C"
  in_container "mkdir -p '${BAG_DIR}' && ros2 bag record -o '${BAG_DIR}/bag' /scan /scan_raw /odom /imu /cmd_vel /tf /tf_static" || true
else
  echo "[field] BAG=0 — 기록 없이 대기. 맵핑 종료 시 Ctrl+C"
  sleep infinity || true
fi
trap - INT

echo "[field] saving real map for ${FLOOR}"
"$(dirname "$0")/save_kku_map.sh" "${FLOOR}" --real

FLOOR_DIR="$(tr '[:upper:]' '[:lower:]' <<<"${FLOOR}")"
MAP_BASE="/ros2_ws/maps/kku_real/${FLOOR_DIR}/kku_${FLOOR_DIR}_real"
echo "[field] verifying artifacts"
in_container "test -f '${MAP_BASE}.yaml' && test -f '${MAP_BASE}.pgm' && test -f '${MAP_BASE}.posegraph'"
echo "[field] OK: ${MAP_BASE}.{yaml,pgm,posegraph}"
[[ "${BAG}" == "1" ]] && echo "[field] bag: ${BAG_DIR} (로컬 전용 — 외부 업로드 금지)"
