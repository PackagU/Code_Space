#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${1:-F1}"
MODE="${2:-}"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/save_kku_map.sh [F1|F2|F3]          # 시뮬 가상맵 저장 (기존 동작)
  ./scripts/save_kku_map.sh [F1|F2|F3] --real   # 실측 맵 + posegraph 저장

Saves to:
  virtual: /ros2_ws/maps/kku_virtual/f1/kku_f1        (.yaml/.pgm)
  real:    /ros2_ws/maps/kku_real/f1/kku_f1_real      (.yaml/.pgm/.posegraph/.data)
USAGE
}

case "${FLOOR}" in
  F1) FLOOR_DIR="f1" ;;
  F2) FLOOR_DIR="f2" ;;
  F3) FLOOR_DIR="f3" ;;
  -h|--help) usage; exit 0 ;;
  *) echo "error: floor must be one of F1, F2, F3; got '${FLOOR}'" >&2; usage >&2; exit 2 ;;
esac

REAL=0
if [[ "${MODE}" == "--real" ]]; then
  REAL=1
elif [[ -n "${MODE}" ]]; then
  echo "error: unknown option '${MODE}' (expected --real)" >&2; usage >&2; exit 2
fi

if [[ ${REAL} -eq 1 ]]; then
  OUTPUT_DIR="/ros2_ws/maps/kku_real/${FLOOR_DIR}"
  MAP_NAME="kku_${FLOOR_DIR}_real"
else
  OUTPUT_DIR="/ros2_ws/maps/kku_virtual/${FLOOR_DIR}"
  MAP_NAME="kku_${FLOOR_DIR}"
fi
OUTPUT_PATH="${OUTPUT_DIR}/${MAP_NAME}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "error: ${CONTAINER_NAME} is not running. Start it with ./scripts/run_kku_sim.sh ${FLOOR} first." >&2
  exit 1
fi

docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && mkdir -p '${OUTPUT_DIR}' && ros2 run nav2_map_server map_saver_cli -f '${OUTPUT_PATH}'"

if [[ ${REAL} -eq 1 ]]; then
  # posegraph 직렬화 — 오프라인 재개/재맵핑용 (스펙 §5.4)
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '${OUTPUT_PATH}'}\""
  echo "Saved real map: ${OUTPUT_PATH}.yaml/.pgm + ${OUTPUT_PATH}.posegraph/.data"
else
  echo "Saved map to ${OUTPUT_PATH}.yaml and ${OUTPUT_PATH}.pgm"
fi
