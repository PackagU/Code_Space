#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${1:-F1}"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/save_kku_map.sh [F1|F2|F3]

Saves the current SLAM map to:
  /ros2_ws/maps/kku_virtual/f1/kku_f1
  /ros2_ws/maps/kku_virtual/f2/kku_f2
  /ros2_ws/maps/kku_virtual/f3/kku_f3
USAGE
}

case "${FLOOR}" in
  F1) FLOOR_DIR="f1"; MAP_NAME="kku_f1" ;;
  F2) FLOOR_DIR="f2"; MAP_NAME="kku_f2" ;;
  F3) FLOOR_DIR="f3"; MAP_NAME="kku_f3" ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    echo "error: floor must be one of F1, F2, F3; got '${FLOOR}'" >&2
    usage >&2
    exit 2
    ;;
esac

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "error: ${CONTAINER_NAME} is not running. Start it with ./scripts/run_kku_sim.sh ${FLOOR} first." >&2
  exit 1
fi

OUTPUT_DIR="/ros2_ws/maps/kku_virtual/${FLOOR_DIR}"
OUTPUT_PATH="${OUTPUT_DIR}/${MAP_NAME}"

docker exec -it -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && mkdir -p '${OUTPUT_DIR}' && ros2 run nav2_map_server map_saver_cli -f '${OUTPUT_PATH}'"

echo "Saved map to ${OUTPUT_PATH}.yaml and ${OUTPUT_PATH}.pgm"
