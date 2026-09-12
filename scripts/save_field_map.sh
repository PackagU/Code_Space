#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${1:-F1}"
MAP_NAME="${2:-}"
case "${FLOOR}" in
  F1|F2|F3) FLOOR_DIR="${FLOOR,,}" ;;
  *) echo "error: floor must be F1, F2, or F3" >&2; exit 2 ;;
esac
if [[ -z "${MAP_NAME}" ]]; then
  MAP_NAME="kku_${FLOOR_DIR}_$(date '+%Y%m%d_%H%M%S')"
fi
[[ "${MAP_NAME}" =~ ^[A-Za-z0-9_-]+$ ]] || {
  echo "error: map name may contain only letters, digits, underscore, and dash" >&2
  exit 2
}

OUTPUT_DIR="/ros2_ws/maps/field/${FLOOR_DIR}"
OUTPUT_PATH="${OUTPUT_DIR}/${MAP_NAME}"

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}

docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || {
  echo "error: ${CONTAINER_NAME} is not running" >&2
  exit 1
}
nodes="$(in_container 'ros2 node list')"
grep -Fxq /slam_toolbox <<<"${nodes}" || {
  echo "error: /slam_toolbox is not running" >&2
  exit 1
}
if in_container "test -e '${OUTPUT_PATH}.yaml' -o -e '${OUTPUT_PATH}.pgm' -o -e '${OUTPUT_PATH}.posegraph'"; then
  echo "error: refusing to overwrite ${OUTPUT_PATH}.*" >&2
  exit 1
fi

in_container "mkdir -p '${OUTPUT_DIR}' && ros2 run nav2_map_server map_saver_cli -f '${OUTPUT_PATH}'"
serialize_output="$(in_container "ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '${OUTPUT_PATH}'}\"")"
printf '%s\n' "${serialize_output}"
grep -Eq 'result[=:][[:space:]]*0|result=0' <<<"${serialize_output}" || {
  echo "error: posegraph serialization did not report result 0" >&2
  exit 1
}
in_container "python3 -m slam_pkg.map_contract '${OUTPUT_PATH}.yaml'"
in_container "cd '${OUTPUT_DIR}' && sha256sum '${MAP_NAME}.yaml' '${MAP_NAME}.pgm' '${MAP_NAME}.posegraph' '${MAP_NAME}.data' > '${MAP_NAME}.sha256' && printf '%s\n' '${OUTPUT_PATH}.yaml' > latest_map.txt"

echo "MAP_YAML=${OUTPUT_PATH}.yaml"
echo "[field-map] saved occupancy map, posegraph, and SHA256 manifest"
