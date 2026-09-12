#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${FLOOR:-F1}"
MAP_YAML="${1:-}"
case "${FLOOR}" in
  F1|F2|F3) FLOOR_DIR="${FLOOR,,}" ;;
  *) echo "error: FLOOR must be F1, F2, or F3" >&2; exit 2 ;;
esac

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}

docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || {
  echo "error: ${CONTAINER_NAME} is not running" >&2
  exit 1
}
if [[ -z "${MAP_YAML}" ]]; then
  MAP_YAML="$(in_container "cat '/ros2_ws/maps/field/${FLOOR_DIR}/latest_map.txt'")"
fi
[[ "${MAP_YAML}" =~ ^/ros2_ws/maps/[A-Za-z0-9_./-]+\.yaml$ ]] || {
  echo "error: map must be an absolute YAML path below /ros2_ws/maps" >&2
  exit 2
}
in_container "python3 -m slam_pkg.map_contract '${MAP_YAML}'"

nodes="$(in_container 'ros2 node list')"
grep -Fxq /slam_toolbox <<<"${nodes}" && {
  echo "error: stop SLAM before starting AMCL/Nav2" >&2
  exit 1
}
if grep -Eiq '/.*(arm|lift|servo)' <<<"${nodes}"; then
  echo "error: arm/lift/servo node detected; autonomous base profile requires them stopped" >&2
  exit 1
fi
for required in /robot_state_publisher /rplidar /packagu_opencr_bridge /nav_safety_gate; do
  grep -Fxq "${required}" <<<"${nodes}" || {
    echo "error: required base node missing: ${required}" >&2
    exit 1
  }
done
topics="$(in_container 'ros2 topic list')"
for required in /scan /odom /drive/ready /nav_safety/ready; do
  grep -Fxq "${required}" <<<"${topics}" || {
    echo "error: required base topic missing: ${required}" >&2
    exit 1
  }
done

ready=0
for _ in $(seq 1 10); do
  sample="$(in_container "timeout 2 ros2 topic echo --once /nav_safety/ready std_msgs/msg/Bool" 2>/dev/null || true)"
  if grep -Eq 'data:[[:space:]]*true' <<<"${sample}"; then ready=1; break; fi
done
[[ "${ready}" == "1" ]] || {
  echo "error: /nav_safety/ready did not become true; do not send a goal" >&2
  exit 1
}

echo "[field-nav] map=${MAP_YAML}"
echo "[field-nav] starting saved-map AMCL/Nav2 only; no goal is sent; arm/lift excluded"
in_container "ros2 launch slam_pkg kku_navigation.launch.py map:='${MAP_YAML}' use_sim_time:=false rviz:=false"
