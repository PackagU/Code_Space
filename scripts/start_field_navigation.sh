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

# 2026-09-12: 중복 실행 가드. 같은 스택을 두 번 띄우면 노드가 이중으로 떠서
# odom->base_footprint TF 를 두 발행자가 동시에 쏘고 CPU 가 포화된다.
# [측정값] 현장에서 field_base 2벌일 때 load average 6.18 -> 13.14, map->base_footprint 조회 실패.
_existing="$(docker exec "${CONTAINER_NAME}" pgrep -f 'kku_navigation\.launch\.py' 2>/dev/null | tr '\n' ' ' || true)"
if [[ -n "${_existing// /}" ]]; then
  echo "error: navigation 가 이미 ${CONTAINER_NAME} 에서 실행 중이다 (컨테이너 PID: ${_existing})" >&2
  echo "       먼저 정리하라: docker exec ${CONTAINER_NAME} pkill -INT -f 'kku_navigation.launch'" >&2
  echo "       정리 확인: docker exec ${CONTAINER_NAME} pgrep -af 'kku_navigation.launch'" >&2
  exit 1
fi
if [[ -z "${MAP_YAML}" ]]; then
  MAP_YAML="$(in_container "cat '/ros2_ws/maps/field/${FLOOR_DIR}/latest_map.txt'")"
fi
[[ "${MAP_YAML}" =~ ^/ros2_ws/maps/[A-Za-z0-9_./-]+\.yaml$ ]] || {
  echo "error: map must be an absolute YAML path below /ros2_ws/maps" >&2
  exit 2
}
in_container "python3 -m slam_pkg.map_contract '${MAP_YAML}'"
# 2026-09-15: 지도 버전 fail-closed. 9/15 00:09 F1 복귀가 v2 기대와 달리 v1을 로드했다.
# latest_map.txt·명시 지도·YAML/PGM SHA256이 map_pins.json과 모두 같아야 Nav2를 시작한다.
in_container "python3 scripts/field_map_guard.py check --floor '${FLOOR}' --stage pre-nav --map-yaml '${MAP_YAML}'" || {
  echo "error: ${FLOOR} map is not the pinned version; Nav2 was not started" >&2
  exit 1
}
# 2026-09-15: 증속 후보 등 대체 params는 명시할 때만 쓴다. 기본은 slam_pkg/config/nav2_params.yaml.
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-}"
params_arg=""
if [[ -n "${NAV2_PARAMS_FILE}" ]]; then
  [[ "${NAV2_PARAMS_FILE}" =~ ^/ros2_ws/src/slam_pkg/config/nav2_params[A-Za-z0-9_]*\.yaml$ ]] || {
    echo "error: NAV2_PARAMS_FILE must be /ros2_ws/src/slam_pkg/config/nav2_params*.yaml" >&2
    exit 2
  }
  in_container "test -f '${NAV2_PARAMS_FILE}'" || { echo "error: ${NAV2_PARAMS_FILE} not found" >&2; exit 2; }
  params_arg="params_file:='${NAV2_PARAMS_FILE}'"
fi

nodes="$(in_container 'ros2 node list')"
grep -Fxq /slam_toolbox <<<"${nodes}" && {
  echo "error: stop SLAM before starting AMCL/Nav2" >&2
  exit 1
}
if grep -Eiq '/.*(arm|lift|servo)' <<<"${nodes}"; then
  echo "error: arm/lift/servo node detected; autonomous base profile requires them stopped" >&2
  exit 1
fi
# 2026-09-13: teleop과 웹 UI는 idle일 때도 /cmd_vel에 0을 보낼 수 있다(웹 UI는 자기가 띄운 Nav2만 인식).
# Nav2 smoother 출력과 0이 교대하면 주행이 끊기므로 명령 소유자를 하나로 강제한다.
for conflicting in /packagu_keyboard_teleop /packagu_field_web_ui; do
  if grep -Fxq "${conflicting}" <<<"${nodes}"; then
    echo "error: ${conflicting} also publishes /cmd_vel; stop it before Nav2 (command ownership)" >&2
    exit 1
  fi
done
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
  sample="$(in_container "timeout 8 ros2 topic echo --once /nav_safety/ready std_msgs/msg/Bool" 2>/dev/null || true)"
  if grep -Eq 'data:[[:space:]]*true' <<<"${sample}"; then ready=1; break; fi
done
[[ "${ready}" == "1" ]] || {
  echo "error: /nav_safety/ready did not become true; do not send a goal" >&2
  exit 1
}

echo "[field-nav] map=${MAP_YAML}"
echo "[field-nav] params=${NAV2_PARAMS_FILE:-default slam_pkg/config/nav2_params.yaml}"
echo "[field-nav] starting saved-map AMCL/Nav2 only; no goal is sent; arm/lift excluded"
in_container "ros2 launch slam_pkg kku_navigation.launch.py map:='${MAP_YAML}' use_sim_time:=false rviz:=false ${params_arg}"
