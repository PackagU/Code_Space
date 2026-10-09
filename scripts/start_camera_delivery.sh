#!/usr/bin/env bash
# Run inside the existing ROS container after the original field base/Nav2 are ready.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"
CONFIG_DIR="${ROOT_DIR}/logs/camera_delivery/config"
PREPARE_ONLY=0
if [[ "${1:-}" == "--prepare-only" ]]; then PREPARE_ONLY=1; shift; fi
python3 scripts/prepare_camera_delivery.py --output "${CONFIG_DIR}" "$@"
if [[ "${PREPARE_ONLY}" == 1 ]]; then exit 0; fi
set +u
source /opt/ros/humble/setup.bash
[[ -f install/setup.bash ]] && source install/setup.bash
set -u
nodes="$(ros2 node list)"
if grep -Eq '^/(arm_sequence|packagu_arm_sequence|floor_orchestrator_node|packagu_camera_web|delivery_mission_node)$' <<<"${nodes}"; then
  echo '기존 팔·카메라 미션 노드를 먼저 종료하세요 (포트 소유자 중복 방지).' >&2; exit 1
fi
for required in /nav_safety_gate /bt_navigator /map_server; do
  grep -Fxq "${required}" <<<"${nodes}" || { echo "기존 현장 주행 노드가 필요합니다: ${required}" >&2; exit 1; }
done
readarray -t CONFIG_VALUES < <(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); [print(d[k]) for k in ("start_floor","target_map","pins","registry")]' "${CONFIG_DIR}/launch.json")
for floor in "${CONFIG_VALUES[0]}" "${CONFIG_VALUES[1]}"; do
  python3 scripts/field_map_guard.py check --floor "${floor}" --stage pre-nav --pins "${CONFIG_VALUES[2]}" --registry "${CONFIG_VALUES[3]}"
done
DISPLAY_FLOOR="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["observed_target_floor"])' "${CONFIG_DIR}/launch.json")"
colcon build --symlink-install --base-paths src test_workspace/elevator_mission/src test_workspace/elevator_auto_map_switch/src \
  --packages-select robot_arm_pkg elevator_mission_pkg auto_floor_orchestrator_pkg --parallel-workers 1
set +u
source install/setup.bash
set -u
exec ros2 launch elevator_mission_pkg elevator_camera_mission.launch.py \
  project_root:="${ROOT_DIR}" serial_port:="${ARM_SERVO_PORT:-/dev/arm_servo}" \
  camera_source:="${CAMERA_SOURCE:-/dev/video0}" mission_workspace:="${ROOT_DIR}/test_workspace/elevator_mission" \
  mission_id:=field_camera_delivery points_yaml:="${CONFIG_DIR}/points.yaml" \
  missions_yaml:="${CONFIG_DIR}/missions.yaml" floor_maps_yaml:="${CONFIG_DIR}/maps.yaml" \
  observed_target_floor:="${DISPLAY_FLOOR}" field_map_guard:=true \
  field_pins:="${CONFIG_VALUES[2]}" field_registry:="${CONFIG_VALUES[3]}"
