#!/usr/bin/env bash
# One command: reuse an installed image, build, run, record, analyse, clean up.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${PACKAGU_SIM_IMAGE:-ghcr.io/packagu/ros2-humble-slam:humble}"
NAME="${PACKAGU_SIM_CONTAINER:-packagu_real_map_sim}"
DOMAIN="${PACKAGU_SIM_DOMAIN:-215}"
SKIP_PREPARE="${SIM_SKIP_PREPARE:-0}"
MODE="${SIM_MODE:-suite}"
[[ "$MODE" == suite || "$MODE" == manual || "$MODE" == hold ]] || exit 2
[[ "$NAME" =~ ^[A-Za-z0-9_.-]+$ ]] || exit 2
[[ "$DOMAIN" =~ ^[0-9]+$ ]] && (( DOMAIN >= 1 && DOMAIN <= 232 )) || exit 2
[[ "$SKIP_PREPARE" == 0 || "$SKIP_PREPARE" == 1 ]] || exit 2
mkdir -p "$ROOT_DIR/logs/real_map_sim"
[[ "${SIM_GUI:-0}" == 0 || "${SIM_GUI:-0}" == 1 ]] || exit 2
if [[ "${SIM_GUI:-0}" == 1 ]]; then
  bash "$ROOT_DIR/sim/real_maps/show_gui.sh" "$NAME" "$DOMAIN" &
fi
docker image inspect "$IMAGE" --format '{{.Id}}'
docker run --rm --name "$NAME" --network bridge --shm-size 512m \
  -e ROS_DOMAIN_ID="$DOMAIN" -e GAZEBO_MODEL_DATABASE_URI= \
  -e SIM_SKIP_PREPARE="$SKIP_PREPARE" -e SIM_MODE="$MODE" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/sim/real_maps/dds_udp.xml \
  -e ROS_LOG_DIR="/ros2_ws/logs/real_map_sim/ros_$DOMAIN" \
  -e GAZEBO_LOG_PATH=/ros2_ws/logs/real_map_sim/gazebo \
  -v "$ROOT_DIR:/ros2_ws" \
  -v "$ROOT_DIR/src/slam_pkg/maps:/ros2_ws/maps:ro" \
  -v "$ROOT_DIR/logs:/ros2_ws/logs" \
  -w /ros2_ws --entrypoint bash "$IMAGE" -c '
    set -e
    source /opt/ros/humble/setup.bash
    if [[ "$SIM_SKIP_PREPARE" == 0 ]]; then
      colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg > logs/real_map_sim/build.log 2>&1
      python3 sim/real_maps/generate_worlds.py > logs/real_map_sim/geometry.log
    fi
    source install/setup.bash
    if [[ "$SIM_MODE" == hold ]]; then
      exec python3 sim/real_maps/hold_position.py "$@"
    fi
    if [[ "$SIM_MODE" == manual ]]; then
      exec python3 sim/real_maps/run_manual_mission.py "$@"
    fi
    exec python3 sim/real_maps/run_scenarios.py "$@"
  ' bash "$@"
