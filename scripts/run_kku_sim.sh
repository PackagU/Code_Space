#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="${COMPOSE_FILE:-${ROOT_DIR}/docker/compose/docker-compose.linux.yml}"
CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${1:-F1}"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/run_kku_sim.sh [F1|F2|F3]

Starts the ROS2 Docker container if needed, builds common_pkg/slam_pkg/drive_pkg,
and launches Gazebo + SLAM Toolbox + RViz2 for the selected KKU floor.
USAGE
}

case "${FLOOR}" in
  F1|F2|F3) ;;
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

if [[ ! -f "${COMPOSE_FILE}" ]]; then
  echo "error: compose file not found: ${COMPOSE_FILE}" >&2
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 || true
  xhost +local:root >/dev/null 2>&1 || true
fi

echo "[1/4] Starting Docker container (${CONTAINER_NAME})..."
docker compose -f "${COMPOSE_FILE}" up -d

echo "[2/4] Cleaning stale simulation processes..."
docker exec "${CONTAINER_NAME}" bash -lc '
  pkill -f "ros2 launch slam_pkg kku_simulation[.]launch[.]py" || true
  pkill -x gzclient || true
  pkill -x gzserver || true
  pkill -f "/opt/ros/humble/lib/rviz2/rviz[2]" || true
  pkill -f "/opt/ros/humble/lib/gazebo_ros/spawn_entity[.]py" || true
  pkill -f "async_slam_toolbox_nod[e]" || true
  pkill -f "robot_state_publishe[r]" || true
  sleep 1
' || true

echo "[3/4] Building ROS2 packages..."
docker exec -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg'

echo "[4/4] Launching KKU simulation on ${FLOOR}..."
echo "Tip: open another terminal and run ./scripts/teleop.sh"
docker exec -it -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_pkg kku_simulation.launch.py floor:=${FLOOR}"
