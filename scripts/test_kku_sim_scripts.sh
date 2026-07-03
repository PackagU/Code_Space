#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

require_file() {
  local path="$1"
  [[ -f "${ROOT_DIR}/${path}" ]] || fail "missing ${path}"
}

require_executable() {
  local path="$1"
  [[ -x "${ROOT_DIR}/${path}" ]] || fail "${path} is not executable"
}

require_contains() {
  local path="$1"
  local pattern="$2"
  grep -Fq "${pattern}" "${ROOT_DIR}/${path}" || fail "${path} does not contain: ${pattern}"
}

scripts=(
  scripts/run_kku_sim.sh
  scripts/teleop.sh
  scripts/save_kku_map.sh
)

for script in "${scripts[@]}"; do
  require_file "${script}"
  require_executable "${script}"
  bash -n "${ROOT_DIR}/${script}"
done

require_contains scripts/run_kku_sim.sh "docker compose"
require_contains scripts/run_kku_sim.sh "Cleaning stale simulation processes"
require_contains scripts/run_kku_sim.sh "pkill -x gzserver"
require_contains scripts/run_kku_sim.sh "colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg"
require_contains scripts/run_kku_sim.sh "kku_simulation.launch.py floor:="
require_contains scripts/teleop.sh "ros2 run drive_pkg keyboard_teleop"
require_contains scripts/save_kku_map.sh "map_saver_cli"
require_contains scripts/save_kku_map.sh "/ros2_ws/maps/kku_virtual"
require_contains src/common_pkg/urdf/robot.urdf.xacro '<origin xyz="${wheel_offset_x} ${-wheel_offset_y} ${wheel_z_in_base}" rpy="-1.5708 0 0"/>'

echo "PASS: KKU simulation helper scripts look ready."
