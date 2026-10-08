#!/usr/bin/env bash
# 9/15 S1~S5/manual/hold runners (WORLD-odom reproduction), now only inside the isolated container.
# 2026-10-08: the 9/15 default-bridge network (domain 215) is replaced by sim_container.sh
# (--internal network, ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID 77); it refuses to start otherwise.
# New pre-tests and E2E use run_pretest.py / run_e2e.py (encoder odometry).
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SC="$ROOT_DIR/sim/real_maps/sim_container.sh"
SKIP_PREPARE="${SIM_SKIP_PREPARE:-0}"
MODE="${SIM_MODE:-suite}"
[[ "$MODE" == suite || "$MODE" == manual || "$MODE" == hold ]] || exit 2
[[ "$SKIP_PREPARE" == 0 || "$SKIP_PREPARE" == 1 ]] || exit 2
mkdir -p "$ROOT_DIR/logs/real_map_sim"
bash "$SC" up
if [[ "$SKIP_PREPARE" == 0 ]]; then
  bash "$SC" exec bash -c 'colcon build --symlink-install --base-paths src test_workspace/elevator_auto_map_switch/src \
    --packages-select common_pkg slam_pkg drive_pkg auto_floor_orchestrator_pkg > logs/real_map_sim/build.log 2>&1'
  bash "$SC" exec python3 sim/real_maps/generate_worlds.py > "$ROOT_DIR/logs/real_map_sim/geometry.log"
fi
case "$MODE" in
  hold) exec bash "$SC" exec python3 -u sim/real_maps/hold_position.py "$@" ;;
  manual) exec bash "$SC" exec python3 -u sim/real_maps/run_manual_mission.py "$@" ;;
  *) exec bash "$SC" exec python3 -u sim/real_maps/run_scenarios.py "$@" ;;
esac
