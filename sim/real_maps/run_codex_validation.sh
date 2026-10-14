#!/usr/bin/env bash
# One command: build -> generate -> offline checks -> autonomous round trip -> logs -> cleanup.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"
export PACKAGU_SIM_CONTAINER="${PACKAGU_SIM_CONTAINER:-packagu_sim_codex_20261013}"
SC="$ROOT_DIR/sim/real_maps/sim_container.sh"
mkdir -p logs/real_map_sim
RUN_NAME="codex_flat_wrist_$(date +%Y%m%d_%H%M%S)"
trap 'bash "$SC" down' EXIT
bash "$SC" up --gui  # X11 rendering is required for the existing wrist camera.
if [[ ! -f install/setup.bash || "${SIM_REBUILD:-0}" == 1 ]]; then
  bash "$SC" exec bash -c 'colcon build --symlink-install --base-paths src test_workspace/elevator_auto_map_switch/src --packages-select common_pkg slam_pkg drive_pkg auto_floor_orchestrator_pkg' \
    > "logs/real_map_sim/${RUN_NAME}_build.log" 2>&1
fi
bash "$SC" exec python3 sim/real_maps/generate_worlds.py > "logs/real_map_sim/${RUN_NAME}_geometry.json"
bash "$SC" exec bash sim/real_maps/run_sim_tests.sh
python3 scripts/check_portability.py
GUI_ARGS=()
[[ "${SIM_GUI:-1}" == 1 ]] && GUI_ARGS+=(--gui)
python3 -u sim/real_maps/run_e2e.py --name "$RUN_NAME" --cabin-mode direct --arm-sim --load pallet \
  --parcels "${SIM_PARCELS:-2}" --speed "${SIM_SPEED:-max}" "${GUI_ARGS[@]}" "$@" \
  | tee "logs/real_map_sim/${RUN_NAME}_console.log"
