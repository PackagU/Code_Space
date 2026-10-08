#!/usr/bin/env bash
# One command: isolated container -> build/generate if needed -> Gazebo real-map world + robot +
# OpenCR shim + nav_safety_gate + Nav2 (ordered STARTUP) -> readiness check. Simulation only.
#
#   bash sim/real_maps/start_sim.sh --floor F1 --params P0 --spawn f1_initial_test [--gui]
#   bash sim/real_maps/start_sim.sh --world building --floor F1 --spawn f1_initial_test   # two floors + elevator
#   bash sim/real_maps/start_sim.sh --stop
# Options: --name NAME --initial WAYPOINT --lidar-noise on|off --odom encoder|world --no-record --arm-sim
# Env: SIM_REBUILD=1 (force colcon build), SIM_REGENERATE=1 (force world generation)
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SC="$ROOT_DIR/sim/real_maps/sim_container.sh"
FLOOR=F1 PARAMS=P0 SPAWN=f1_initial_test WORLD="" NAME="" GUI=0 STOP=0
EXTRA=()
while (($#)); do
  case "$1" in
    --floor) FLOOR="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --spawn) SPAWN="$2"; shift 2 ;;
    --world) WORLD="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --initial|--lidar-noise|--odom) EXTRA+=("$1" "$2"); shift 2 ;;
    --no-record|--arm-sim) EXTRA+=("$1"); shift ;;
    --gui) GUI=1; shift ;;
    --stop) STOP=1; shift ;;
    -h|--help) sed -n 2,10p "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done
if ((STOP)); then
  CNAME="$(bash "$SC" name)"
  if docker exec "$CNAME" pgrep -f sim_stack.py >/dev/null 2>&1; then
    docker exec "$CNAME" pkill -TERM -f sim_stack.py || true
    for _ in $(seq 1 60); do docker exec "$CNAME" pgrep -f sim_stack.py >/dev/null 2>&1 || break; sleep 1; done
  fi
  bash "$SC" down
  exit 0
fi
[[ "$FLOOR" =~ ^F[12]$ ]] || { echo "--floor must be F1 or F2" >&2; exit 2; }
[[ "$PARAMS" =~ ^(P0|P1|v011|v012)$ ]] || { echo "--params must be P0|P1|v011|v012" >&2; exit 2; }
WORLD="${WORLD:-${FLOOR,,}}"
NAME="${NAME:-sim_${WORLD}_${PARAMS}_$(date +%Y%m%d_%H%M%S)}"
[[ "$NAME" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "invalid --name" >&2; exit 2; }
RUN_DIR="$ROOT_DIR/logs/real_map_sim/$NAME"
[[ ! -e "$RUN_DIR/bag" ]] || { echo "run $NAME already has a bag; choose a new --name" >&2; exit 2; }

if ((GUI)); then bash "$SC" up --gui; else bash "$SC" up; fi
CNAME="$(bash "$SC" name)"
if docker exec "$CNAME" pgrep -x gzserver >/dev/null 2>&1; then
  echo "a simulation is already running in $CNAME; stop it first: bash sim/real_maps/start_sim.sh --stop" >&2
  exit 1
fi
mkdir -p "$RUN_DIR"
if [[ "${SIM_REBUILD:-0}" == 1 || ! -d "$ROOT_DIR/install/slam_pkg/share/slam_pkg" \
      || ! -d "$ROOT_DIR/install/auto_floor_orchestrator_pkg" ]]; then
  echo "[start_sim] colcon build (log: logs/real_map_sim/build.log)"
  bash "$SC" exec bash -c 'colcon build --symlink-install --base-paths src test_workspace/elevator_auto_map_switch/src \
    --packages-select common_pkg slam_pkg drive_pkg auto_floor_orchestrator_pkg > logs/real_map_sim/build.log 2>&1'
fi
GEN="$ROOT_DIR/sim/real_maps/generated/geometry_validation.json"
if [[ "${SIM_REGENERATE:-0}" == 1 || ! -f "$GEN" || "$ROOT_DIR/sim/real_maps/world_params.yaml" -nt "$GEN" \
      || "$ROOT_DIR/sim/real_maps/generate_worlds.py" -nt "$GEN" ]]; then
  echo "[start_sim] generating worlds (log: logs/real_map_sim/geometry.log)"
  bash "$SC" exec python3 sim/real_maps/generate_worlds.py > "$ROOT_DIR/logs/real_map_sim/geometry.log"
fi
docker exec -d -w /ros2_ws "$CNAME" bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && \
  exec python3 -u sim/real_maps/sim_stack.py --name '$NAME' --world '$WORLD' --floor '$FLOOR' --params '$PARAMS' \
  --spawn '$SPAWN' ${EXTRA[*]:-} > logs/real_map_sim/$NAME/stack.log 2>&1"
echo "[start_sim] starting $NAME (world=$WORLD floor=$FLOOR params=$PARAMS spawn=$SPAWN) ..."
for _ in $(seq 1 600); do
  if [[ -f "$RUN_DIR/ready.json" ]]; then
    python3 - "$RUN_DIR/ready.json" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
print(f"READY  rtf={r['rtf']:.2f}  scan={r['scan_rate_hz_sim']:.2f} Hz/{r.get('scan_points')} pts  "
      f"amcl_err={r.get('amcl_pos_err_m', float('nan')):.3f} m  drive_ready={r['drive_ready']}")
print('lifecycle:', ' '.join(f"{k}={v}" for k, v in r['lifecycle'].items()))
PY
    if ((GUI)); then
      docker exec -d -w /ros2_ws "$CNAME" bash -c "source /opt/ros/humble/setup.bash && \
        GAZEBO_MODEL_DATABASE_URI= gzclient > logs/real_map_sim/$NAME/gzclient.log 2>&1"
      docker exec -d -w /ros2_ws "$CNAME" bash -c "source /opt/ros/humble/setup.bash && \
        rviz2 -d sim/real_maps/nav2_view.rviz --ros-args -p use_sim_time:=true > logs/real_map_sim/$NAME/rviz.log 2>&1"
    fi
    echo "goal:  python3 sim/real_maps/sim_goal.py --name $NAME --to f1_locker"
    echo "stop:  bash sim/real_maps/start_sim.sh --stop"
    exit 0
  fi
  if [[ -f "$RUN_DIR/failed.json" ]] || ! docker exec "$CNAME" pgrep -f sim_stack.py >/dev/null 2>&1; then
    echo "[start_sim] FAILED: $(cat "$RUN_DIR/failed.json" 2>/dev/null || echo 'stack exited')" >&2
    tail -20 "$RUN_DIR/stack.log" >&2 || true
    exit 1
  fi
  sleep 1
done
echo "[start_sim] readiness timeout (600 s); see $RUN_DIR/stack.log" >&2
exit 1
