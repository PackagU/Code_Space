#!/usr/bin/env bash
# 무명령 활주(§1.22) 회귀 검사 — 바퀴 마찰 파라미터 변경 시 필수 재실측.
# Gazebo F1 을 띄우고 cmd_vel 없이 DRIFT_WINDOW_S 동안 로봇 참값 이동량을 잰다.
# PASS: 이동 < MAX_DRIFT_M (기본 0.02m/90s — §1.22 수정 실측 2.4mm/90s 의 8배 여유).
# 사용(컨테이너): bash test_workspace/gazebo_world_swap/scripts/check_idle_drift.sh
set -euo pipefail

ROOT="${ROOT:-/ros2_ws}"
DRIFT_WINDOW_S="${DRIFT_WINDOW_S:-90}"
MAX_DRIFT_M="${MAX_DRIFT_M:-0.02}"
OUT="${OUT:-/tmp/idle_drift}"
mkdir -p "$OUT"

log() { printf '[idle-drift] %s\n' "$*"; }

get_xy() {
  # 로봇 참값 (x y) — 실패 시 빈 출력
  timeout 15 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
    "{name: 'elevator_robot_f1'}" 2>/dev/null | tr -d '\n' \
    | grep -oP "position=geometry_msgs\.msg\.Point\(x=\K[-0-9.e+]+, y=[-0-9.e+]+" \
    | head -1 | tr -d ' ' | tr ',' ' ' | sed 's/y=//'
}

cleanup() {
  pkill -f "ros2 launch common_pkg gazebo.launch.py" 2>/dev/null || true
  sleep 1
  pkill -9 -f gzserver 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cd "$ROOT"
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
ros2 daemon stop >/dev/null 2>&1 || true
cleanup

log "launching gazebo F1 (headless)"
ros2 launch common_pkg gazebo.launch.py floor:=F1 spawn_point:=charge_station \
  use_sim_time:=true gui:=false >"$OUT/gazebo.log" 2>&1 &

end=$((SECONDS + 60))
until timeout 10 ros2 service list 2>/dev/null | grep -q "^/gazebo/get_entity_state$"; do
  if (( SECONDS >= end )); then log "FAIL: gazebo services not up in 60s"; exit 1; fi
  sleep 2
done
sleep 8  # 스폰 안착 대기

read -r X0 Y0 <<<"$(get_xy)"
if [[ -z "${X0:-}" || -z "${Y0:-}" ]]; then log "FAIL: initial pose unavailable"; exit 1; fi
log "t0 pose: ($X0, $Y0) — waiting ${DRIFT_WINDOW_S}s with no commands"
sleep "$DRIFT_WINDOW_S"
read -r X1 Y1 <<<"$(get_xy)"
if [[ -z "${X1:-}" || -z "${Y1:-}" ]]; then log "FAIL: final pose unavailable"; exit 1; fi

DRIFT=$(python3 -c "import math;print(f'{math.hypot($X1-($X0), $Y1-($Y0)):.4f}')")
log "t1 pose: ($X1, $Y1) — drift ${DRIFT}m / ${DRIFT_WINDOW_S}s (threshold $MAX_DRIFT_M)"
python3 -c "import sys; sys.exit(0 if $DRIFT < $MAX_DRIFT_M else 1)" \
  && log "PASS idle drift" || { log "FAIL idle drift"; exit 1; }
