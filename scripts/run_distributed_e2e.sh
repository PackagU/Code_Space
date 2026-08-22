#!/usr/bin/env bash
# 분산 E2E 원커맨드 (데스크톱에서 실행): 데스크톱 Gazebo 호스트 + Jetson 실전 스택(smoke) 를
# 순서대로 기동·확인·수집한다. docs/deployment/01_portability_policy.md §4.5 의 수동 순서를 자동화:
#   [0] Jetson 잔존 노드 정리  [1] 데스크톱 Gazebo 리셋+기동 → "Successfully spawned" 확인(사람 눈 의존 제거)
#   [2] Jetson smoke(GAZEBO_REMOTE=1, nohup) 기동  [3] 종료까지 폴링  [4] 로그·아티팩트 수집  [5] 정리
#
# 사용: bash scripts/run_distributed_e2e.sh            # 왕복+보행자+정적장애물+리프트+팔+프로파일
#   환경변수: JETSON_SSH(기본 hsm) JETSON_REPO(기본 ~/Code_Space, Jetson 측 경로) GAZEBO_GUI(false)
#             SMOKE_ENV(Jetson smoke 에 전달할 추가 env 문자열) POLL_TIMEOUT_S(기본 3000)
# 산출물: test_workspace/gazebo_world_swap/verification/dist_<ts>/ (dist.log, sim_host.log,
#         jetson_smoke.log, jetson_run/ = Jetson 의 run_<ts> 디렉터리 복사본)
# 종료 코드: Jetson smoke 의 종료 코드 (0=PASS). 호스트 Gazebo 는 항상 정리한다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
JETSON_SSH="${JETSON_SSH:-hsm}"
# Jetson 쪽 저장소 경로 (원격 셸이 ~ 를 펼친다 — 사용자 홈 하드코딩 금지, portability R1)
JETSON_REPO="${JETSON_REPO:-~/Code_Space}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
POLL_TIMEOUT_S="${POLL_TIMEOUT_S:-3000}"
# MAX_MISSED_RATE 60: Jetson 실측 기준선이 23~29(2026-08-21 3회, 데스크톱 0~3)라 30 은 임계 근접 —
# 폭주 회귀(수십~수백)만 막는 안전망으로 2배 여유. 실기 임계는 실센서 주행에서 별도 수립.
SMOKE_ENV="${SMOKE_ENV:-WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_STATIC_OBSTACLE=1 WITH_LIFT=1 WITH_ARM=1 WITH_PROFILE=1 NAV_GOAL_RETRIES=2 MAX_MISSED_RATE=60}"
RUN_TAG="dist_$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/test_workspace/gazebo_world_swap/verification/$RUN_TAG"
mkdir -p "$OUT"

log() { printf '[dist-e2e] %s %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$OUT/dist.log"; }
jssh() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$JETSON_SSH" "$@"; }

HOST_PID=""
cleanup() {
  local rc=$?
  trap '' INT TERM
  if [[ -n "$HOST_PID" ]]; then
    log "stopping desktop gazebo host (pid $HOST_PID)"
    kill -INT "$HOST_PID" 2>/dev/null || true
    sleep 3
    kill -TERM "$HOST_PID" 2>/dev/null || true
  fi
  docker exec ros2_humble bash -c "pkill -f '[g]azebo.launch.py' 2>/dev/null; pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null; true" >/dev/null 2>&1 || true
  log "done rc=$rc (artifacts: $OUT)"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# [0] Jetson 잔존 정리 (이전 smoke/노드) — 같은 컨테이너 이름이라 Jetson 쪽에서만 실행
log "[0] jetson: cleaning stale smoke/nav processes"
# 주의: 원격 bash -c 의 명령줄에는 아래 패턴 문자열이 그대로 들어 있어 pkill -f 가 자기 자신을 죽인다
# (실측: run1~3 "jetson cleanup ssh failed"). pgrep 결과에서 자기 pid($$)를 제외하고 kill 한다.
jssh "docker exec ros2_humble bash -c 'for p in run_l3_world_swap_smoke.sh component_container_isolated auto_floor_orchestrator world_swap pedestrians.py spawn_static_obstacles.py lift_cycle.py arm_sequence profile_resources.sh kku_navigation.launch.py; do for pid in \$(pgrep -f \"\$p\"); do [ \"\$pid\" != \"\$\$\" ] && kill \"\$pid\" 2>/dev/null; done; done; sleep 1; true'" >/dev/null 2>&1 || log "WARN: jetson cleanup ssh failed (continuing)"

# [1] 데스크톱 Gazebo 리셋 + 기동 + 스폰 확인
log "[1] desktop: reset + start gazebo host (run_sim_host.sh F1, gui=$GAZEBO_GUI)"
docker exec ros2_humble bash -c "pkill -f '[g]azebo.launch.py' 2>/dev/null; pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null; true" >/dev/null 2>&1 || true
for _ in $(seq 1 10); do docker exec ros2_humble pgrep -x gzserver >/dev/null 2>&1 || break; sleep 1; done
sleep 2
GAZEBO_GUI="$GAZEBO_GUI" nohup bash scripts/run_sim_host.sh F1 >"$OUT/sim_host.log" 2>&1 &
HOST_PID=$!
spawned=0
for _ in $(seq 1 90); do
  if grep -q "Successfully spawned entity \[elevator_robot_f1\]" "$OUT/sim_host.log" 2>/dev/null; then spawned=1; break; fi
  if ! kill -0 "$HOST_PID" 2>/dev/null; then break; fi
  sleep 2
done
if (( ! spawned )); then
  log "FAIL: desktop gazebo spawn not confirmed within 180s (see $OUT/sim_host.log)"
  tail -20 "$OUT/sim_host.log" | tee -a "$OUT/dist.log"
  exit 1
fi
log "desktop gazebo up: spawn confirmed"
sleep 5

# [2] Jetson smoke 기동 (컨테이너 안 nohup — ssh 가 끊겨도 계속)
JLOG="/ros2_ws/logs/smoke_${RUN_TAG}.log"
log "[2] jetson: starting smoke (GAZEBO_REMOTE=1 $SMOKE_ENV) -> $JLOG"
jssh "docker exec ros2_humble bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null; export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/scripts/fastdds_lan_peers.xml && ros2 daemon stop >/dev/null 2>&1; cd /ros2_ws && mkdir -p logs && GAZEBO_REMOTE=1 $SMOKE_ENV nohup bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh > $JLOG 2>&1 & sleep 2; echo started'" | tee -a "$OUT/dist.log"

# [3] 종료까지 폴링 (smoke 프로세스 생존 여부)
log "[3] polling jetson smoke (timeout ${POLL_TIMEOUT_S}s)"
start=$SECONDS
while true; do
  if ! jssh "docker exec ros2_humble pgrep -f run_l3_world_swap_smoke.sh >/dev/null 2>&1"; then
    break
  fi
  if (( SECONDS - start > POLL_TIMEOUT_S )); then
    log "TIMEOUT: jetson smoke still running after ${POLL_TIMEOUT_S}s — killing"
    jssh "docker exec ros2_humble pkill -TERM -f run_l3_world_swap_smoke.sh" >/dev/null 2>&1 || true
    break
  fi
  last=$(jssh "docker exec ros2_humble tail -1 $JLOG 2>/dev/null" 2>/dev/null | cut -c1-110 || true)
  log "  ... $(( SECONDS - start ))s: ${last}"
  sleep 30
done

# [4] 수집: Jetson 로그 + RUN_DIR 아티팩트
log "[4] collecting artifacts"
jssh "docker exec ros2_humble cat $JLOG" >"$OUT/jetson_smoke.log" 2>/dev/null || true
JRUN=$(grep -m1 -oP '^\[world-swap-smoke\] RUN_DIR=\K.*' "$OUT/jetson_smoke.log" || true)
if [[ -n "$JRUN" ]]; then
  # 컨테이너 경로 /ros2_ws/test_workspace/... -> Jetson 호스트 경로 $JETSON_REPO/test_workspace/...
  JHOST="${JRUN/#\/ros2_ws/$JETSON_REPO}"
  scp -q -r "$JETSON_SSH:$JHOST" "$OUT/jetson_run" 2>/dev/null || log "WARN: scp of $JHOST failed"
fi
rc=1
if grep -q "^\[world-swap-smoke\] logs written to" "$OUT/jetson_smoke.log" 2>/dev/null; then rc=0; fi
log "jetson smoke result: $([[ $rc == 0 ]] && echo PASS || echo FAIL) (summary: $OUT/jetson_run/scenario_summary.md)"
grep -E "ARM PASS|LIFT PASS|PASS world swap|SUCCEEDED after|CONTROL FIDELITY|REALIGN|STATIC OBSTACLES|logs written" "$OUT/jetson_smoke.log" | tail -15 | tee -a "$OUT/dist.log" || true
exit "$rc"
