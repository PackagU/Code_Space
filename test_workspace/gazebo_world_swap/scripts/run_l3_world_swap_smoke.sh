#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/ros2_ws}"
WORKSPACE="$ROOT/test_workspace/gazebo_world_swap"
OUT="$WORKSPACE/verification/latest"
KEEP_RUNNING="${KEEP_RUNNING:-0}"
WITH_RVIZ="${WITH_RVIZ:-false}"
# Gazebo GUI(gzclient) 관찰 옵션. 기본 false -> gzserver 헤드리스 (부하 측정 오염 방지).
# 데스크톱 관찰 시 GAZEBO_GUI=true.
GAZEBO_GUI="${GAZEBO_GUI:-false}"
# 분산 시뮬 모드(Jetson 부하테스트). GAZEBO_REMOTE=1 이면 Gazebo 를 이 머신에서 띄우지
# 않고, 같은 LAN 의 다른 머신(데스크톱 scripts/run_sim_host.sh)이 띄운 Gazebo 를
# DDS 로 쓴다. Gazebo Classic 은 arm64 바이너리가 없어 Jetson 에선 이 모드가 표준
# (docs/deployment/01_portability_policy.md §4.5). 기본 0 -> 로컬 Gazebo.
GAZEBO_REMOTE="${GAZEBO_REMOTE:-0}"
# 동적 장애물(보행자) 회피 연습 옵션. 기본 off -> 결정적 smoke 유지.
# WITH_PEDESTRIAN=1 이면 F1 복도를 가로질러 왕복하는 collision 보행자를 띄워
# 로봇이 LiDAR 로 감지하고 Nav2 로 회피하게 한다(실기 동적 회피 전이용).
WITH_PEDESTRIAN="${WITH_PEDESTRIAN:-0}"
# 회복 동작 자동검증 옵션. 기본 off -> 결정적 smoke 유지.
# WITH_RECOVERY=1 이면 정상 미션 끝에 '도달 불가능 goal'을 한 번 보내,
# Nav2 가 무한 회전/무한 재시도 없이 한정된 시간 안에 ABORTED 로 안전 종료하는지 확인한다.
WITH_RECOVERY="${WITH_RECOVERY:-0}"
# Localization 고장주입 옵션. 기본 off -> 결정적 smoke 유지.
# WITH_LOC_FAULT=1 이면 마지막 F2 복도 goal 직전 '0.5m 어긋난 initialpose'를 주입해,
# AMCL + scan 매칭이 한정된 오차를 보정하고도 goal 에 도달하는지(강건성) 확인한다.
WITH_LOC_FAULT="${WITH_LOC_FAULT:-0}"
# 부하 프로파일 옵션. 기본 off -> 결정적 smoke 유지.
# WITH_PROFILE=1 이면 profile_resources.sh 를 백그라운드로 띄워 CPU/메모리/런타임을
# 샘플링한다(Jetson Xavier NX 실기 부하 대비용). 산출물은 $OUT/profile/.
WITH_PROFILE="${WITH_PROFILE:-0}"
# 제어 충실도 회귀 가드(선택, P0 계측). MAX_MISSED_RATE=N 이면 nav2 컨트롤러의
# 'control loop missed its desired rate' 발생이 N 회 초과 시 smoke 를 FAIL 시킨다
# (부하로 제어 주기가 깨지는 회귀를 차단). 기본 미설정 -> 계측만 기록, FAIL 안 함.
MAX_MISSED_RATE="${MAX_MISSED_RATE:-}"
# 부하 주입 robustness 검증(P2). WITH_STRESS=N 이면 미션 동안 CPU 점유 워커 N개를 띄워
# 의도적으로 부하를 준다. P1 graceful degradation 으로 미션이 그래도 완주하는지와
# 제어 계측(missed_rate 상승)을 함께 확인한다. 기본 0 -> 부하 없음(결정적 smoke 유지).
WITH_STRESS="${WITH_STRESS:-0}"
# 제어 연산 보호(P3). WITH_RT_PRIORITY=1 이면 nav2 컨테이너에 RT 우선순위(SCHED_RR)+
# renice 를 부여해, 부하(stress/gazebo)가 제어 루프를 굶기지 못하게 한다. 기본 0.
# 실기 Jetson 에서는 동등하게 nav2 를 RT prio/전용 코어로 구동(문서 참조).
WITH_RT_PRIORITY="${WITH_RT_PRIORITY:-0}"
# WITH_F3=1 이면 F2 검증 후 두 번째 층 전환(F2->F3)까지 수행한다:
# 엘베 복귀 -> target_floor=F3 param set(레거시 SwitchFloor 와 동일 계약) ->
# request_switch -> ARRIVED_OPEN -> F3 world/map 검증 -> F3 복도 goal. 기본 0.
WITH_F3="${WITH_F3:-0}"
# 로봇팔 통합 옵션. 기본 off -> 결정적 smoke 유지.
# WITH_ARM=1 이면 robot_arm_pkg arm_sequence 노드를 띄워, 층 전환 완료(phase=ready)마다
# 하드코딩 버튼 시퀀스(50Hz JointState)가 시작·완료되는지 로그로 검증한다
# (Jetson 시뮬 부하테스트 + 실서보 통합용). ARM_SERIAL_PORT 지정 시 실서보로도 전송
# (예: /dev/arm_servo, 기본 빈 값=토픽 전용).
WITH_ARM="${WITH_ARM:-0}"
ARM_SERIAL_PORT="${ARM_SERIAL_PORT:-}"
# 왕복 배달 옵션. WITH_RETURN=1 이면 F2 배달(f2_corridor) 후 엘베로 복귀해
# F2->F1 역전환 -> F1 충전소 복귀까지 수행한다(한 층 왕복 완성 체인).
# 기본 0 -> 기존 결정적 smoke 유지. WITH_F3 와 동시 사용 금지(왕복은 F1<->F2 전용).
WITH_RETURN="${WITH_RETURN:-0}"
if [[ "$WITH_RETURN" == "1" && "$WITH_F3" == "1" ]]; then
  echo "error: WITH_RETURN=1 과 WITH_F3=1 은 동시 사용 불가 (왕복은 F1<->F2 전용)" >&2
  exit 1
fi
# 정적 장애물 옵션. WITH_STATIC_OBSTACLE=1 이면 복도에 정지 보행자(정적 장애물)를
# 스폰해 로봇이 '멈춰 서 있는 사람'을 우회하는지 검증한다(최종 시나리오의 정적
# 장애물 요구 — 동적 보행자와 독립 옵션). 기본 0 -> 결정적 smoke 유지.
WITH_STATIC_OBSTACLE="${WITH_STATIC_OBSTACLE:-0}"
# 리프트(Z축) 동작 옵션. WITH_LIFT=1 이면 택배 픽업 직후와 F2 하역 시점에
# lift_joint 상승/하강 사이클을 실행·검증한다(최종 시나리오의 리프트 동작 —
# URDF joint trajectory 플러그인 + /joint_states 검증). 기본 0.
WITH_LIFT="${WITH_LIFT:-0}"
# nav goal 재시도 횟수. 보행자 런 기본 2(조우로 인한 일시 ABORTED 흡수),
# 결정적 smoke 기본 0(회귀를 재시도로 가리지 않기). 명시 설정이 우선.
if [[ -z "${NAV_GOAL_RETRIES:-}" ]]; then
  if [[ "${WITH_PEDESTRIAN}" == "1" ]]; then
    NAV_GOAL_RETRIES=2
  else
    NAV_GOAL_RETRIES=0
  fi
fi

# 타임스탬프 run 디렉터리(아티팩트 누적용). latest 는 back-compat 으로 유지.
RUN_TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$WORKSPACE/verification/run_$RUN_TS"

mkdir -p "$OUT" "$RUN_DIR"
rm -f "$OUT"/*.log "$OUT"/*.txt

PIDS=()
STRESS_PIDS=()

log() {
  printf '[world-swap-smoke] %s\n' "$*"
}

source_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

kill_matching() {
  local pattern="$1"
  mapfile -t matches < <(pgrep -f "$pattern" || true)
  for pid in "${matches[@]}"; do
    if [[ "$pid" != "$$" && "$pid" != "${BASHPID:-$$}" ]]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
}

cleanup_started() {
  # 정리 중 2차 INT/TERM 은 무시 — 정리 도중 끊기면 finalize(아티팩트 보존)가 중단된다
  # (2026-08-21 fg-INT 테스트: 오류 종료 직후 INT 가 EXIT trap 을 끊어 finalize=0). 정리는
  # 대기가 전부 유한(최대 ~20s)이라 안전. 종료 코드는 trap 진입 전 값(130/143/1)이 보존된다.
  trap '' INT TERM
  # 미션이 실패해도 제어 계측은 남긴다(부하 실패 측정). nav 노드 kill 전에 먼저 기록.
  write_control_metrics 2>/dev/null || true
  # 실패 경로 아티팩트 보존 (2026-08-20 리뷰 findings #13): finalize 를 못 거치고
  # 죽으면 증거가 latest/ 에만 남아 다음 런의 rm -f 에 파괴됐다(Jetson 빈 실패
  # 디렉터리 4개 실증). trap 에서 멱등 finalize 로 항상 run_<ts>/ 에 스냅샷을 남긴다.
  finalize_artifacts 2>/dev/null || true
  # 부하 주입 워커는 KEEP_RUNNING 과 무관하게 항상 정리(미션용 부하일 뿐).
  for pid in "${STRESS_PIDS[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  if [[ "$KEEP_RUNNING" == "1" ]]; then
    log "KEEP_RUNNING=1, leaving launched processes alive"
    return
  fi
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  cleanup_stale
}

cleanup_stale() {
  log "cleaning stale simulation processes"
  kill_matching "ros2 launch common_pkg gazebo.launch.py"
  kill_matching "ros2 launch slam_pkg kku_navigation.launch.py"
  kill_matching "rviz2"
  kill_matching "auto_floor_orchestrator.launch.py"
  kill_matching "world_swap.launch.py"
  kill_matching "auto_floor_orchestrator_node"
  kill_matching "world_swap_node"
  kill_matching "component_container_isolated"
  kill_matching "robot_state_publisher"
  kill_matching "spawn_entity.py"
  kill_matching "pedestrians.py"
  kill_matching "arm_sequence"
  kill_matching "profile_resources.sh"
  kill_matching "gzserver"
  kill_matching "gzclient"
  # 2026-08-20: 죽어가는 gzserver 와 새 gzserver 가 겹치면(마스터 포트 11345 인계 레이스)
  # 로봇이 "무명령 활주(6mm/s, 마찰 무력)" 모드로 스폰되는 사례를 반복 재현(§1.22 의 활주는
  # 기동 레이스가 방아쇠였다 — journal 2026-08-20_review_followup §2). 완전히 사라질 때까지
  # 기다린 뒤(최대 10s, 잔존 시 SIGKILL) 2s 안정화한다.
  # pgrep -x(프로세스명 정확 일치): -f 는 이 스크립트를 감싼 셸의 명령줄에 'gzserver' 가
  # 들어 있으면 자기 자신을 잡아 영원히 기다린다(실측: 래퍼 bash -c 문자열 자기매칭).
  local w
  for w in $(seq 1 10); do
    pgrep -x gzserver >/dev/null 2>&1 || break
    sleep 1
  done
  if pgrep -x gzserver >/dev/null 2>&1; then
    log "stale gzserver still alive after 10s -> SIGKILL"
    pkill -9 -x gzserver 2>/dev/null || true
    sleep 1
  fi
  sleep 2
}

# 주의: 아래 wait_for_* 의 CLI 는 반드시 timeout 으로 감싼다 (2026-08-20 리뷰
# findings #14) — daemon 오염 등으로 CLI 자체가 행하면 루프가 다음 반복으로 못 가
# timeout 검사에 영원히 도달하지 못한다(§1.23f 의 /clock 멈춤이 이 경로).
wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local end=$((SECONDS + timeout_sec))
  until timeout 10 ros2 topic list >"$OUT/topics.tmp" 2>/dev/null && grep -qx "$topic" "$OUT/topics.tmp"; do
    if (( SECONDS >= end )); then
      log "timeout waiting for topic $topic"
      return 1
    fi
    sleep 1
  done
}

wait_for_service() {
  local service="$1"
  local timeout_sec="$2"
  local end=$((SECONDS + timeout_sec))
  until timeout 10 ros2 service list >"$OUT/services.tmp" 2>/dev/null && grep -qx "$service" "$OUT/services.tmp"; do
    if (( SECONDS >= end )); then
      log "timeout waiting for service $service"
      return 1
    fi
    sleep 1
  done
}

wait_for_action() {
  local action="$1"
  local timeout_sec="$2"
  local end=$((SECONDS + timeout_sec))
  until timeout 10 ros2 action list >"$OUT/actions.tmp" 2>/dev/null && grep -qx "$action" "$OUT/actions.tmp"; do
    if (( SECONDS >= end )); then
      log "timeout waiting for action $action"
      return 1
    fi
    sleep 1
  done
}

wait_for_lifecycle_active() {
  # action 이 목록에 떠도(configured) lifecycle 이 active 전이면 goal 이 거부된다
  # (시작 레이스로 첫 goal "Goal was rejected" 재현, 2026-07-03). active 를 명시 대기.
  local node="$1"
  local timeout_sec="$2"
  local end=$((SECONDS + timeout_sec))
  # ros2 lifecycle get 은 component container 노드에서 node graph 검증에 걸려
  # 실패할 수 있어 get_state 서비스를 직접 호출한다.
  until timeout 10 ros2 service call "${node}/get_state" lifecycle_msgs/srv/GetState 2>/dev/null \
      | grep -q "label='active'"; do
    if (( SECONDS >= end )); then
      log "timeout waiting for lifecycle active: $node"
      return 1
    fi
    sleep 1
  done
}

start_bg() {
  local name="$1"
  shift
  log "starting $name"
  "$@" >"$OUT/$name.log" 2>&1 &
  PIDS+=("$!")
}

verify_arm_sequence() {
  # 층 전환 후 팔 버튼 시퀀스가 시작(해당 층 트리거)·완료됐는지 노드 로그로 확인한다.
  # 2026-08-20 리뷰 findings #20: 완료 로그에 층이 없어 누계 카운트는 층 무구분이었다
  # (F1 복귀 시 F2 완료 2회면 F1 미완료여도 통과). "해당 층 시작 줄 이후에 찍힌 완료"가
  # 1개 이상인지(층별 쌍)로 판정하고, min_complete 는 전체 누계 하한으로만 쓴다.
  local floor="$1"
  local timeout_sec="$2"
  local min_complete="${3:-1}"
  local end=$((SECONDS + timeout_sec))
  while true; do
    if grep -q "버튼 시퀀스 시작 ($floor)" "$OUT/arm_sequence.log" 2>/dev/null \
      && [[ "$(grep -c "버튼 시퀀스 완료" "$OUT/arm_sequence.log" 2>/dev/null)" -ge "$min_complete" ]] \
      && [[ "$(awk -v f="버튼 시퀀스 시작 ($floor)" 'index($0,f){seen=1;c=0;next} seen&&index($0,"버튼 시퀀스 완료"){c++} END{print c+0}' "$OUT/arm_sequence.log" 2>/dev/null)" -ge 1 ]]; then
      log "ARM PASS: $floor 버튼 시퀀스 시작+완료 확인 (층별 쌍)"
      return 0
    fi
    if (( SECONDS >= end )); then
      log "ARM FAIL: $floor 버튼 시퀀스 로그 미확인 (${timeout_sec}s)"
      cat "$OUT/arm_sequence.log" 2>/dev/null || true
      return 1
    fi
    sleep 2
  done
}

publish_initial_pose() {
  local name="$1"
  local x="$2"
  local y="$3"
  local yaw_z="$4"
  local yaw_w="$5"
  log "publishing initial pose $name"
  # AMCL 구독자가 아직 준비 전이면 발행분이 전부 유실될 수 있다(재현: 2026-07-03,
  # "AMCL cannot publish a pose ... set the initial pose" 반복 -> nav2 activation 데드락).
  # 발행 후 /amcl_pose 실제 수신(적용 증거)까지 확인하고, 미적용이면 재발행한다.
  local try
  for try in 1 2 3 4 5 6; do
    ros2 topic pub --times 5 --rate 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {z: $yaw_z, w: $yaw_w}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}" \
      >"$OUT/initialpose_$name.log" 2>&1
    if timeout 15 ros2 topic echo --once /amcl_pose >>"$OUT/initialpose_$name.log" 2>&1; then
      return 0
    fi
    log "initial pose not applied by AMCL yet (try $try/6) -> republishing"
  done
  log "AMCL did not accept initial pose $name"
  return 1
}

send_nav_goal() {
  local name="$1"
  local x="$2"
  local y="$3"
  local yaw_z="$4"
  local yaw_w="$5"
  local timeout_sec="$6"
  # 보행자 런에서는 좁은 복도 조우로 controller patience 가 초과돼 ABORTED 될 수 있다
  # (동적 장애물은 수 초면 지나감). 실기 배달 로봇과 동일하게 goal 재요청으로 흡수한다.
  # 결정적 smoke(보행자 0)는 재시도 0 회 -> 회귀를 가리지 않는다.
  local retries="${NAV_GOAL_RETRIES:-0}"
  local attempt
  for attempt in $(seq 0 "$retries"); do
    if (( attempt > 0 )); then
      log "goal $name failed (attempt $attempt/$retries) -> clearing costmaps + retrying in 10s (waiting for dynamic obstacle to clear)"
      # 이전 시도의 goal 이 액션 서버에 살아있을 수 있다(send_goal CLI 는 timeout
      # SIGTERM 에 cancel 을 보내지 않음, findings #15). 살아있는 goal 위에서
      # initialpose 재발행/costmap 클리어를 하면 주행 중 재정위가 된다 — 먼저 취소.
      timeout 10 ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}" \
        >/dev/null 2>&1 || true
      dump_world_state "${name}_a${attempt}"
      realign_belief_if_drifted
      # 떠난 장애물의 stale lethal 마크가 통로를 계속 봉쇄하는 사례 대응
      # (2026-08-17 반복 런 run_05: F2 엘베 출구 450s collision-ahead — §1.25).
      local clear_svc
      for clear_svc in "/global_costmap/clear_entirely_global_costmap" "/local_costmap/clear_entirely_local_costmap"; do
        timeout 20 ros2 service call "$clear_svc" nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1 || true
      done
      sleep 10
    fi
    log "sending navigation goal $name"
    # 서버측 증거 병행 판정 (2026-08-21 G004 run2 실측): rmw_fastrtps 가 goal 응답을 유실하면
    # (nav2 로그 "[bt_navigator.rclcpp_action] Failed to send goal response (timeout)") CLI 는
    # 결과를 영영 못 받아 timeout 까지 대기 → 이미 도착한 goal 을 실패로 세고 150s+재시도를
    # 낭비한다(§1.23e 와 같은 응답 유실 계열). CLI 를 백그라운드로 띄우고 앵커 이후의
    # bt_navigator "Goal succeeded/failed" 로그로 조기 판정한다. 참값(시뮬 오라클)은 쓰지 않는다.
    local anchor cli_pid verdict="" tailn g
    anchor=$(wc -l <"$OUT/nav2_f1.log" 2>/dev/null || echo 0)
    timeout "${timeout_sec}s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {z: $yaw_z, w: $yaw_w}}}}" \
      >"$OUT/nav2_goal_$name.log" 2>&1 &
    cli_pid=$!
    while true; do
      if ! kill -0 "$cli_pid" 2>/dev/null; then
        wait "$cli_pid" 2>/dev/null || true
        if grep -q "status: SUCCEEDED" "$OUT/nav2_goal_$name.log"; then verdict=ok; else verdict=failed; fi
        break
      fi
      tailn=$(tail -n +$((anchor + 1)) "$OUT/nav2_f1.log" 2>/dev/null \
        | grep -E "\[bt_navigator\]: Goal (succeeded|failed)" | tail -1 || true)
      if [[ "$tailn" == *"Goal succeeded"* || "$tailn" == *"Goal failed"* ]]; then
        # CLI 가 결과를 곧 받을 수 있으니 5s 유예 후 판정
        for g in 1 2 3 4 5; do kill -0 "$cli_pid" 2>/dev/null || break; sleep 1; done
        if kill -0 "$cli_pid" 2>/dev/null; then
          kill -TERM "$cli_pid" 2>/dev/null || true
          wait "$cli_pid" 2>/dev/null || true
          if [[ "$tailn" == *"Goal succeeded"* ]]; then
            echo "SERVER-EVIDENCE: bt_navigator 'Goal succeeded' after anchor; client response lost" >>"$OUT/nav2_goal_$name.log"
            verdict=lost_ok
          else
            echo "SERVER-EVIDENCE: bt_navigator 'Goal failed' after anchor; client response lost" >>"$OUT/nav2_goal_$name.log"
            verdict=failed
          fi
        else
          wait "$cli_pid" 2>/dev/null || true
          if grep -q "status: SUCCEEDED" "$OUT/nav2_goal_$name.log"; then verdict=ok
          elif [[ "$tailn" == *"Goal succeeded"* ]]; then verdict=lost_ok
          else verdict=failed; fi
        fi
        break
      fi
      sleep 2
    done
    if [[ "$verdict" == "ok" || "$verdict" == "lost_ok" ]]; then
      grep "status: SUCCEEDED" "$OUT/nav2_goal_$name.log" || true
      if [[ "$verdict" == "lost_ok" ]]; then
        log "goal $name: server reported success, client response lost -> accepting (response-loss absorbed, §1.23e)"
        cp -f "$OUT/nav2_goal_$name.log" "$OUT/nav2_goal_${name}_a${attempt}.responselost.log" 2>/dev/null || true
      fi
      if (( attempt > 0 )); then
        # 재시도로 살아난 goal 은 요약 표에서 구분되게 표식을 남긴다 (findings #23).
        log "goal $name SUCCEEDED after $attempt retry(ies) — recovered, not clean"
      fi
      return 0
    fi
    # 실패한 attempt 의 로그를 보존한다 — 다음 attempt 가 같은 파일을 덮어써 재시도 이력이
    # 사라지던 문제(findings #23). 요약 표는 마지막 attempt 결과만 반영하므로 attempt 사본으로
    # 실패 원인(ABORTED 사유)을 사후 판독할 수 있게 한다.
    cp -f "$OUT/nav2_goal_$name.log" "$OUT/nav2_goal_${name}_a${attempt}.fail.log" 2>/dev/null || true
  done
  # 최종 실패 시에도 잔존 goal 취소 — 이후 정리/스냅샷이 주행 중 로봇 위에서 돌지 않게.
  timeout 10 ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}" \
    >/dev/null 2>&1 || true
  dump_world_state "${name}_final"
  cat "$OUT/nav2_goal_$name.log"
  return 1
}

set_costmap_footprint() {
  # local/global costmap 의 footprint(string param)를 polygon 문자열로 교체.
  # 주의: "[]" 는 ros2 param set 이 bool_array 로 파싱해 type error 로 무음 실패했던
  # 이력이 있다(F2 엘베 갇힘 원인). 반드시 좌표 polygon 문자열만 넘길 것.
  local tag="$1"
  local polygon="$2"
  local scope
  for scope in local global; do
    local logf="$OUT/footprint_${tag}_${scope}.log"
    # timeout+재시도: Jetson 로컬 CLI가 산발적 discovery 실패로 무한 대기하는 사례 실측 (§1.23)
    local attempt ok=0
    for attempt in 1 2 3; do
      timeout 30 ros2 param set "/${scope}_costmap/${scope}_costmap" footprint "$polygon" >"$logf" 2>&1 || true
      if grep -q "Set parameter successful" "$logf"; then ok=1; break; fi
      log "footprint set retry: tag=$tag scope=${scope}_costmap attempt=$attempt/3"
    done
    if (( ! ok )); then
      log "FOOTPRINT SET FAILED: tag=$tag scope=${scope}_costmap"
      cat "$logf"
      exit 1
    fi
  done
}

dump_world_state() {
  # 실패 시점 시뮬 참값 스냅샷(§1.25 산발 봉쇄 원인 판별): 전체 모델 pose + AMCL belief.
  local tag="$1"
  local logf="$OUT/world_state_${tag}.log"
  {
    echo "== world state snapshot: $tag =="
    local models m
    models=$(timeout 15 ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}" 2>/dev/null \
      | grep -oP "model_names=\[\K[^]]*" | tr -d "' " | tr ',' ' ') || true
    for m in $models; do
      printf '%s ' "$m"
      timeout 10 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState "{name: '$m'}" 2>/dev/null \
        | grep -oP "position=geometry_msgs\.msg\.Point\([^)]*\)" | head -1 || echo "?"
    done
    echo "-- amcl belief --"
    timeout 10 bash -c "ros2 topic echo --once /amcl_pose | head -20" 2>/dev/null || echo "amcl_pose unavailable"
  } >"$logf" 2>&1 || true
  log "world state snapshot saved: world_state_${tag}.log"
}

realign_belief_if_drifted() {
  # 시뮬 한정 재정위(§1.25): AMCL belief 와 시뮬 참값 오프셋이 0.3m 초과면 참값으로
  # initialpose 재발행 — 잘못된 좌표계에 스캔 벽이 재마킹되는 자기강화 봉쇄를 끊는다.
  # 실기의 도킹/QR 재정위에 해당하는 시뮬 오라클 부기(§1.24와 동일 부류) — 로그에 명시.
  local resp belief tx ty tz tw bx by dist
  resp=$(timeout 10 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
    "{name: 'elevator_robot_f1'}" 2>/dev/null | tr -d '\n') || true
  tx=$(grep -oP "position=geometry_msgs\.msg\.Point\(x=\K[-0-9.e+]+" <<<"$resp" | head -1)
  ty=$(grep -oP "position=geometry_msgs\.msg\.Point\(x=[-0-9.e+]+, y=\K[-0-9.e+]+" <<<"$resp" | head -1)
  tz=$(grep -oP "orientation=geometry_msgs\.msg\.Quaternion\(x=[-0-9.e+]+, y=[-0-9.e+]+, z=\K[-0-9.e+]+" <<<"$resp" | head -1)
  tw=$(grep -oP "orientation=geometry_msgs\.msg\.Quaternion\([^)]*w=\K[-0-9.e+]+" <<<"$resp" | head -1)
  belief=$(timeout 10 bash -c "ros2 topic echo --once /amcl_pose" 2>/dev/null | tr -d '\n') || true
  bx=$(grep -oP "x: \K[-0-9.e+]+" <<<"$belief" | head -1)
  by=$(grep -oP "y: \K[-0-9.e+]+" <<<"$belief" | head -1)
  # 수치 검증 (findings #16): grep 문자클래스가 '.'/'e' 단독 캡처를 허용해 awk 소스가
  # 깨지면 set -e 로 스모크 전체가 즉사한다 — 정규식으로 완전한 수치만 통과시킨다.
  local num_re='^-?[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$'
  local v
  for v in "$tx" "$ty" "$tz" "$tw" "$bx" "$by"; do
    if [[ ! "$v" =~ $num_re ]]; then
      log "belief drift check skipped (unparsable pose: true=($tx,$ty) belief=($bx,$by))"
      return 0
    fi
  done
  dist=$(awk "BEGIN{printf \"%.3f\", sqrt(($tx-($bx))^2 + ($ty-($by))^2)}")
  log "belief drift: true=($tx,$ty) belief=($bx,$by) dist=${dist}m"
  if awk "BEGIN{exit !($dist > 0.3)}"; then
    log "REALIGN: belief drift ${dist}m > 0.3m -> republishing initialpose at true pose (sim-only bookkeeping, §1.25)"
    publish_initial_pose realign "$tx" "$ty" "$tz" "$tw" || true
  fi
}

set_orchestrator_target_floor() {
  # target_floor param set 을 timeout+재시도로 감싼다 (로컬 CLI half-hang 흡수 — §1.23e).
  local floor="$1"
  local logf="$OUT/param_target_$(echo "$floor" | tr 'A-Z' 'a-z').log"
  local attempt
  for attempt in 1 2 3; do
    timeout 30 ros2 param set /floor_orchestrator_node target_floor "$floor" >"$logf" 2>&1 || true
    if grep -q "Set parameter successful" "$logf"; then return 0; fi
    log "target_floor set retry: floor=$floor attempt=$attempt/3"
  done
  log "TARGET FLOOR SET FAILED: $floor"
  cat "$logf"
  return 1
}

request_floor_switch() {
  # request_switch 호출 + half-hang 흡수(§1.23e): 응답 유실 시(서버는 armed 완료,
  # rmw 'failed to send response' — 2026-08-17 데스크톱 반복 런 run_02 실측)
  # orchestrator 로그의 armed 증거로 성공을 판정한다. 증거도 없으면 1회 재호출.
  # 2026-08-20 리뷰 findings #4/#19: (a) ros2 service call 은 서버가 거절
  # (success=False, busy 등)해도 rc=0 이라 응답 본문 success=True 로 판정한다.
  # (b) armed 증거는 이 호출 이후에 새로 찍힌 로그에서만 찾는다(시간 앵커) —
  # 같은 층 재방문/재호출 시 과거 armed 라인 재사용 방지.
  local floor="$1"
  local logf="$OUT/request_switch_$(echo "$floor" | tr 'A-Z' 'a-z').log"
  local anchor
  anchor=$(wc -l <"$OUT/orchestrator.log" 2>/dev/null || echo 0)
  local attempt
  for attempt in 1 2; do
    if timeout 45 ros2 service call /floor_orchestrator/request_switch std_srvs/srv/Trigger >"$logf" 2>&1 \
      && grep -q "success=True" "$logf"; then
      return 0
    fi
    if grep -q "success=False" "$logf"; then
      log "request_switch($floor) REJECTED by server (attempt $attempt/2)"
      cat "$logf"
    elif tail -n +$((anchor + 1)) "$OUT/orchestrator.log" 2>/dev/null \
      | grep -q "auto floor switch armed: target=$floor"; then
      log "request_switch($floor) response lost but server armed — continuing (§1.23e half-hang absorbed)"
      return 0
    fi
    log "request_switch($floor) attempt $attempt/2 failed without armed evidence -> retry"
  done
  log "request_switch call failed/timed out: $floor"
  cat "$logf"
  return 1
}

align_robot_to_spawn() {
  # 층 전환 직후 로봇 실위치를 스폰(=orchestrator initialpose) 좌표 (0,0)으로 정렬.
  # elevator_inside goal 이 tolerance 한계(~0.5m 오프셋)로 성공한 채 전환되면 belief
  # 오프셋 탓에 스캔 벽이 문 통로 위에 그려져 'no valid path' ABORT (2026-08-17 실측).
  # 실물 엘베는 물리 연속이라 없는 문제 — 시뮬 world-swap 의 위치 연속성 부기.
  local floor="$1"
  log "aligning robot to $floor elevator spawn + clearing costmaps"
  timeout 30 ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
    "{state: {name: elevator_robot_f1, pose: {position: {x: 0.0, y: 0.0, z: 0.05}, orientation: {z: 0.0, w: 1.0}}}}" \
    >"$OUT/align_${floor}.log" 2>&1 || log "WARN: $floor teleport failed (continuing)"
  sleep 2
  local svc
  for svc in "/global_costmap/clear_entirely_global_costmap" "/local_costmap/clear_entirely_local_costmap"; do
    timeout 20 ros2 service call "$svc" nav2_msgs/srv/ClearEntireCostmap "{}" >>"$OUT/align_${floor}.log" 2>&1 \
      || timeout 20 ros2 service call "$svc" nav2_msgs/srv/ClearEntireCostmap "{}" >>"$OUT/align_${floor}.log" 2>&1 \
      || log "WARN: $floor costmap clear failed: $svc (continuing)"
  done
  sleep 1
}

start_pedestrian() {
  local ped_dir="$WORKSPACE/pedestrian"
  log "starting pedestrians (dynamic obstacles, waypoint-driven)"
  # 노드가 여러 보행자를 스폰하고 waypoint 경로로 이동시킨다.
  start_bg pedestrians python3 "$ped_dir/pedestrians.py"
}

spawn_static_obstacles() {
  # 정적 장애물(정지 보행자) 스폰 — 원샷, 실패 시 즉시 FAIL (장애물 없이 통과하면
  # '정적 장애물 하 완주' 검증이 무의미해지므로 fail-closed).
  log "spawning static obstacles (stationary pedestrians)"
  if ! timeout 60 python3 "$WORKSPACE/pedestrian/spawn_static_obstacles.py" \
    >"$OUT/static_obstacles.log" 2>&1; then
    log "STATIC OBSTACLE SPAWN FAILED"
    cat "$OUT/static_obstacles.log"
    exit 1
  fi
}

run_lift_cycle() {
  # 리프트 상승→하강 사이클 + /joint_states 실측 검증 (WITH_LIFT=1).
  local tag="$1"
  log "running lift cycle: $tag (up 0.30 -> down 0.0, verified via /joint_states)"
  if ! timeout 90 python3 "$WORKSPACE/scripts/lift_cycle.py" --tag "$tag" \
    >"$OUT/lift_$tag.log" 2>&1; then
    log "LIFT FAIL: $tag"
    cat "$OUT/lift_$tag.log"
    exit 1
  fi
  grep "LIFT PASS" "$OUT/lift_$tag.log" || true
}

start_profiler() {
  log "starting resource profiler (CPU/mem/runtime sampling)"
  mkdir -p "$OUT/profile"
  # 이전 회차 산출물 제거 — 이번 런 파일만 아카이브되게 한다(findings #3).
  rm -f "$OUT/profile/resource_summary.txt" "$OUT/profile/resource_samples.csv"
  start_bg profile bash "$WORKSPACE/scripts/profile_resources.sh" \
    --out "$OUT/profile" --interval 2 --label smoke
  PROFILE_PID="${PIDS[${#PIDS[@]}-1]}"
}

start_stress() {
  # P2: CPU 점유 워커 N개로 의도적 부하 주입. 미션이 그래도 완주하는지(robustness) 검증용.
  local n="$1"
  log "injecting CPU stress: $n busy workers (P2 load test)"
  local i
  for ((i = 0; i < n; i++)); do
    bash -c 'while :; do :; done' >/dev/null 2>&1 &
    STRESS_PIDS+=("$!")
  done
}

apply_rt_priority() {
  # P3 compute 보호: nav2 컨테이너에 SCHED_RR(RT) + renice 부여.
  # RT 스레드는 SCHED_OTHER(stress/gazebo)를 선점 -> 부하 중에도 제어 루프 유지.
  local navpids p
  mapfile -t navpids < <(pgrep -f component_container_isolated)
  if [[ ${#navpids[@]} -eq 0 ]]; then
    log "P3 WARN: no nav2 container (component_container_isolated) found for RT boost"
    return
  fi
  for p in "${navpids[@]}"; do
    renice -n -10 -p "$p" >/dev/null 2>&1 || true
    chrt -a -r -p 10 "$p" 2>/dev/null || chrt -r -p 10 "$p" 2>/dev/null || true
  done
  log "P3: RT priority(SCHED_RR/10) + renice(-10) applied to ${#navpids[@]} nav2 process(es)"
}

inject_wrong_initialpose() {
  # localization 고장주입: 실제보다 (dx,dy) 만큼 어긋난 initialpose 를 발행한다.
  local name="$1"
  local x="$2"
  local y="$3"
  log "INJECT wrong initialpose $name at ($x,$y) (localization fault)"
  ros2 topic pub --times 5 --rate 2 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25]}}" \
    >"$OUT/initialpose_fault_$name.log" 2>&1
}

expect_nav_abort() {
  # 회복 동작 검증: '도달 불가능 goal'을 보내, Nav2 가 무한 회전/무한 재시도 없이
  # 한정된 시간(timeout) 안에 ABORTED 로 안전 종료하는지 확인한다.
  # PASS 조건: timeout 안에 액션이 반환되고, 그 결과가 SUCCEEDED 가 아님(ABORTED/CANCELED).
  # FAIL 조건: timeout 으로 죽음(무한 hang) 또는 SUCCEEDED(도달 불가 goal 이 성공할 수 없음).
  local name="$1"
  local x="$2"
  local y="$3"
  local timeout_sec="$4"
  log "RECOVERY check: sending unreachable goal $name ($x,$y), expecting bounded ABORTED"
  if timeout "${timeout_sec}s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" \
    >"$OUT/nav2_recovery_$name.log" 2>&1; then
    if grep -q "status: SUCCEEDED" "$OUT/nav2_recovery_$name.log"; then
      log "RECOVERY FAIL: unreachable goal reported SUCCEEDED"
      cat "$OUT/nav2_recovery_$name.log"
      return 1
    fi
    log "RECOVERY PASS: goal returned within ${timeout_sec}s without success (bounded safe stop)"
    grep -E "status: (ABORTED|CANCELED)" "$OUT/nav2_recovery_$name.log" || true
    return 0
  fi
  log "RECOVERY FAIL: action did not return within ${timeout_sec}s (possible infinite retry/hang)"
  cat "$OUT/nav2_recovery_$name.log"
  return 1
}

write_control_metrics() {
  # P0 제어 충실도 계측: nav2 컨트롤러 로그에서 주기 깨짐/ TF 지연/진행 실패 횟수.
  # 미션이 실패(set -e exit)해도 EXIT trap 에서 호출되어 항상 기록되게 한다(부하 실패도 측정).
  [[ -d "$OUT" ]] || return 0
  local navlog="$OUT/nav2_f1.log"
  local m_missed m_tf m_prog
  m_missed=$(grep -c "missed its desired rate" "$navlog" 2>/dev/null || true); m_missed=${m_missed:-0}
  m_tf=$(grep -c "extrapolation into the future" "$navlog" 2>/dev/null || true); m_tf=${m_tf:-0}
  m_prog=$(grep -c "Failed to make progress" "$navlog" 2>/dev/null || true); m_prog=${m_prog:-0}
  {
    echo "control_loop_missed_rate=$m_missed"
    echo "tf_extrapolation=$m_tf"
    echo "controller_failed_progress=$m_prog"
  } > "$OUT/control_metrics.txt"
}

finalize_artifacts() {
  # 로그/아티팩트 자동 수집: scenario_summary.md 작성 후 latest -> 타임스탬프 dir 로 복사.
  # 멱등: EXIT trap 과 본문 양쪽에서 호출될 수 있다(실패 경로 보존, findings #13).
  if [[ "${FINALIZED:-0}" == "1" ]]; then return 0; fi
  FINALIZED=1
  local summary="$OUT/scenario_summary.md"
  # 프로파일 flush (2026-08-20 리뷰 findings #3): resource_summary.txt 는 프로파일러의
  # SIGTERM 핸들러에서만 생성된다 — 복사 전에 먼저 종료시키고 이번 런 파일 생성을
  # 기다리지 않으면 이전 회차 summary 가 아카이브돼 repeat 표 cpu/mem 이 전부
  # 오귀속된다(run_01~03 md5 동일 실증).
  if [[ -n "${PROFILE_PID:-}" ]]; then
    kill -TERM "$PROFILE_PID" 2>/dev/null || true
    local w
    for w in 1 2 3 4 5 6 7 8 9 10; do
      [[ -s "$OUT/profile/resource_summary.txt" ]] && break
      sleep 1
    done
    PROFILE_PID=""
  fi
  write_control_metrics
  {
    echo "# world-swap smoke scenario summary"
    echo
    echo "- run_ts: $RUN_TS"
    echo "- WITH_PEDESTRIAN: $WITH_PEDESTRIAN"
    echo "- WITH_RECOVERY: $WITH_RECOVERY"
    echo "- WITH_LOC_FAULT: $WITH_LOC_FAULT"
    echo "- WITH_PROFILE: $WITH_PROFILE"
    echo "- WITH_STRESS: $WITH_STRESS"
    echo "- WITH_RT_PRIORITY: $WITH_RT_PRIORITY"
    echo "- WITH_RETURN: $WITH_RETURN"
    echo "- WITH_STATIC_OBSTACLE: $WITH_STATIC_OBSTACLE"
    echo "- WITH_LIFT: $WITH_LIFT"
    echo "- NAV_GOAL_RETRIES: $NAV_GOAL_RETRIES"
    echo
    echo "## nav goals"
    echo
    echo "| goal | result |"
    echo "|------|--------|"
    for f in "$OUT"/nav2_goal_*.log; do
      [[ -e "$f" ]] || continue
      [[ "$f" == *.fail.log || "$f" == *.responselost.log ]] && continue   # attempt 사본은 표에서 제외(원본 파일로 판정)
      local gname res
      gname="$(basename "$f" .log | sed 's/^nav2_goal_//')"
      if grep -q "status: SUCCEEDED" "$f"; then res="SUCCEEDED"
      elif grep -q "SERVER-EVIDENCE: bt_navigator 'Goal succeeded'" "$f"; then res="SUCCEEDED (server evidence, client response lost)"
      else res="NOT_SUCCEEDED"; fi
      # 재시도 사본 수 = 실패한 attempt 수 (0 이면 무결점 1차 통과)
      local nfail
      nfail=$(ls "$OUT"/nav2_goal_"${gname}"_a[0-9]*.fail.log 2>/dev/null | wc -l)
      if (( nfail > 0 )); then res="$res (after $nfail failed attempt(s))"; fi
      echo "| $gname | $res |"
    done
    for f in "$OUT"/nav2_recovery_*.log; do
      [[ -e "$f" ]] || continue
      local gname res
      gname="$(basename "$f" .log | sed 's/^nav2_//')"
      if grep -qE "status: (ABORTED|CANCELED)" "$f"; then res="ABORTED(expected)"; else res="UNEXPECTED"; fi
      echo "| $gname | $res |"
    done
    echo
    echo "## verify_world_swap_state"
    echo
    echo '```text'
    if [[ -e "$OUT/verify_world_swap_state.log" ]]; then cat "$OUT/verify_world_swap_state.log"; fi
    echo '```'
    echo
    echo "## control health (nav2 loop)"
    echo
    echo '```text'
    cat "$OUT/control_metrics.txt"
    echo '```'
    if [[ -e "$OUT/profile/resource_summary.txt" ]]; then
      echo
      echo "## resource profile"
      echo
      echo '```text'
      cat "$OUT/profile/resource_summary.txt"
      echo '```'
    fi
  } > "$summary"
  # 토픽/서비스/액션 스냅샷 보존.
  ros2 topic list >"$OUT/topics_final.txt" 2>&1 || true
  ros2 service list >"$OUT/services_final.txt" 2>&1 || true
  cp -a "$OUT/." "$RUN_DIR/" 2>/dev/null || true
  log "artifacts collected -> $RUN_DIR (summary: $RUN_DIR/scenario_summary.md)"
}

ensure_maps() {
  # fresh clone 대응: pgm 은 gitignore — 없으면 생성기로 만들고 시작한다.
  local pgm="$ROOT/src/slam_pkg/maps/kku_virtual/f2/kku_f2.pgm"
  if [[ -f "$pgm" ]]; then
    return 0
  fi
  if [[ -f "$ROOT/scripts/generate_kku_maps.py" ]]; then
    log "maps missing -> generating worlds+maps"
    python3 "$ROOT/scripts/generate_kku_worlds.py"
    python3 "$ROOT/scripts/generate_kku_maps.py"
  else
    echo "error: $pgm 없음, $ROOT/scripts 도 없음 — host에서 scripts/bootstrap_workspace.sh 를 먼저 실행할 것" >&2
    exit 1
  fi
}

trap cleanup_started EXIT
# 시그널 rc 보존 (findings #5): INT trap 이 없으면 trap 마지막 명령(sleep) rc=0 이
# 스크립트 종료 코드가 돼 Ctrl-C 로 중단된 회차가 repeat 러너에서 PASS 로 집계됐다.
trap 'exit 130' INT
trap 'exit 143' TERM

# 반복 러너와의 아티팩트 귀속 계약 (findings #17): 이번 런의 산출 디렉터리를
# stdout 에 명시한다 — 러너는 ls -dt 추측 대신 이 줄을 파싱한다.
log "RUN_DIR=$RUN_DIR"

cd "$ROOT"
source_setup /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export DISPLAY="${DISPLAY:-:1}"

cleanup_stale
# ros2 daemon 이 오염된 상태(!rclpy.ok() XML-RPC fault)면 wait_for_topic 의
# 'ros2 topic list' 가 전부 죽어 /clock 대기에서 멈춘다(2026-08-17 실측).
# 반복 런 강건성을 위해 매 런 daemon 을 리셋한다(다음 CLI 호출이 새로 띄움).
ros2 daemon stop >/dev/null 2>&1 || true
ensure_maps

# SKIP_BUILD=1 이면 4단 colcon 빌드를 건너뛴다 (반복 러너가 2회차부터 전달 — 코드 불변이라
# 검증력 손실 없이 회차당 1~2분 절감, 2026-08-20 리뷰 최적화 (a)-1). 기본 0 = 항상 빌드.
SKIP_BUILD="${SKIP_BUILD:-0}"
if [[ "$SKIP_BUILD" == "1" ]]; then
  log "SKIP_BUILD=1 — colcon 빌드 생략 (이전 회차 install 재사용)"
else
log "building root workspace"
# --base-paths src: 루트 빌드가 /ros2_ws 재귀 탐색으로 test_workspace 패키지까지
# 루트 install에 흡수하던 문제 차단 (패키지 삭제 시 stale ament index로
# GazeboRosPaths 전체 순회가 죽는 사고의 근본 원인)
colcon build --symlink-install --base-paths src >"$OUT/build_root.log" 2>&1

log "building elevator_auto_map_switch"
(
  cd "$ROOT/test_workspace/elevator_auto_map_switch"
  source_setup /opt/ros/humble/setup.bash
  source_setup "$ROOT/install/setup.bash"
  colcon build --symlink-install
) >"$OUT/build_auto_map_switch.log" 2>&1

log "building elevator_mission"
(
  cd "$ROOT/test_workspace/elevator_mission"
  source_setup /opt/ros/humble/setup.bash
  source_setup "$ROOT/install/setup.bash"
  colcon build --symlink-install
) >"$OUT/build_elevator_mission.log" 2>&1

log "building gazebo_world_swap"
(
  cd "$WORKSPACE"
  source_setup /opt/ros/humble/setup.bash
  source_setup "$ROOT/install/setup.bash"
  colcon build --symlink-install
) >"$OUT/build_gazebo_world_swap.log" 2>&1
fi

source_setup "$ROOT/install/setup.bash"
source_setup "$ROOT/test_workspace/elevator_auto_map_switch/install/setup.bash"
source_setup "$ROOT/test_workspace/elevator_mission/install/setup.bash"
source_setup "$WORKSPACE/install/setup.bash"

if [[ "$GAZEBO_REMOTE" == "1" ]]; then
  log "GAZEBO_REMOTE=1 — 원격 Gazebo 사용 (spawn_point:=charge_station 는 원격 호스트에서 띄울 것)"
else
  start_bg gazebo_f1 ros2 launch common_pkg gazebo.launch.py floor:=F1 spawn_point:=charge_station use_sim_time:=true gui:=$GAZEBO_GUI
fi
wait_for_topic /clock 30
wait_for_service /spawn_entity 30
wait_for_service /delete_entity 30
wait_for_service /get_model_list 30

start_bg nav2_f1 ros2 launch slam_pkg kku_navigation.launch.py floor:=F1 rviz:=$WITH_RVIZ use_sim_time:=true
wait_for_service /map_server/load_map 90
wait_for_action /navigate_to_pose 90
wait_for_topic /amcl_pose 90

if [[ "$WITH_RT_PRIORITY" == "1" ]]; then
  apply_rt_priority
fi

publish_initial_pose charge_station 1.6 0.0 0.0 1.0

# bt_navigator 는 initialpose -> AMCL map->odom TF -> costmap activate 이후에야
# active 가 된다. 그 전에 goal 을 보내면 "Goal was rejected" (시작 레이스, 2026-07-03).
# 반드시 initialpose 발행 뒤에 대기할 것 — 앞에 두면 데드락.
wait_for_lifecycle_active /bt_navigator 90

if [[ "$WITH_PEDESTRIAN" == "1" ]]; then
  start_pedestrian
fi

if [[ "$WITH_STATIC_OBSTACLE" == "1" ]]; then
  spawn_static_obstacles
fi

if [[ "$WITH_PROFILE" == "1" ]]; then
  start_profiler
fi

if (( WITH_STRESS > 0 )); then
  start_stress "$WITH_STRESS"
fi

log "running F1 pickup route from charge station"
# 복도 확장(CORRIDOR_HALF 2.5, 5m)으로 택배 door가 y=-2.5, alcove 가 -2.5~-4.3 로 이동.
send_nav_goal f1_parcel_corridor 5.0 0.0 0.0 1.0 150
send_nav_goal f1_parcel_storage 5.0 -2.1 -0.7071068 0.7071068 150
send_nav_goal f1_parcel_pickup 5.0 -3.4 -0.7071068 0.7071068 150

# 도킹 재정위: 좁은 alcove에서 재시도/회복기동을 거치면 AMCL belief가 틀어질 수 있고,
# 그 상태로 출구를 향하면 스캔 벽이 문 위에 마킹돼 자기강화 봉쇄가 된다(§1.25 run_08,
# 376s collision-ahead). 픽업 지점은 좌표가 알려진 도킹 지점이므로 실기와 동일하게
# 여기서 initialpose 를 재발행해 belief 를 재정박한다.
publish_initial_pose parcel_dock 5.0 -3.4 -0.7071068 0.7071068

# 택배함 픽업 리프트 동작 (최종 시나리오: 대기 -> 택배함(리프트) -> 로비 -> ...).
if [[ "$WITH_LIFT" == "1" ]]; then
  run_lift_cycle pickup
fi

log "returning to F1 elevator for floor transfer"
send_nav_goal f1_parcel_exit 5.0 -2.1 0.7071068 0.7071068 150

# 택배 픽업 후 출고 -> 앞쪽으로 살짝 튀어나온 footprint 적용(택배 들고 가는 형상).
# 2026-08-20 적대 리뷰 C 반영: 구 폴리곤(반폭 0.26, 후방 -0.18)은 내접반경 0.18 <
# 실반폭 0.2691 이라 Smac2D 가 벽에서 8.9cm 물리 불가능 경로를 허용했다(엘베 진입
# patience 초과의 유력 기전). 반폭 0.27(실폭 0.5382m)·후방 -0.27 로 내접 0.27 확보.
# 노즈는 0.40 복원 — 구 0.35 축소의 명분(포켓 inflation 간섭)은 두 폴리곤의 내접
# 반경이 동일(코스트필드 동일)해 물리적으로 불성립(리뷰 findings #9). 택배 크기는
# hardware_spec 미정 항목 — 실측 확정 시 갱신.
PARCEL_FOOTPRINT="[[0.40,0.12],[0.40,-0.12],[0.27,-0.27],[-0.27,-0.27],[-0.27,0.27],[0.27,0.27]]"
# 원복용 몸통 polygon (전방 확장 제거). robot_radius 0.28 원 대신 실제 몸통 사각형 —
# "[]" 로 radius 복귀를 시도하면 type error 로 무음 실패한다(2026-07-03 엘베 갇힘).
NORMAL_FOOTPRINT="[[0.27,0.27],[0.27,-0.27],[-0.27,-0.27],[-0.27,0.27]]"
log "parcel loaded -> extend front footprint (carrying parcel)"
set_costmap_footprint parcel "$PARCEL_FOOTPRINT"
send_nav_goal f1_corridor_return 5.0 0.0 1.0 0.0 150
send_nav_goal f1_elevator_entry 1.6 0.0 1.0 0.0 150
send_nav_goal f1_elevator_inside 0.0 0.0 0.0 1.0 150

if [[ "$WITH_ARM" == "1" ]]; then
  # 층 전환 신호를 놓치지 않도록 orchestrator 보다 먼저 구독을 시작한다.
  # 주의: serial_port 는 값이 있을 때만 넘긴다 — 빈 값 '-p serial_port:=' 는
  # rcl 파라미터 파싱 에러로 노드가 즉사한다(2026-08-17 실측, mock 경로 최초 노출).
  ARM_ARGS=(--ros-args -p use_sim_time:=true)
  if [[ -n "$ARM_SERIAL_PORT" ]]; then
    ARM_ARGS+=(-p "serial_port:=$ARM_SERIAL_PORT")
  fi
  start_bg arm_sequence ros2 run robot_arm_pkg arm_sequence "${ARM_ARGS[@]}"
fi

start_bg orchestrator ros2 launch auto_floor_orchestrator_pkg auto_floor_orchestrator.launch.py dry_run_map_load:=false target_floor:=F2 use_sim_time:=true
wait_for_service /floor_orchestrator/request_switch 30
start_bg world_swap ros2 launch gazebo_world_swap_pkg world_swap.launch.py method:=model_swap initial_floor:=F1 use_sim_time:=true

log "arming floor switch"
# timeout 없이는 CLI가 discovery 실패 시 무한 대기 (fastdds unicast peers 함정 — improvement_report §1.23)
request_floor_switch F2 || exit 1

log "publishing F2 ARRIVED_OPEN elevator state"
ros2 topic pub --times 10 --rate 2 /elevator/state std_msgs/msg/String \
  "{data: '{\"current_floor\":\"F2\",\"target_floor\":\"F2\",\"door_state\":\"open\",\"state\":\"ARRIVED_OPEN\"}'}" \
  >"$OUT/elevator_state_pub.log" 2>&1

log "verifying map, world entity, status, and scan"
if ! python3 "$WORKSPACE/scripts/verify_world_swap_state.py" --timeout-sec 90 \
  >"$OUT/verify_world_swap_state.log" 2>&1; then
  cat "$OUT/verify_world_swap_state.log"
  exit 1
fi

cat "$OUT/verify_world_swap_state.log"

if [[ "$WITH_ARM" == "1" ]]; then
  # 시퀀스 13s(sim time) + Jetson RTF 저하 여유 -> 90s wall clock.
  verify_arm_sequence F2 90 1 || exit 1
fi

# F2 도착 후 택배 하역 완료로 간주 -> footprint 몸통 원복.
# 확장 footprint 유지 시 엘리베이터(문 1.0m)에서 빠져나오다 끼는 문제 방지.
log "parcel delivered -> reset footprint to normal for F2"
set_costmap_footprint reset "$NORMAL_FOOTPRINT"
align_robot_to_spawn F2

if [[ "$WITH_LOC_FAULT" == "1" ]]; then
  # F2 진입점 실제 ~(0,0). 0.5m 어긋난 initialpose 주입 -> AMCL/scan 매칭 보정 후에도
  # 아래 f2_corridor goal 이 SUCCEEDED 해야 한다(한정 오차 robustness 검증).
  inject_wrong_initialpose f2_entry 0.5 0.0
fi

# 엘베 출구 스테이징 (2026-08-21 G004 run2): 전환 직후 (0,0)에서 곧바로 (2.5,12) 로 향하면
# 경로가 문 북측 jamb 쪽으로 꺾여 RPP 가 코너를 깎다 footprint 가장자리가 jamb inflation(내접
# 0.27) 에 걸려 'collision ahead' 로 3회 전부 ABORT(true y=+0.11 에서 정지). 문을 직진으로
# 빠져나오는 중간 goal 을 두어 정렬된 상태로 통과한 뒤 북쪽으로 돈다(실기 미션도 동일 패턴).
log "staging straight out of the F2 elevator door"
send_nav_goal f2_elevator_exit 1.8 0.0 0.0 1.0 150

log "sending F2 corridor navigation goal near room 208"
send_nav_goal f2_corridor 2.5 12.0 0.0 1.0 150

# F2 배달 지점 도착 -> 택배 하차 리프트 동작 (최종 시나리오: F2 도착 -> 하차(리프트)).
if [[ "$WITH_LIFT" == "1" ]]; then
  run_lift_cycle f2_delivery
fi

if [[ "$WITH_RECOVERY" == "1" ]]; then
  # 도달 불가능 goal(맵 밖) -> Nav2 가 한정된 시간 안에 ABORTED 로 안전 종료해야 한다.
  expect_nav_abort unreachable 100.0 100.0 120
fi

if [[ "$WITH_F3" == "1" ]]; then
  # 두 번째 층 전환: F2 -> F3 (F3 world/맵은 현재 F2 레이아웃과 동일 — 실측 후 교체 예정).
  log "returning to F2 elevator for F3 transfer"
  send_nav_goal f2_elevator_inside 0.0 0.0 0.0 1.0 150

  log "arming F3 floor switch (target_floor param -> F3)"
  set_orchestrator_target_floor F3 || exit 1
  request_floor_switch F3 || exit 1

  log "publishing F3 ARRIVED_OPEN elevator state"
  ros2 topic pub --times 10 --rate 2 /elevator/state std_msgs/msg/String \
    "{data: '{\"current_floor\":\"F3\",\"target_floor\":\"F3\",\"door_state\":\"open\",\"state\":\"ARRIVED_OPEN\"}'}" \
    >"$OUT/elevator_state_pub_f3.log" 2>&1

  log "verifying F2 -> F3 map, world entity, status, and scan"
  if ! python3 "$WORKSPACE/scripts/verify_world_swap_state.py" --timeout-sec 90 \
    --floor F3 --from-floor F2 >"$OUT/verify_world_swap_state_f3.log" 2>&1; then
    cat "$OUT/verify_world_swap_state_f3.log"
    exit 1
  fi
  cat "$OUT/verify_world_swap_state_f3.log"

  if [[ "$WITH_ARM" == "1" ]]; then
    verify_arm_sequence F3 90 2 || exit 1
  fi

  align_robot_to_spawn F3
  log "sending F3 corridor navigation goal"
  send_nav_goal f3_corridor 2.5 12.0 0.0 1.0 150
fi

if [[ "$WITH_RETURN" == "1" ]]; then
  # 왕복 복귀: F2 배달 완료 -> 엘베 -> F2->F1 역전환 -> 로비 -> 충전소 대기.
  # 역방향 전환은 orchestrator/world-swap 이 방향 무관이라 기존 경로 그대로 재사용
  # (swap_trigger: current_floor != last_floor 이면 스왑, floor_maps.yaml F1 시딩 존재).
  log "returning to F2 elevator for F1 transfer (roundtrip)"
  send_nav_goal f2_elevator_inside 0.0 0.0 0.0 1.0 150

  log "arming F1 floor switch (target_floor param -> F1)"
  set_orchestrator_target_floor F1 || exit 1
  request_floor_switch F1 || exit 1

  log "publishing F1 ARRIVED_OPEN elevator state"
  ros2 topic pub --times 10 --rate 2 /elevator/state std_msgs/msg/String \
    "{data: '{\"current_floor\":\"F1\",\"target_floor\":\"F1\",\"door_state\":\"open\",\"state\":\"ARRIVED_OPEN\"}'}" \
    >"$OUT/elevator_state_pub_f1.log" 2>&1

  log "verifying F2 -> F1 map, world entity, status, and scan"
  if ! python3 "$WORKSPACE/scripts/verify_world_swap_state.py" --timeout-sec 90 \
    --floor F1 --from-floor F2 >"$OUT/verify_world_swap_state_f1.log" 2>&1; then
    cat "$OUT/verify_world_swap_state_f1.log"
    exit 1
  fi
  cat "$OUT/verify_world_swap_state_f1.log"

  if [[ "$WITH_ARM" == "1" ]]; then
    # 왕복 팔 완료 누계: F2 전환 1회 + F1 복귀 1회 = 2회.
    verify_arm_sequence F1 90 2 || exit 1
  fi

  align_robot_to_spawn F1
  log "sending F1 charge station return goal (roundtrip complete)"
  send_nav_goal f1_charge_station 1.6 0.0 0.0 1.0 150
fi

# P0 제어 충실도 회귀 가드: MAX_MISSED_RATE 설정 시 임계 초과면 FAIL.
# 2026-08-20 리뷰 findings #12: (a) finalize 이전에 판정해 FAIL 런의 아티팩트가
# 성공처럼 남지 않게 한다 (b) 지표가 비거나 숫자가 아니면 fail-closed —
# 구버전은 빈 값이 산술 구문오류를 타고 조용히 '통과'였다.
gate_rc=0
write_control_metrics
if [[ -n "$MAX_MISSED_RATE" ]]; then
  missed=$(grep '^control_loop_missed_rate=' "$OUT/control_metrics.txt" 2>/dev/null | cut -d= -f2 || true)
  if [[ ! "$missed" =~ ^[0-9]+$ ]]; then
    log "CONTROL FIDELITY FAIL: missed-rate metric unavailable ('$missed') — fail-closed"
    gate_rc=1
  elif (( missed > MAX_MISSED_RATE )); then
    log "CONTROL FIDELITY FAIL: nav2 control loop missed rate $missed > $MAX_MISSED_RATE"
    gate_rc=1
  else
    log "control fidelity: missed-rate=$missed <= MAX_MISSED_RATE=$MAX_MISSED_RATE — OK"
  fi
fi

finalize_artifacts
if (( gate_rc != 0 )); then
  exit "$gate_rc"
fi

log "logs written to $OUT"
