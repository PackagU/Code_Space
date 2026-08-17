#!/usr/bin/env bash
# 왕복 스모크 반복 러너 — 연속 N회 성공 판정 (Roadmap 06 반복 안정성).
# 각 회차는 독립 smoke 전체 실행(빌드/기동/cleanup 포함) — 회차 간 상태 오염 없음
# (매 런 Gazebo 재기동 = "매 런 전 리셋" 운영 규칙 자동 충족).
# 사용(컨테이너): REPEAT_N=10 bash test_workspace/gazebo_world_swap/scripts/run_roundtrip_repeat.sh
set -euo pipefail

ROOT="${ROOT:-/ros2_ws}"
WORKSPACE="$ROOT/test_workspace/gazebo_world_swap"
REPEAT_N="${REPEAT_N:-10}"
REPEAT_TS="$(date +%Y%m%d_%H%M%S)"
REPEAT_DIR="$WORKSPACE/verification/repeat_$REPEAT_TS"
SUMMARY="$REPEAT_DIR/repeat_summary.md"
mkdir -p "$REPEAT_DIR"

# 반복 회차 공통 옵션(왕복+보행자+팔 mock+프로파일). env 로 덮어쓰기 가능.
export WITH_RETURN="${WITH_RETURN:-1}"
export WITH_PEDESTRIAN="${WITH_PEDESTRIAN:-1}"
export WITH_PROFILE="${WITH_PROFILE:-1}"
export WITH_ARM="${WITH_ARM:-1}"

log() { printf '[roundtrip-repeat] %s\n' "$*"; }

oom_trace() {
  # OOM killer 흔적 검사 (컨테이너에서 dmesg 접근 불가 환경이면 빈 출력 = 통과).
  dmesg 2>/dev/null | tail -300 | grep -iE "out of memory|oom-kill" || true
}

{
  echo "# roundtrip repeat summary"
  echo
  echo "- started: $REPEAT_TS"
  echo "- target: $REPEAT_N consecutive PASS (WITH_RETURN=$WITH_RETURN WITH_PEDESTRIAN=$WITH_PEDESTRIAN WITH_ARM=$WITH_ARM)"
  echo
  echo "| run | result | duration_s | cpu_pct_peak | mem_used_peak_mb | missed_rate |"
  echo "|-----|--------|------------|--------------|------------------|-------------|"
} > "$SUMMARY"

pass=0
for ((i = 1; i <= REPEAT_N; i++)); do
  slot="run_$(printf '%02d' "$i")"
  log "$slot/$REPEAT_N starting"
  start=$SECONDS
  rc=0
  bash "$WORKSPACE/scripts/run_l3_world_swap_smoke.sh" >"$REPEAT_DIR/$slot.log" 2>&1 || rc=$?
  dur=$((SECONDS - start))
  latest_run="$(ls -dt "$WORKSPACE"/verification/run_2* 2>/dev/null | head -1)"
  [[ -n "$latest_run" ]] && mv "$latest_run" "$REPEAT_DIR/$slot"
  cpu_peak="$(grep -oP 'cpu_pct_peak: *\K[0-9.]+' "$REPEAT_DIR/$slot/profile/resource_summary.txt" 2>/dev/null || true)"
  mem_peak="$(grep -oP 'mem_used_peak_mb: *\K[0-9.]+' "$REPEAT_DIR/$slot/profile/resource_summary.txt" 2>/dev/null || true)"
  missed="$(grep -oP 'control_loop_missed_rate=\K[0-9]+' "$REPEAT_DIR/$slot/control_metrics.txt" 2>/dev/null || true)"
  if (( rc == 0 )); then result=PASS; pass=$((pass+1)); else result="FAIL(rc=$rc)"; fi
  echo "| $i | $result | $dur | ${cpu_peak:-n/a} | ${mem_peak:-n/a} | ${missed:-n/a} |" >> "$SUMMARY"
  oom="$(oom_trace)"
  if [[ -n "$oom" ]]; then
    { echo; echo "## OOM evidence ($slot)"; echo '```text'; echo "$oom"; echo '```'; } >> "$SUMMARY"
    log "OOM KILLER TRACE DETECTED at $slot — failing"
    exit 1
  fi
  if (( rc != 0 )); then
    { echo; echo "consecutive PASS broken at $slot (rc=$rc) — see $slot.log"; } >> "$SUMMARY"
    log "$slot FAILED (rc=$rc) — stopping (연속 성공 조건 위반)"
    exit 1
  fi
  log "$slot PASS (${dur}s, cpu ${cpu_peak:-n/a}%, mem ${mem_peak:-n/a}MB, missed ${missed:-n/a})"
done

{ echo; echo "RESULT: $pass/$REPEAT_N consecutive PASS"; } >> "$SUMMARY"
log "ALL $REPEAT_N RUNS PASSED — summary: $SUMMARY"
