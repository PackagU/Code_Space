#!/usr/bin/env bash
# 왕복 스모크 반복 러너 — 연속 N회 성공 판정 (Roadmap 06 반복 안정성).
# 각 회차는 독립 smoke 전체 실행(빌드/기동/cleanup 포함) — 회차 간 상태 오염 없음
# (매 런 Gazebo 재기동 = "매 런 전 리셋" 운영 규칙 자동 충족).
# 사용(컨테이너): REPEAT_N=10 bash test_workspace/gazebo_world_swap/scripts/run_roundtrip_repeat.sh
set -euo pipefail
trap 'exit 130' INT
trap 'exit 143' TERM

ROOT="${ROOT:-/ros2_ws}"
WORKSPACE="$ROOT/test_workspace/gazebo_world_swap"
REPEAT_N="${REPEAT_N:-10}"
REPEAT_TS="$(date +%Y%m%d_%H%M%S)"
REPEAT_DIR="$WORKSPACE/verification/repeat_$REPEAT_TS"
SUMMARY="$REPEAT_DIR/repeat_summary.md"
mkdir -p "$REPEAT_DIR"

# 반복 회차 공통 옵션(왕복+동/정적 장애물+팔+리프트+프로파일). env 로 덮어쓰기 가능.
export WITH_RETURN="${WITH_RETURN:-1}"
export WITH_PEDESTRIAN="${WITH_PEDESTRIAN:-1}"
export WITH_STATIC_OBSTACLE="${WITH_STATIC_OBSTACLE:-1}"
export WITH_PROFILE="${WITH_PROFILE:-1}"
export WITH_ARM="${WITH_ARM:-1}"
export WITH_LIFT="${WITH_LIFT:-1}"
# 잠정 제어 충실도 게이트 (2026-08-20): 관측 missed 2~6 대비 5배 여유 — 폭주 회귀
# (예: 서비스 콜 스톰)만 차단하는 안전망. 실기 임계는 실센서 주행에서 별도 수립.
export MAX_MISSED_RATE="${MAX_MISSED_RATE:-30}"
# KEEP_RUNNING 이 환경에 남아 있으면 10회가 전부 프로세스를 살려둬 메모리 폭주가
# 된다(리뷰 findings M11) — 반복 러너에서는 강제 차단.
export KEEP_RUNNING=0

log() { printf '[roundtrip-repeat] %s\n' "$*"; }

oom_count() {
  # cgroup OOM kill 카운터 (컨테이너에서 dmesg 보다 신뢰 가능 — 시간 무관 링버퍼
  # 오탐과 CAP_SYSLOG 부재 무력화 문제 회피, 리뷰 findings H9).
  # v2: /sys/fs/cgroup/memory.events "oom_kill N" (데스크톱) /
  # v1: /sys/fs/cgroup/memory/memory.oom_control "oom_kill N" (Jetson JetPack 실측 2026-08-20).
  local f
  for f in /sys/fs/cgroup/memory.events /sys/fs/cgroup/memory/memory.oom_control; do
    if [[ -r "$f" ]]; then
      awk '$1=="oom_kill"{print $2; found=1} END{if(!found) print 0}' "$f" 2>/dev/null && return 0
    fi
  done
  echo 0
}

OOM_BASE="$(oom_count)"
if ! [[ -r /sys/fs/cgroup/memory.events || -r /sys/fs/cgroup/memory/memory.oom_control ]]; then
  log "WARN: cgroup OOM 카운터(v2 memory.events / v1 memory.oom_control) 읽기 불가 — OOM 가드 무력 (지표만 n/a)"
fi

{
  echo "# roundtrip repeat summary"
  echo
  echo "- started: $REPEAT_TS"
  echo "- target: $REPEAT_N consecutive PASS (WITH_RETURN=$WITH_RETURN WITH_PEDESTRIAN=$WITH_PEDESTRIAN WITH_STATIC_OBSTACLE=$WITH_STATIC_OBSTACLE WITH_ARM=$WITH_ARM WITH_LIFT=$WITH_LIFT MAX_MISSED_RATE=$MAX_MISSED_RATE)"
  echo
  echo "| run | result | duration_s | cpu_pct_peak | mem_used_peak_mb | missed_rate | retries | realign | resp_lost |"
  echo "|-----|--------|------------|--------------|------------------|-------------|---------|---------|-----------|"
} > "$SUMMARY"

pass=0
for ((i = 1; i <= REPEAT_N; i++)); do
  slot="run_$(printf '%02d' "$i")"
  log "$slot/$REPEAT_N starting"
  start=$SECONDS
  rc=0
  # 2회차부터 빌드 생략 (코드 불변, 회차당 1~2분 절감 — 2026-08-20 리뷰 최적화 (a)-1).
  # 1회차가 빌드에 실패하면 rc!=0 으로 즉시 중단되므로 stale install 위에서 도는 일은 없다.
  if (( i == 1 )); then export SKIP_BUILD=0; else export SKIP_BUILD=1; fi
  bash "$WORKSPACE/scripts/run_l3_world_swap_smoke.sh" >"$REPEAT_DIR/$slot.log" 2>&1 || rc=$?
  dur=$((SECONDS - start))
  # 아티팩트 귀속: 스모크가 stdout 에 명시한 RUN_DIR 만 신뢰 (구 ls -dt 최신 추정은
  # 병렬/잔존 디렉터리 오귀속 — 리뷰 findings #17).
  run_dir="$(grep -m1 -oP '^\[world-swap-smoke\] RUN_DIR=\K.*' "$REPEAT_DIR/$slot.log" || true)"
  if [[ -n "$run_dir" && -d "$run_dir" ]]; then
    mv "$run_dir" "$REPEAT_DIR/$slot"
  else
    log "WARN: $slot 의 RUN_DIR 미확인 — 아티팩트 미이동 (smoke 조기 사망?)"
  fi
  cpu_peak="$(grep -oP 'cpu_pct_peak: *\K[0-9.]+' "$REPEAT_DIR/$slot/profile/resource_summary.txt" 2>/dev/null || true)"
  mem_peak="$(grep -oP 'mem_used_peak_mb: *\K[0-9.]+' "$REPEAT_DIR/$slot/profile/resource_summary.txt" 2>/dev/null || true)"
  missed="$(grep -oP 'control_loop_missed_rate=\K[0-9]+' "$REPEAT_DIR/$slot/control_metrics.txt" 2>/dev/null || true)"
  # 회복 개입 계측 — "무결점 1차 통과"와 "회복으로 살아난 런"을 표에서 구분
  # (리뷰 A: 이 구분 없이는 회복 스택이 체계 결함을 은폐한다).
  retries="$(grep -c 'clearing costmaps + retrying' "$REPEAT_DIR/$slot.log" 2>/dev/null || true)"
  realign="$(grep -c 'REALIGN: belief drift' "$REPEAT_DIR/$slot.log" 2>/dev/null || true)"
  # 응답 유실 흡수 횟수 (서버 증거로 성공 판정한 goal 수 — rmw 응답 유실 빈도 추적, §1.23e)
  lost="$(grep -c 'client response lost -> accepting' "$REPEAT_DIR/$slot.log" 2>/dev/null || true)"
  if (( rc == 0 )); then result=PASS; pass=$((pass+1)); else result="FAIL(rc=$rc)"; fi
  echo "| $i | $result | $dur | ${cpu_peak:-n/a} | ${mem_peak:-n/a} | ${missed:-n/a} | ${retries:-0} | ${realign:-0} | ${lost:-0} |" >> "$SUMMARY"
  oom_now="$(oom_count)"
  if (( oom_now > OOM_BASE )); then
    { echo; echo "OOM kill detected at $slot (cgroup oom_kill $OOM_BASE -> $oom_now)"; } >> "$SUMMARY"
    log "OOM KILL DETECTED at $slot (cgroup counter $OOM_BASE -> $oom_now) — failing"
    echo "RESULT: FAIL(oom) at $slot — $pass/$REPEAT_N" >> "$SUMMARY"
    exit 1
  fi
  if (( rc != 0 )); then
    { echo; echo "consecutive PASS broken at $slot (rc=$rc) — see $slot.log + $slot/"; } >> "$SUMMARY"
    { echo; echo "RESULT: FAIL at $slot — $pass/$REPEAT_N consecutive PASS"; } >> "$SUMMARY"
    log "$slot FAILED (rc=$rc) — stopping (연속 성공 조건 위반)"
    exit 1
  fi
  log "$slot PASS (${dur}s, cpu ${cpu_peak:-n/a}%, mem ${mem_peak:-n/a}MB, missed ${missed:-n/a}, retries ${retries:-0}, realign ${realign:-0})"
done

{ echo; echo "RESULT: $pass/$REPEAT_N consecutive PASS"; } >> "$SUMMARY"
log "ALL $REPEAT_N RUNS PASSED — summary: $SUMMARY"
