# 왕복 배달 시뮬 완성 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** F1↔F2 왕복 배달 체인(충전소→택배함→엘베→F2 배달→엘베→F1 복귀)을 시뮬에서 완성하고, 반복 10회 + 분산 3회 + docker HW-ready까지 검증한다.

**Architecture:** 기존 `run_l3_world_swap_smoke.sh`에 `WITH_RETURN` 게이트를 추가해 역방향(F2→F1) 전환을 기존 orchestrator/world-swap 경로 그대로 재사용한다(코드 사전조사: 역방향은 이미 지원됨). 반복 안정성은 스모크 전체를 회차 단위로 재실행하는 러너로 검증한다.

**Tech Stack:** Bash smoke script, ROS2 Humble (Nav2/AMCL/Gazebo Classic), Python contract tests, Docker compose.

**Spec:** `docs/superpowers/specs/2026-08-17-roundtrip-delivery-sim-design.md`

## Global Constraints

- `WITH_*` 옵션 기본값은 전부 0 유지 (결정적 스모크 보존, contract 테스트가 가드)
- `CORRIDOR_HALF = 2.5` 불변, F2 맵 498x348 불변
- 커밋 전 `python3 scripts/check_portability.py` PASS 필수
- 브랜치: `lee/hw-design-review`
- 실물 라이다·로봇팔·HDMI는 제거된 상태 — Jetson에서 `/dev/rplidar`, `/dev/arm_servo` 부재 전제로 작업 (ARM_SERIAL_PORT 빈 값 = mock)
- 컨테이너 실행 명령은 `docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && ..."` (비대화형 exec은 ROS 미소싱)
- 두 머신 컨테이너 이름 동일(`ros2_humble`) — Jetson 명령은 반드시 `ssh hsm` 경유로 실행

---

### Task 1: WITH_RETURN 왕복 게이트

**Files:**
- Modify: `test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py`
- Modify: `test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh`

**Interfaces:**
- Produces: `WITH_RETURN=1` env 게이트, `set_orchestrator_target_floor <floor>` 헬퍼 (timeout 30s ×3회 재시도, 실패 시 return 1)

- [ ] **Step 1: contract 테스트에 왕복 가드 추가 (failing test)**

`test_smoke_scripts_contract.py`의 옵션 루프 튜플에 `"WITH_RETURN"` 추가, 그리고 F3 가드 블록 뒤에 추가:

```python
    # --- 회귀 보호: 왕복(F1<->F2) 체인 ---
    assert "WITH_RETURN" in runner_text, "roundtrip gate missing"
    assert "--floor F1 --from-floor F2" in runner_text, "F2->F1 reverse verification missing"
    assert "send_nav_goal f1_charge_station 1.6 0.0" in runner_text, "charge station return goal missing"
    assert "verify_arm_sequence F1" in runner_text, "arm sequence F1 verification missing"
    assert "WITH_RETURN=1 과 WITH_F3=1" in runner_text, "WITH_RETURN/WITH_F3 mutual exclusion missing"
    assert "set_orchestrator_target_floor F1" in runner_text, "F1 target_floor helper call missing"
    assert "set_orchestrator_target_floor F3" in runner_text, (
        "F3 target_floor must use timeout helper (bare 'ros2 param set' half-hang, §1.23e)"
    )
```

- [ ] **Step 2: 실패 확인** — `python3 test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py` → AssertionError "roundtrip gate missing"

- [ ] **Step 3: 스모크 스크립트 수정** — 4개 편집:

(a) 옵션 블록(WITH_ARM 아래)에 추가:

```bash
# 왕복 배달 옵션. WITH_RETURN=1 이면 F2 배달(f2_corridor) 후 엘베로 복귀해
# F2->F1 역전환 -> F1 충전소 복귀까지 수행한다(한 층 왕복 완성 체인).
# 기본 0 -> 기존 결정적 smoke 유지. WITH_F3 와 동시 사용 금지(왕복은 F1<->F2 전용).
WITH_RETURN="${WITH_RETURN:-0}"
if [[ "$WITH_RETURN" == "1" && "$WITH_F3" == "1" ]]; then
  echo "error: WITH_RETURN=1 과 WITH_F3=1 은 동시 사용 불가 (왕복은 F1<->F2 전용)" >&2
  exit 1
fi
```

(b) `set_costmap_footprint` 아래 헬퍼 추가:

```bash
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
```

(c) WITH_F3 블록의 `ros2 param set /floor_orchestrator_node target_floor F3 >"$OUT/param_target_f3.log" 2>&1` 줄을 `set_orchestrator_target_floor F3 || exit 1`로 교체 (timeout 없는 기존 결함 수정).

(d) WITH_F3 블록 종료 뒤·`finalize_artifacts` 앞에 왕복 블록 추가:

```bash
if [[ "$WITH_RETURN" == "1" ]]; then
  # 왕복 복귀: F2 배달 완료 -> 엘베 -> F2->F1 역전환 -> 로비 -> 충전소 대기.
  # 역방향 전환은 orchestrator/world-swap 이 방향 무관이라 기존 경로 그대로 재사용
  # (swap_trigger: current_floor != last_floor 이면 스왑, floor_maps.yaml F1 시딩 존재).
  log "returning to F2 elevator for F1 transfer (roundtrip)"
  send_nav_goal f2_elevator_inside 0.0 0.0 0.0 1.0 150

  log "arming F1 floor switch (target_floor param -> F1)"
  set_orchestrator_target_floor F1 || exit 1
  if ! timeout 45 ros2 service call /floor_orchestrator/request_switch std_srvs/srv/Trigger >"$OUT/request_switch_f1.log" 2>&1; then
    log "F1 request_switch call failed/timed out"
    cat "$OUT/request_switch_f1.log"
    exit 1
  fi

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
```

(e) `finalize_artifacts`의 옵션 echo 목록에 `echo "- WITH_RETURN: $WITH_RETURN"` 추가.

- [ ] **Step 4: 통과 확인** — contract 테스트 PASS + `bash -n run_l3_world_swap_smoke.sh` + `bash scripts/run_offline_tests.sh` 전부 PASS
- [ ] **Step 5: Commit** — `feat(smoke): WITH_RETURN 왕복 게이트 — F2→F1 역전환+충전소 복귀 (param set timeout 결함 동시 수정)`

### Task 2: 반복 러너

**Files:**
- Create: `test_workspace/gazebo_world_swap/scripts/run_roundtrip_repeat.sh` (실행권한 +x)
- Modify: `test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py`

**Interfaces:**
- Consumes: Task 1의 `WITH_RETURN=1` 게이트
- Produces: `REPEAT_N=10 bash run_roundtrip_repeat.sh` → `verification/repeat_<ts>/repeat_summary.md` + 회차별 `run_NN/` 아티팩트, 실패 시 즉시 exit 1

- [ ] **Step 1: contract 테스트 추가 (failing)**

```python
    repeat_runner = scripts / "run_roundtrip_repeat.sh"
    assert repeat_runner.exists(), "run_roundtrip_repeat.sh missing"
    assert os.access(repeat_runner, os.X_OK), "repeat runner is not executable"
    repeat_text = repeat_runner.read_text(encoding="utf-8")
    assert 'REPEAT_N="${REPEAT_N:-10}"' in repeat_text, "repeat default must be 10"
    assert "oom" in repeat_text.lower(), "OOM guard missing"
    assert "repeat_summary.md" in repeat_text, "aggregate summary missing"
```

- [ ] **Step 2: 실패 확인** — contract 테스트 → "run_roundtrip_repeat.sh missing"
- [ ] **Step 3: 러너 작성**

```bash
#!/usr/bin/env bash
# 왕복 스모크 반복 러너 — 연속 N회 성공 판정 (Roadmap 06 반복 안정성).
# 각 회차는 독립 smoke 전체 실행(빌드/기동/cleanup 포함) — 회차 간 상태 오염 없음.
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
```

- [ ] **Step 4: 통과 확인** — `chmod +x` 후 contract 테스트 PASS + `bash -n` PASS + `run_offline_tests.sh` PASS
- [ ] **Step 5: Commit** — `feat(smoke): 왕복 반복 러너 — 연속 N회(기본 10) + OOM 가드 + 회차 집계`

### Task 3: compose 장치 매핑 환경변수화

**Files:**
- Modify: `docker/compose/docker-compose.jetson.yml`
- Modify: `docker/README.md` (변수 사용법 3줄)

**Interfaces:**
- Produces: `RPLIDAR_DEVICE`/`OPENCR_DEVICE`/`ARM_SERVO_DEVICE`/`MOTOR_NANO_DEVICE` env — 기본 `/dev/null` (HW 미장착에서도 up/recreate 성공), 장착 시 `.env` 또는 env로 실경로 지정

배경: 라이다·팔 USB 제거로 `/dev/rplidar` 등이 사라지면 **기존 정적 매핑은 컨테이너 recreate/start 실패**를 유발한다. 또한 장치 주석 토글이 Jetson pull 충돌 단골이었다(§4.5 주의사항).

- [ ] **Step 1: devices 절 교체**

```yaml
    # 장치 매핑은 env 로 제어 — 기본 /dev/null 이라 HW 미장착에서도 up 이 성공한다.
    # 장착 시 Jetson 의 docker/compose/.env 에 실경로 지정 (예: RPLIDAR_DEVICE=/dev/rplidar).
    # 주석 토글 방식은 pull 충돌 단골이라 폐지 (2026-08-17).
    devices:
      - ${RPLIDAR_DEVICE:-/dev/null}:/dev/rplidar     # RPLiDAR A1m8
      - ${OPENCR_DEVICE:-/dev/null}:/dev/opencr       # OpenCR 1.0
      - ${ARM_SERVO_DEVICE:-/dev/null}:/dev/arm_servo # 로봇팔 서보 (CH340)
      - ${MOTOR_NANO_DEVICE:-/dev/null}:/dev/motor_nano
```

- [ ] **Step 2: 게이트** — `python3 scripts/check_portability.py` PASS + `docker compose -f docker/compose/docker-compose.jetson.yml config` 문법 검증(amd64 데스크톱에서 config만) + `run_offline_tests.sh` PASS
- [ ] **Step 3: Commit** — `fix(docker): jetson compose 장치 매핑 env 화 — HW 미장착 recreate 실패/ pull 충돌 해소`

### Task 4: HW 업데이트 체크리스트 문서

**Files:**
- Create: `docs/deployment/03_hw_update_checklist.md`

내용 (Notion 스타일 규칙 준수 — H1~H3, 코드블록 언어 태그): 구동부 모터 확정 시(URDF 바퀴 반경·질량·dynamics, nav2 속도/가속 한계, OpenCR 브리지 파라미터, diff_drive cmd_vel_timeout §1.22d), 라이다 재장착 시(udev·compose `RPLIDAR_DEVICE`), 로봇팔 재장착 시(`ARM_SERVO_DEVICE`, `ARM_SERIAL_PORT=/dev/arm_servo`), Depth 카메라 확정 시(신규 udev·compose 추가 절차), 각 항목에 파일 경로·검증 명령 표.

- [ ] **Step 1: 문서 작성** (위 내용 표로)
- [ ] **Step 2: Commit** — `docs(deployment): 구동부/센서 HW 확정 시 업데이트 체크리스트`

### Task 5: 데스크톱 단일 왕복 검증 (결정적)

- [ ] **Step 1:** 데스크톱 컨테이너 확인 후 실행:

```bash
docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && WITH_RETURN=1 WITH_ARM=1 WITH_PROFILE=1 bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh"
```

- [ ] **Step 2:** 판정 — exit 0, F1 7 goal + `ARM PASS: F2` + f2_corridor + `PASS world swap`(F1 복귀: kku_f1_building present/kku_f2_building absent) + `ARM PASS: F1`(누계 2) + f1_charge_station SUCCEEDED. 실패 시 superpowers:systematic-debugging으로 원인 격파 후 재실행.
- [ ] **Step 3:** goals.py checkpoint (evidence: run 디렉터리 경로 + 결과 요약)

### Task 6: 보행자 왕복 검증

- [ ] **Step 1:** `WITH_RETURN=1 WITH_ARM=1 WITH_PEDESTRIAN=1 WITH_PROFILE=1` 동일 실행 (NAV_GOAL_RETRIES 기본 2 — 보행자 런 자동)
- [ ] **Step 2:** 판정 동일 + 보행자 5명 스폰 로그 확인. 실패 시 디버깅 후 재실행.

### Task 7: 반복 10회

- [ ] **Step 1:** `REPEAT_N=10 bash .../run_roundtrip_repeat.sh` (컨테이너, nohup + 로그 파일)
- [ ] **Step 2:** 판정 — `RESULT: 10/10 consecutive PASS`, OOM 흔적 없음, mem 피크 추세 증가 없음. 실패 시 원인 격파 → 러너 재시작(연속 카운터 리셋).
- [ ] **Step 3:** goals.py checkpoint + 커밋 (`report:` 형식으로 improvement_report 갱신 포함)

### Task 8: Jetson bundle 재동기화 + 분산 3회

- [ ] **Step 1: bundle 재동기화** (TODO.md 절차):

```bash
cd ~/2026_graduation_project && git bundle create ~/code_space.bundle --all
scp ~/code_space.bundle hsm:/home/hsm/code_space.bundle
ssh hsm 'cd ~/2026_graduation_project && git checkout -- scripts/fastdds_lan_peers.xml test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh 2>/dev/null; git fetch /home/hsm/code_space.bundle lee/hw-design-review && git checkout lee/hw-design-review && git merge --ff-only FETCH_HEAD'
```

- [ ] **Step 2: Jetson 컨테이너 recreate** (신규 env 장치 매핑 — HW 제거 상태이므로 기본 /dev/null): `ssh hsm 'cd ~/2026_graduation_project && docker compose -f docker/compose/docker-compose.jetson.yml up -d --force-recreate'` 후 `ros-humble-gazebo-msgs` apt 재주입(recreate로 소실) + colcon 빌드
- [ ] **Step 3: 분산 런 ×3** — §4.5 순서 엄수(매 회: Jetson `docker restart` → 데스크톱 `docker restart` + `run_sim_host.sh F1` + 스폰 확인 → Jetson nohup 스모크). 옵션: `GAZEBO_REMOTE=1 WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_ARM=1 WITH_PROFILE=1 NAV_GOAL_RETRIES=1` (ARM_SERIAL_PORT 미지정 = mock, 실물 팔 제거됨)
- [ ] **Step 4:** 판정 — 3회 모두 exit 0 + cpu_pct_peak < 600% + 아티팩트 수집. Jetson 접근 불가/사망 시: 폴백(사유 기록, 데스크톱 결과로 대체) 후 진행.
- [ ] **Step 5:** goals.py checkpoint

### Task 9: 기록 + 마감

- [ ] **Step 1:** `docs/session_wiki/2026-08-18_overnight_roundtrip/briefing.md` — 완료/실패/블로커/증거 경로/다음 할 일
- [ ] **Step 2:** improvement_report(§1.24 갱신 등)·Roadmap(06/07 진척)·TODO.md·AGENTS.md 현재 포커스 갱신
- [ ] **Step 3:** 최종 게이트 — `run_offline_tests.sh` + `check_portability.py` + contract 전부 PASS 확인 후 커밋·푸시
- [ ] **Step 4:** goals.py 최종 checkpoint (`--verify-cmd`/`--verify-evidence` 필수) + `python scripts/log_session.py`
