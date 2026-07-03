# 06. 통합 E2E 반복 안정성

> 상태: [60%] 🔄 · 담당: Lee · 선행: 04, 05 · 갱신: 2026-07-03
> 실제 진행: 부하 강건성 P0~P3(제어 계측→graceful degradation 튜닝→WITH_STRESS 재현→
> SCHED_RR RT 보호, STRESS=8에서 3회 연속 완주), 고장 주입 2종 자동화(WITH_RECOVERY/
> WITH_LOC_FAULT), 회귀 가드. 남은 것: 반복 10회 9/10 통과율 정식 측정, 고장 주입 5종 완결.

## 목표

"한 번 성공"을 "반복해도 성공, 고장 나면 안전하게 정지"로 끌어올린다. Jetson 투입 가부를 판정하는 단계다.

## 작업 단계

### 1단계: 반복 E2E

- [ ] 전체 미션(F1 픽업 → F2/F3 배달 → 복귀) 10회 연속 실행 스크립트 작성 (결과 자동 기록: 소요 시간, 실패 단계, 로그 경로)
- [ ] 목표 통과율 9/10 이상. 실패는 전부 실패 모드 카탈로그에 기록

### 2단계: 고장 주입 (각 1회 이상)

| 고장 | 기대 동작 |
|------|----------|
| elevator sim 무응답 (노드 kill) | mission이 해당 단계에서 대기/정지, 폭주 없음 |
| load_map 실패 (잘못된 경로 주입) | `phase=failed`, `pending=true` 유지, 주행 재개 없음 (현 PoC 보장) |
| Nav2 goal 거부/플래너 실패 반복 | recovery 후 한계 횟수 도달 시 mission FAILURE 정지 |
| localization 강제 오염 (틀린 initialpose 주입) | 회복 또는 정지 — 어느 쪽이든 충돌 없음 |
| mission 노드 재시작 | 재시작 후 운영자가 안전하게 재개할 수 있는 절차 문서화 |

### 3단계: 감시 장치

- [ ] mission 단계별 타임아웃(watchdog) 추가 — 단계가 N분 넘게 RUNNING이면 FAILURE 정지
- [ ] 소프트 E-stop: `/emergency_stop` 토픽 → cmd_vel 게이트 노드 (08 문서 사양)
- [ ] 미션 결과를 `scripts/log_session.py` 형식으로 자동 축적

### 4단계: 구조 결정

- [ ] py_trees_ros 승격 여부 결정: 현 경량 sequencer로 충분하면 유지 (behavior 인터페이스가 호환되므로 미루는 비용 낮음 — legacy 보고서 §6)
- [ ] test_workspace PoC들의 production `src/` 승격 범위 결정 (AGENTS 규칙 11에 따라 SLAM 구현 코드는 `SLAM/` repo 반영 검토)

## 완료 기준

- 10회 반복 결과표 작성, 통과율 9/10 이상.
- 고장 주입 5종 전부 "충돌·폭주 없음" 확인.

## 리스크 / 안전·보안 체크

| 리스크 | 대응 |
|--------|------|
| 간헐 실패의 원인 미상 | 실패 시 rosbag 기록을 켜두고 반복 — 재현 자료 확보 |
| 타임아웃 값이 자의적 | 10회 반복의 단계별 소요 시간 분포로 설정 (최대치 × 2) |

[08_safety_security.md](./08_safety_security.md) 게이트: E-stop 경로(소프트)가 어느 mission 단계에서도 1초 내 정지로 이어지는지.
