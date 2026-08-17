# 왕복 배달 시뮬 완성 설계 (2026-08-17)

## 1. 목표

한 층 왕복 배달 체인을 시뮬레이션에서 완벽히 돌게 만든다. 회의 결정: F3 확장 대신 **로비↔F2 한 층 움직임의 완성도**에 집중한다.

체인: `충전소 대기 → 택배함 픽업 → 로비 → 엘베 탑승 → F2 전환(월드+맵+팔 mock) → F2 배달 지점 → 엘베 복귀 → F2→F1 역전환 → 로비 → 충전소 복귀 대기`

완료 기준 (goal G001):

1. 데스크톱 단독 왕복 스모크 **10회 연속 성공** — OOM·경로 이탈·멈춤·ABORT 0회, 리소스 로그 수집
2. 보행자 동적 장애물이 왕복 전 구간 활성 상태에서 회피 성공
3. Jetson bundle 재동기화 후 분산 구성 **3회 완주** (불가 시 데스크톱 폴백 + 사유 기록)
4. docker/compose SLAM·robot_arm 세팅 HW-ready 정리 + 구동부 완성 시 변경 체크리스트 문서화
5. WITH_F3 옵션 유지(기본 OFF), 오프라인 테스트·portability·smoke contract 전부 PASS
6. 매 사이클 계획→테스트→검증→기록 커밋 + 아침 브리핑 문서

## 2. 사전 조사 결과 (2026-08-17 확인)

역방향 전환은 기존 코드가 이미 지원한다 — 신규 구현 범위가 작다.

- `swap_trigger.py`: 방향 무관 (`current_floor != last_floor`면 스왑 요청 생성)
- `verify_world_swap_state.py`: `--floor F1 --from-floor F2` 인자 지원, `map_expectations.py`에 F1 등록됨
- `floor_maps.yaml`: F1 `elevator_inside (0,0)` 시딩 존재 — orchestrator가 역전환 시 F1 맵 로드 + initialpose 시딩 가능
- `gazebo.launch.py`: `charge_station = (1.6, 0.0)` (elevator_exit 별칭)
- 보행자(`pedestrians.py`): F1 메인 복도 3명 + F2 뒤 복도 2명 transit 모델 — 층 전환과 무관하게 상주하므로 왕복 전 구간을 이미 커버. 스폰은 world 파일 밖이라 world swap의 delete 대상이 아님
- 로봇팔 mock(`arm_sequence`): 층 전환 ready마다 버튼 시퀀스 — 왕복이면 F2 1회 + F1 복귀 1회 = 완료 2회

## 3. 설계

### 3.1 스모크 왕복 확장 — `WITH_RETURN` 게이트

`run_l3_world_swap_smoke.sh`에 `WITH_RETURN=1` 옵션 추가 (기본 0 — 기존 결정적 스모크와 contract 테스트 보존).

f2_corridor 도착(=배달 완료 간주, footprint는 기존대로 F2 진입 시 이미 원복) 이후:

1. `f2_elevator_inside (0,0)` goal
2. `ros2 param set /floor_orchestrator_node target_floor F1` — **timeout 30s + 3회 재시도** (기존 F3 경로의 param set에 timeout이 없는 결함도 동시 수정)
3. `request_switch` 호출 (기존 timeout 45s 패턴)
4. `/elevator/state`에 F1 ARRIVED_OPEN 발행
5. `verify_world_swap_state.py --floor F1 --from-floor F2`
6. `WITH_ARM=1`이면 `verify_arm_sequence F1` (완료 누계 2회)
7. `align_robot_to_spawn F1` (teleport + costmap 클리어 — 시뮬 world-swap 위치 연속성 부기)
8. `f1_charge_station (1.6, 0.0)` goal → 대기 완료

제약: `WITH_RETURN=1`과 `WITH_F3=1` 동시 설정은 시작 시점에 에러로 거부 (왕복은 F1↔F2 전용).

### 3.2 반복 러너 — `run_roundtrip_repeat.sh`

신규 스크립트 `test_workspace/gazebo_world_swap/scripts/run_roundtrip_repeat.sh`:

- `REPEAT_N`(기본 10)회 반복, 각 회차는 `WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_PROFILE=1` 스모크 전체(cleanup 포함)를 새로 실행 — 회차 간 상태 오염 없음 (매 런 Gazebo 재기동 = 리셋 규칙 자동 충족)
- 회차별 아티팩트: `verification/repeat_<ts>/run_NN/` + 회차 결과·소요시간·리소스 피크·missed-rate를 집계한 `repeat_summary.md`
- 실패 시 즉시 중단(연속 카운터 리셋 조건 명확화), 실패 로그 보존
- 메모리 가드: 회차별 profile의 mem 피크 기록 + 회차 간 증가 추세 표기(누수 조기 발견), dmesg의 OOM killer 흔적 검사

### 3.3 분산 3회 — 기존 절차 재사용

portability policy §4.5 절차 그대로: Jetson bundle 재동기화 → 데스크톱 `run_sim_host.sh` → Jetson에서 `GAZEBO_REMOTE=1 WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_ARM=1 WITH_PROFILE=1 NAV_GOAL_RETRIES=1` nohup 스모크 ×3. 데스크톱에서 SSH로 오케스트레이션(순서 강제: Jetson 재시작 → 데스크톱 리셋 → 스모크). 팔은 mock(시리얼 미지정 — 실물 제거됨).

### 3.4 docker HW-ready

- compose 3종의 장치 매핑을 환경변수화 (`RPLIDAR_DEVICE`, `ARM_SERVO_DEVICE`, `OPENCR_DEVICE` — 기본값 현행 유지, 미장착 시 주석 대신 변수로 제어) — Jetson pull 충돌 단골 해소
- `docs/deployment/03_hw_update_checklist.md` 신규: 구동부(모터)·Depth 카메라 확정 시 바꿀 파일·파라미터 목록 (URDF 바퀴 스펙, nav2 속도 한계, OpenCR 브리지, udev, compose 장치)

### 3.5 기록

- 아침 브리핑: `docs/session_wiki/2026-08-18_overnight_roundtrip/briefing.md` — 완료/실패/블로커/증거 경로 한눈에
- improvement_report·Roadmap·TODO 갱신, 사이클마다 커밋 (브랜치 `lee/hw-design-review`)

## 4. 테스트 전략

- 오프라인: contract 테스트에 WITH_RETURN 기본값·왕복 goal 목록 가드 추가, 기존 26종 + 신규 전부 PASS 유지
- 시뮬 검증: 단일 왕복 1회 통과 → 보행자 왕복 1회 통과 → 반복 10회 → 분산 3회 순으로 단계 게이트
- `check_portability.py` 커밋 전 통과

## 5. 리스크

- Jetson 야간 무인 접근 실패 → 데스크톱 단독 폴백 + 사유 기록 (완료 기준 3의 명시적 폴백 조항)
- F1 복귀 시 보행자 `ped_cross`(x 3.5~4.2, y −2~2)가 복귀 경로와 교차 — 의도된 회피 검증 요소, `NAV_GOAL_RETRIES`로 일시 ABORT 흡수
- 반복 10회 중 Gazebo/Nav2 산발 기동 실패 → 회차 실패로 계수하고 원인 로그 보존 (기동 실패도 "멈춤 0회" 기준에 포함)
