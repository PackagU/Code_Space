# 05. 엘리베이터 내부 시뮬레이션 + 멀티층 자동 전환

> 상태: [40%] 🔄 · 담당: Lee (실물 인터페이스는 Kim 협의 필요) · 선행: 01, 03 · 갱신: 2026-07-03
> 방식 변경 기록: 멀티층 자동 전환은 본 문서의 권장안(teleport A안)이 아니라
> **Gazebo building model swap**(delete_entity+spawn_entity)으로 구현·검증됨 — 로봇 pose를
> 유지한 채 건물만 교체해 teleport 없이 동일 효과. 남은 것: 엘베 칸 내부 모델링(실측 후),
> 문 개폐 브리지, 탑승/하차 도킹 정밀도.

## 목표

엘베 탑승 → 문 닫힘 → 층 이동 → 문 열림 → 하차를 Gazebo 안에서 연속으로 재생한다. 현 PoC의 마지막 구멍인 "Nav2 맵만 바뀌고 Gazebo 건물은 그대로"를 메우는 단계다.

## 작업 단계

### 1단계: 엘베 칸 모델링

- [ ] 실측 치수(03에서 확보)로 엘베 칸 내부 모델 작성: 3벽 + 문
- [ ] 문 개폐 구현 — 두 방식 중 택1:
  - 방식 A (권장, 단순): 문 벽 모델을 elevator state에 맞춰 spawn/delete (`/gazebo/spawn_entity`, `/gazebo/delete_entity` 서비스)
  - 방식 B (정교): prismatic joint + 플러그인으로 슬라이딩 문 애니메이션
- [ ] elevator_sim의 `door_state`와 Gazebo 문 상태를 동기화하는 작은 브리지 노드 작성 (신규 패키지, legacy 무수정 원칙 유지)

### 2단계: 탑승/하차 정밀 주행

- [ ] `elevator_inside` 도킹 허용 오차 정의 (예: ±0.10m, ±10도) 후 도킹 성공률 측정
- [ ] 칸 내부에서 하차 방향 정렬: 제자리 회전 vs 후진 하차 비교, 실로봇 회전반경(02)으로 판단
- [ ] 문턱(엘베-복도 단차) 모델 추가 시 통과 확인 — hardware_spec.md §2(모터)/§3(문턱 실측) 연계

### 3단계: 멀티층 world 자동 전환

- [ ] 층 이동 시뮬 방식 결정 — 두 방식 중 택1:
  - 방식 A (권장, A3-lite): world는 그대로 두고 도착 이벤트 시 로봇 모델만 목표층 엘베 칸 위치로 teleport (`/gazebo/set_entity_state`). 층별 world가 동일 좌표계이므로 좌표 변환 불필요
  - 방식 B: 층별 world swap (Gazebo 재시작) — 느리고 자동화 어려움, 비권장
- [ ] teleport 트리거를 auto orchestrator의 finalizing 단계 또는 elevator sim 도착 이벤트에 연결 (신규 노드, 기존 코드 무수정)
- [ ] 검증: F1 탑승 → F2 하차 → F3 재탑승까지 사람 개입 0회

### 4단계: 실물 인터페이스 계약 고정

- [ ] `/elevator/call`, `/elevator/state` JSON 스키마를 문서로 고정 (현 elevator_sim 형식 기준)
- [ ] 추후 교체 경로 명시: mock → 로봇팔 버튼 누름(Kim) 또는 건물 연동. mission/orchestrator는 어느 쪽이든 무수정이어야 함

## 완료 기준

- 탑승~하차 연속 재생 1회 (사람 개입 0회).
- 도킹 성공률 10회 중 9회 이상.

## 리스크 / 안전·보안 체크

| 리스크 | 대응 |
|--------|------|
| 칸 내부에서 LiDAR가 4벽에 갇혀 AMCL 발산 | 칸 내부에서는 위치 추정을 신뢰하지 않는 설계 유지 — 하차 후 initialpose 재초기화(현 PoC 동작)가 이미 이 대응 |
| teleport 직후 센서/odom 불연속 | teleport → initialpose → costmap clear 순서 보장 (auto orchestrator finalizing에 통합) |
| 문 개폐 타이밍과 주행 충돌 | 문이 완전히 열린 상태(`door_state=open`)에서만 진입/하차 — 현 게이트 유지 |

[08_safety_security.md](./08_safety_security.md) 게이트: "문 닫힘 중 진입 금지"가 시나리오 테스트에 포함되어 있는지.
