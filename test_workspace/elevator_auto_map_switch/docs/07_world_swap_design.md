# Gazebo World Swap 설계 (2026-06-25)

> L3에서 Nav2 맵(occupancy grid)만 F1→F2로 바꾸는 것까지 검증됐다. 이 문서는 같은 순간
> Gazebo 물리 world도 F2로 바꿔서 LiDAR 스캔과 맵이 일치하게 만드는 코드 구조를 설계한다.
> 구현은 다음 세션에서 한다. 이 문서는 "무엇을 어떻게" 까지만 정한다.

## 1. 현재 갭

- `auto_floor_orchestrator`는 Nav2 `/map_server/load_map`만 호출한다. Gazebo world는 그대로다.
- `gazebo.launch.py floor:=F2`로 `kku_f2.world`를 띄울 수는 있으나, 이건 **시작 시 한 번**뿐이고
  미션 도중 런타임에 world를 바꾸는 장치가 없다.
- 결과: 맵은 F2인데 LiDAR는 F1 world를 스캔 → AMCL이 정합 실패로 떨린다(L3에서 관찰된 노이즈).

## 2. 설계 원칙 (먼저 못박는다)

- **orchestrator는 Gazebo를 몰라야 한다.** 실물 로봇에는 Gazebo가 없다. 실물에서는 건물 자체가
  이미 그 층이다. 그래서 world swap은 **시뮬레이션 전용 별도 컴포넌트**로 분리하고, orchestrator
  코드는 건드리지 않는다. orchestrator는 지금처럼 sim/real 양쪽에서 동일하게 동작한다.
- world swap 컴포넌트는 orchestrator가 이미 publish하는 `/floor_orchestrator/status`를 **구독**해서
  트리거된다. 둘 사이에 새 의존성을 만들지 않는다.

## 3. 발견한 enabler (이게 방법 B를 깔끔하게 만든다)

코드 확인 결과:

- 각 world의 건물 전체가 **단일 모델**이다: `kku_f1_building`, `kku_f2_building`, `kku_f3_building`.
- 엘리베이터 기하학이 모든 층에서 동일하고, `floor_maps.yaml`의 `elevator_inside`가 모든 층에서
  `(0, 0, 0)`이다. 맵이 바뀌는 순간 로봇은 물리적으로 엘베 안 = 원점에 있다.
- 즉 world를 바꿀 때 **로봇은 (0,0)에 그대로 두고 건물 모델만 교체**하면 LiDAR가 새 층 벽을
  스캔하게 된다. 로봇 텔레포트가 필요 없다.

## 4. 두 가지 방법

### 방법 A — gzserver 재시작

맵 전환 ready 신호를 받으면 gzserver를 죽이고 `gazebo.launch.py floor:=F2`로 재기동 + 로봇 재스폰.

- 장점: 구현 단순. 기존 launch 재사용. 확실히 새 world가 뜬다.
- 단점: sim clock 점프(`use_sim_time` 소비자 전체 영향), TF 트리 재초기화, 로봇 순간이동,
  센서/토픽 재연결, Nav2 lifecycle 재정렬 필요할 수 있음.
- 필요한 수정 1가지: `gazebo.launch.py`의 스폰 포즈가 현재 `elevator_exit(1.6,0)` 하드코딩이다.
  world swap 재스폰은 `elevator_inside(0,0)`여야 orchestrator initialpose와 일치한다.
  → launch에 `spawn_point` 인자 추가(기본 exit, swap 시 inside).

### 방법 B — 런타임 모델 교체 (단일 gazebo 유지)

gzserver를 유지한 채 Gazebo 서비스로 건물 모델만 갈아끼운다.

- 흐름: `/delete_entity`로 `kku_f1_building` 삭제 → `/spawn_entity`로 `kku_f2_building` 스폰
  (world 원점 정렬). 로봇은 `(0,0)`에 그대로.
- 장점: clock 연속, 로봇 텔레포트 없음, Nav2/AMCL 재시작 불필요. 실전 흐름에 가장 가깝다.
- 단점: 건물 모델을 `.world` 인라인이 아니라 **스폰 가능한 SDF**로 추출해야 한다
  (예: `model://kku_f2_building` 또는 모델 XML 문자열). 새 벽이 로봇 주변에 스폰될 때 충돌/물리
  안정화 시간 고려.

### 비교 / 선택

| 기준 | 방법 A 재시작 | 방법 B 모델 교체 |
|------|--------------|------------------|
| 구현 난이도 | 낮음 | 중간 |
| clock/TF 연속성 | 끊김 | 연속 |
| 실전 유사도 | 낮음 | 높음 |
| 선행 작업 | spawn_point 인자 추가 | building 모델 SDF 추출 |

권장: **PoC E2E는 방법 A로 먼저** 관통(가장 빠른 검증). 그다음 **방법 B를 목표 구조**로 전환.

## 5. 트리거 연동 지점

시뮬 전용 `world_swap` 노드(또는 스크립트)를 새로 만든다.

- 구독: `/floor_orchestrator/status` (std_msgs/String, JSON). 필드 `phase`, `current_floor`,
  `target_floor`, `pending`, `map_loaded` 사용 가능.
- 트리거 조건(PoC): `map_loaded=true`이고 `current_floor`가 이전과 달라진 시점에 1회 swap.
  중복 발행에 안전하도록 마지막 처리한 층을 기억한다(orchestrator 상태머신과 동일 패턴).
- 방법 A에서는 트리거 시 gzserver 재기동, 방법 B에서는 delete+spawn 호출.

## 6. 재스폰 포즈 출처

- world swap 시 로봇 포즈/initialpose의 SSOT는 `config/floor_maps.yaml`의 `elevator_inside`다.
- 방법 A 재스폰 좌표, 방법 B 텔레포트(필요 시) 모두 이 값을 읽는다. launch의 `SPAWN_POSES`
  (fresh start용 exit 포즈)와 혼동하지 않는다.
- 현재 모든 층 `elevator_inside=(0,0,0)`라 방법 B는 텔레포트 자체가 불필요.

## 7. 시퀀싱

이상적 순서(정합 끊김 최소화):

```text
orchestrator: load_map(F2) 성공
  -> world swap (A: 재기동 / B: 건물 모델 교체)   <- 새 컴포넌트
  -> orchestrator: costmap clear -> initialpose(elevator_inside) -> READY
  -> AMCL 재수렴 (이제 LiDAR가 F2 벽을 봄)
```

PoC에서는 world swap이 READY 직후에 일어나도 허용(맵-world 불일치가 1초 미만). 정밀하게는
`map_loaded=true` 시점에 swap이 끝나는 게 좋다.

## 8. 리스크

- 방법 A: clock 점프로 Nav2/AMCL/RViz 타임스탬프 꼬임 → lifecycle 재구성 또는 재기동 필요.
- 방법 B: 인라인 building을 스폰 가능한 SDF로 추출하는 작업, 벽 스폰 시 로봇과의 충돌/물리 튐.
- 공통: Nav2 맵 로드와 world swap의 순서 경쟁(transient 불일치). 멱등 트리거로 완화.
- 공통: 실물에는 world swap이 없어야 한다 — 이 노드는 sim launch에서만 띄운다.

## 9. 완료 기준 (DOD)

- 맵 전환 후 RViz 맵과 Gazebo world가 둘 다 F2.
- LiDAR 스캔이 F2 맵과 정합되어 AMCL이 수렴한다.
- 목적지까지 경로 생성 + 도착 로그(L5 E2E).

## 10. 다음 세션 작업 순서

1. `gazebo.launch.py`에 `spawn_point` 인자 추가(기본 exit).
2. 시뮬 전용 `world_swap` 노드: `/floor_orchestrator/status` 구독 → 방법 A 재기동 트리거.
3. F1→F2 E2E 한 번 관통, 위 DOD 확인.
4. (그 다음) building 모델 SDF 추출 → 방법 B로 전환.
