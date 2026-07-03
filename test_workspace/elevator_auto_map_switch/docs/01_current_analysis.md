# Current Analysis For Auto Map Switch

작성일: 2026-06-02

## 목표

엘리베이터 층 이동 자체가 확인된 상태에서, 로봇이 엘리베이터에 탑승하고 목표층에 도착하면 사람이 수동으로 ack를 누르지 않아도 Nav2가 목표층 저장 맵으로 자동 전환되게 한다.

이번 문서는 현재 코드가 어디까지 준비되어 있고, 자동 맵 전환을 위해 어떤 부분을 새 PoC에서 다뤄야 하는지 분석한다.

## 기존 legacy PoC 상태

기존 수동 엘리베이터 mission PoC는 `test_workspace/elevator_mission/` 아래에 있다. 이 폴더는 앞으로 legacy로 보존한다.

| 영역 | 위치 | 현재 상태 |
|------|------|-----------|
| mission node | `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/delivery_mission_node.py` | 택배 픽업, 엘베 탑승, 층 전환, 재초기화, 목적지 이동 sequence 보유 |
| behavior | `test_workspace/elevator_mission/src/elevator_mission_pkg/elevator_mission_pkg/behaviors.py` | `NavigateRoute`, `CallElevator`, `WaitElevatorArrived`, `SwitchFloor`, `Relocalize` 구현 |
| floor orchestrator | `test_workspace/elevator_mission/src/floor_orchestrator_pkg/floor_orchestrator_pkg/floor_orchestrator_node.py` | 수동 2-Phase request/ack 방식 |
| elevator sim | `test_workspace/elevator_mission/src/elevator_sim_pkg/elevator_sim_pkg/elevator_sim_node.py` | `/elevator/state` JSON 발행, 층 이동 시간 기반 mock |
| point registry | `test_workspace/elevator_mission/config/kku_nav_points.yaml` | F1/F2/F3 point와 map yaml 경로 보유 |
| routing | `test_workspace/elevator_mission/scripts/orthogonal_router.py` | 복도 중심선 기반 직각 waypoint 생성 |

## 현재 성공한 것

2026-06-01 기준으로 waypoint 기반 직각 라우터가 추가되었고, `delivery_mission_node`가 단일 대각선 `NavigateToPose` 대신 route waypoint를 순차 전송하도록 바뀌었다. F2/F3 방 이동은 `main_corridor_x=2.0`, F1 택배존 이동은 `main_corridor_y=0.0` 기준으로 ㄱ자 또는 ㄴ자 경유점을 만든다.

2026-06-02 분석 시점에 오프라인 회귀는 통과했다.

```text
PASS point registry
PASS behavior dry-run
PASS navigate route behavior
PASS orthogonal router
PASS nav2 orthogonal tuning
```

## 현재 막혀 있는 지점

자동 맵 전환 관점에서 병목은 `SwitchFloor`와 `floor_orchestrator`가 아직 수동 ack를 전제로 한다는 점이다.

현재 흐름은 다음과 같다.

```text
mission
  CallElevator(target_floor)
  SwitchFloor(target_floor, elevator_inside)
    -> /floor_orchestrator/request_switch
    -> WAITING_FOR_USER_ACK

operator
  Gazebo/Nav2를 목표층으로 수동 재실행
  /floor_orchestrator/ack 호출

mission
  WaitElevatorArrived(target_floor, open)
  Relocalize(elevator_exit@target_floor)
  NavigateRoute(destination)
```

자동화하려는 흐름은 다음과 같다.

```text
mission
  CallElevator(target_floor)
  SwitchFloor(target_floor, elevator_inside)

auto orchestrator
  /elevator/state에서 목표층 도착과 문 열림 확인
  Nav2 map server에 목표층 map load 요청
  costmap clear 수행
  current_floor와 map_loaded 상태 발행

mission
  Relocalize(elevator_exit@target_floor)
  NavigateRoute(destination)
```

## 자동 전환에 필요한 신호

| 신호 | 출처 | 사용 목적 |
|------|------|-----------|
| `target_floor` | mission 또는 orchestrator parameter | 어떤 층 map을 load할지 결정 |
| `spawn_point_id` | mission 또는 orchestrator parameter | 재초기화 기준 pose 결정 |
| `/elevator/state` | elevator sim 또는 실제 elevator bridge | 목표층 도착과 문 열림 판단 |
| 층별 map yaml | point registry 또는 별도 config | Nav2 map server에 전달 |
| `/initialpose` | mission 또는 orchestrator | AMCL 초기 pose 재설정 |
| Nav2 map load 결과 | map server service response | mission 재개 가능 여부 판단 |

## 추천 설계 방향

기존 수동 legacy를 수정하지 않고, 새 폴더에서 `auto_floor_orchestrator`를 만든다. 이 노드는 legacy `floor_orchestrator`의 인터페이스를 최대한 유지하되 내부 동작만 자동화한다.

추천 인터페이스는 다음과 같다.

| 이름 | 타입 | 의미 |
|------|------|------|
| `/floor_orchestrator/request_switch` | `std_srvs/Trigger` | target floor 전환 시작 |
| `/floor_orchestrator/status` | `std_msgs/String` JSON | current floor, target floor, pending, map_loaded, phase |
| `/elevator/state` | `std_msgs/String` JSON | 목표층 도착 및 문 열림 감지 |
| `/map_server/load_map` | `nav2_msgs/srv/LoadMap` | 저장 맵 교체 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL pose 재초기화 |

## 주요 리스크

| 리스크 | 영향 | 대응 |
|--------|------|------|
| map server 서비스명이 실제 launch에서 다를 수 있음 | load_map 호출 실패 | 서비스명 parameter화 및 probe 스크립트 작성 |
| map load 후 costmap이 이전 층 장애물을 유지할 수 있음 | 잘못된 경로 생성 | global/local costmap clear 서비스 호출 |
| AMCL initialpose가 map load보다 먼저 발행될 수 있음 | 위치추정 실패 | map_loaded 이후 initialpose 발행 순서 보장 |
| Gazebo world 자체가 아직 목표층 geometry로 바뀌지 않을 수 있음 | sensor와 map 불일치 | 이번 PoC의 전제와 별도 world/respawn 자동화 범위를 분리 |
| point 좌표가 seed 상태일 수 있음 | 도착 위치 오차 | 캡처된 좌표 사용 전까지 dry-run과 RViz 검증을 분리 |

## 결론

다음 작업은 legacy mission 코드를 건드리는 것이 아니라, 새 테스트 폴더에서 자동 전환 orchestrator를 작게 검증하는 방식이 맞다. `SwitchFloor`의 외부 계약은 유지하고, 수동 ack 대신 `elevator arrived -> load map -> clear costmap -> publish ready` 상태를 만들어 mission이 이어가도록 한다.
