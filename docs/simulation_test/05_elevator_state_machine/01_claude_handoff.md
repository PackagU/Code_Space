# Claude Handoff For Elevator Mission MVP

작성일: 2026-05-28

## 1. 새 대화에서 먼저 읽을 파일

새 Claude 대화는 아래 파일을 순서대로 읽고 시작한다.

1. @AGENTS.md
2. @TODO.md
3. @docs/_style/notion_markdown_style.md
4. @docs/simulation_test/02_gazebo_slam_mapping/01_kku_pre_simulation_plan.md
5. @docs/simulation_test/02_gazebo_slam_mapping/02_kku_pre_simulation_completion_report.md
6. @docs/simulation_test/03_nav2_one_floor/03_run_one_floor_nav2.md
7. @docs/simulation_test/04_point_goal_delivery/01_kku_nav2_point_goal_workflow.md
8. @src/common_pkg/config/kku_pre_simulation_map.yaml
9. @src/common_pkg/launch/gazebo.launch.py
10. @src/slam_pkg/launch/kku_navigation.launch.py
11. @docs/simulation_test/05_elevator_state_machine/02_implementation_plan.md

`git pull origin dev` 는 AGENTS 규칙상 먼저 시도한다. 현재 세션에서는 원격 `dev` ref가 없어서 실패했다. 새 대화에서도 같은 문제가 나면 원격 브랜치를 확인하고, 구현은 기존 파일을 덮어쓰지 않는 조건으로 계속한다.

## 2. 사용자 의도

사용자는 엘리베이터 층 이동 상태머신을 만들고, Nav2가 엘리베이터 탑승과 층 이동을 포함한 배송 mission을 수행하는 구조를 원한다.

가장 중요한 제한은 다음과 같다.

- 기존 코드와 폴더를 삭제하거나 overwrite 하지 않는다.
- 테스트용 새 폴더에 독립 구현한다.
- 나중에 검증이 끝나면 쉽게 기존 `src/` 패키지로 승격할 수 있게 만든다.
- 2026-05-31까지 반드시 끝낼 필요는 없다.
- 다만 빠르게 MVP를 만들어 실제 테스트하는 흐름을 우선한다.

## 3. 최종 구현 결정

Claude가 처음 제시한 4-Layer 아키텍처는 유지한다. 단, MVP에서는 위험한 부분을 줄인다.

| 영역 | Claude 초안 | 최종 MVP 결정 |
|------|-------------|---------------|
| Mission Layer | `delivery_mission_node` + py_trees_ros | 유지 |
| Skill Layer | `NavigateToPoint`, `CallElevator`, `WaitElevatorArrived`, `SwitchFloor`, `Relocalize`, `WaitForAck` | 유지 |
| Infra Layer | `floor_orchestrator_node`, `elevator_sim_node` | 유지 |
| Config | `kku_nav_points.yaml`, `delivery_missions.yaml` | 유지 |
| custom interface | `elevator_msgs/srv/SwitchFloor.srv` | 제외 |
| world swap | Gazebo 재시작 또는 world swap | MVP에서는 수동 2-Phase ack |
| 시각화 | 미정 | MVP 후 A3-lite 가로 배치 world |

MVP는 B 방식이다. 로봇이 엘리베이터 내부 point에 도착하면 mission이 floor switch request를 내고 기다린다. 사용자가 별도 터미널에서 현재 Gazebo/Nav2를 종료한 뒤 목표 층으로 다시 launch하고, ack 서비스를 호출하면 mission이 다음 단계로 진행한다.

point 좌표는 설계 YAML의 초기값만 믿지 않는다. 구현 계획의 `Task 1.5: Point Acquisition Workflow`에 따라 `/amcl_pose` 기반 캡처 스크립트와 가이드를 만든 뒤, 실제 F1/F2/F3 Nav2 실행 상태에서 `elevator_inside`, `elevator_exit`, 배송지 방 번호를 보정한다.

## 4. MVP ROS 인터페이스

MVP에서는 custom msg/srv 패키지를 만들지 않는다.

| 이름 | 타입 | 발행자 또는 서버 | 소비자 또는 클라이언트 | 의미 |
|------|------|------------------|------------------------|------|
| `/elevator/call` | `std_msgs/String` | mission | elevator sim | 목표층 또는 door command |
| `/elevator/state` | `std_msgs/String` JSON | elevator sim | mission | 현재층, 문 상태, 목표층, 상태 |
| `/floor_orchestrator/request_switch` | `std_srvs/Trigger` | floor orchestrator | mission | pending target floor 요청 수락 |
| `/floor_orchestrator/ack` | `std_srvs/Trigger` | floor orchestrator | 사용자 터미널 | 수동 floor 전환 완료 승인 |
| `/floor_orchestrator/status` | `std_msgs/String` JSON | floor orchestrator | mission | pending, current floor, ack 상태 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | mission | AMCL | 목표층 재초기화 pose |

`/floor_orchestrator/request_switch` 호출 전 mission은 parameter client로 `target_floor`와 `spawn_point_id`를 설정한다. 기본값은 `target_floor=F2`, `spawn_point_id=elevator_inside` 이다.

## 5. 수동 2-Phase 실행 흐름

```text
Terminal 1: mission
  GoToPickup
  LoadParcel mock ack
  GoToElevatorEntryF1
  CallElevator F2
  WaitElevatorAtF1
  EnterElevator
  SwitchFloor request F2
  WAITING_FOR_USER_ACK

Terminal 2: user
  Ctrl+C existing F1 Gazebo/Nav2
  ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
  ros2 service call /floor_orchestrator/ack std_srvs/srv/Trigger

Terminal 1: mission
  Relocalize elevator_exit@F2
  ExitElevator
  GoToRoom 208
  delivered mock ack
```

이 흐름은 자동 world swap보다 덜 화려하지만, 상태머신과 Nav2 연동을 가장 빨리 검증한다. 자동화는 floor orchestrator 내부 구현만 바꾸면 이어 붙일 수 있게 한다.

## 6. 테스트 워크스페이스 구조

실제 구현은 @test_workspace/elevator_mission/ 아래에서만 한다.

```text
test_workspace/elevator_mission/
├── README.md
├── src/
│   ├── elevator_mission_pkg/
│   ├── elevator_sim_pkg/
│   └── floor_orchestrator_pkg/
├── config/
│   ├── kku_nav_points.yaml
│   └── delivery_missions.yaml
├── launch/
├── scripts/
│   ├── capture_nav_point.py
│   ├── test_point_registry.py
│   ├── test_behaviors_dryrun.py
│   └── run_demo.sh
└── docs/
    ├── point_capture_guide.md
    └── completion_report.md
```

초안의 `elevator_msgs/` 패키지는 만들지 않는다. 이 결정은 custom srv 빌드 설정에 시간을 쓰지 않기 위한 것이다.

## 7. A3-lite 확장 기준

MVP 통과 후 시각적 이동이 필요하면 A3-lite를 추가한다.

- F1, F2, F3 world geometry를 하나의 통합 world에 x축 방향으로 나란히 배치한다.
- mission/floor_orchestrator 인터페이스는 그대로 둔다.
- floor_orchestrator 내부만 manual ack에서 `gazebo/set_entity_state` 기반 teleport로 교체한다.
- 수직 적층 A3a는 보류한다. TF, AMCL, LiDAR 간섭, 층별 collision 때문에 디버깅 비용이 높다.

## 8. 완료 기준

- `python3 test_workspace/elevator_mission/scripts/test_point_registry.py` 통과
- `python3 test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py` 통과
- `colcon build --symlink-install` 이 @test_workspace/elevator_mission/ 에서 통과
- `ros2 launch elevator_sim_pkg elevator_sim_only.launch.py` 실행 시 `/elevator/state` 가 5Hz로 발행
- 수동 2-Phase 흐름으로 F1 `parcel_pickup` 에서 F2 `208`까지 mission이 완료
- @test_workspace/elevator_mission/docs/completion_report.md 작성

## 9. 실행 계획 파일

팀 공유와 다음 Claude 작업 기준은 @docs/simulation_test/05_elevator_state_machine/02_implementation_plan.md 이다.

@docs/superpowers/plans/2026-05-28-elevator-mission-mvp.md 에도 같은 내용이 있지만, 현재 `.gitignore` 정책상 `docs/superpowers/` 는 Git 추적 대상이 아니다. 새 대화에서는 추적 가능한 `02_implementation_plan.md` 를 우선한다.
