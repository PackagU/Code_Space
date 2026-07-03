# Elevator Mission Test Workspace

작성일: 2026-05-28

## 목적

이 폴더는 기존 @src/ 패키지를 건드리지 않고 엘리베이터 층 이동 상태머신과 Nav2 mission PoC를 구현하기 위한 독립 테스트 워크스페이스다.

구현자는 먼저 @docs/simulation_test/05_elevator_state_machine/01_claude_handoff.md 를 읽고, 그 다음 @docs/simulation_test/05_elevator_state_machine/02_implementation_plan.md 의 체크리스트를 순서대로 수행한다.

## 경계

- 이 폴더 아래 파일은 새로 만들거나 수정해도 된다.
- 기존 @src/common_pkg/, @src/slam_pkg/, @src/drive_pkg/, @src/robot_arm_pkg/ 는 읽기만 한다.
- 검증 전에는 이 폴더의 코드를 기존 ROS2 workspace로 옮기지 않는다.
- MVP에서는 custom msg/srv 패키지를 만들지 않는다.

## 목표 구조

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
└── docs/
```

## 실행 개요

```bash
cd test_workspace/elevator_mission
colcon build --symlink-install
source install/setup.bash
```

상세 구현과 테스트 순서는 @docs/simulation_test/05_elevator_state_machine/02_implementation_plan.md 를 따른다.
