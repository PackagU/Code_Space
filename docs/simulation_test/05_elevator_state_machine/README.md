# Elevator State Machine Test Plan

작성일: 2026-05-28

## 목적

이 폴더는 엘리베이터 층 이동 상태머신과 Nav2 연동 PoC를 기존 `src/`와 분리해서 구현하기 위한 문서 허브다.

새 Claude 대화에서는 먼저 @docs/simulation_test/05_elevator_state_machine/01_claude_handoff.md 를 읽고, 실제 구현은 @docs/simulation_test/05_elevator_state_machine/02_implementation_plan.md 순서대로 진행한다.

## 구현 경계

- 테스트 구현 루트: @test_workspace/elevator_mission/
- 기존 ROS2 패키지: @src/common_pkg/, @src/slam_pkg/, @src/drive_pkg/, @src/robot_arm_pkg/ 는 읽기만 한다.
- 기존 문서: @docs/simulation_test/01_environment/, @docs/simulation_test/02_gazebo_slam_mapping/, @docs/simulation_test/03_nav2_one_floor/, @docs/simulation_test/04_point_goal_delivery/ 는 참고만 한다.
- MVP는 수동 2-Phase floor switch 방식으로 구현한다.
- 시각화 업그레이드는 MVP 통과 후 A3-lite 방식으로 별도 진행한다.

## 핵심 결정

| 항목 | 결정 |
|------|------|
| 층 전환 MVP | B 방식: 사용자가 Gazebo/Nav2 floor를 수동 재실행하고 ack |
| 서비스 인터페이스 | B2 방식: `std_srvs/Trigger` + ROS parameter |
| 엘리베이터 상태 | `std_msgs/String` JSON payload |
| custom srv/msg | MVP에서 만들지 않음 |
| Gazebo 자동 world swap | MVP에서 제외 |
| 수직 통합 world | 보류 |
| 시각적 floor 이동 | MVP 후 A3-lite 가로 배치 world로 확장 |

## 참고

@docs/superpowers/plans/2026-05-28-elevator-mission-mvp.md 에도 같은 실행 계획을 저장했다. 현재 `.gitignore` 정책상 `docs/superpowers/` 는 추적되지 않으므로, 팀 공유와 Notion 동기화 기준 문서는 이 폴더의 @docs/simulation_test/05_elevator_state_machine/02_implementation_plan.md 이다.
