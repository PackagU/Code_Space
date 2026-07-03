# Test ROS2 Packages

이 폴더에는 엘리베이터 mission PoC용 ROS2 패키지만 둔다.

## 예정 패키지

| 패키지 | 역할 |
|--------|------|
| `elevator_mission_pkg` | py_trees 기반 delivery mission, Nav2 action client, relocalize |
| `elevator_sim_pkg` | mock elevator FSM, `/elevator/state`, `/elevator/call` |
| `floor_orchestrator_pkg` | 수동 2-Phase floor switch request와 ack 처리 |

`elevator_msgs` 패키지는 MVP에서 만들지 않는다.

