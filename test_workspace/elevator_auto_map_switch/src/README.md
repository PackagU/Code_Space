# Source

작성일: 2026-06-02 · 갱신: 2026-06-12 (구현 완료)

## 현재 패키지

| 패키지 | 역할 |
|--------|------|
| `auto_floor_orchestrator_pkg` | `/elevator/state` 도착 감지 → Nav2 map load → costmap clear → `/initialpose` → ready status 발행. ROS 노드 이름은 legacy 호환을 위해 `floor_orchestrator_node` (이유: `FeedBack/05_auto_floor_orchestrator_node.md`) |

계획 단계 후보였던 `auto_elevator_mission_pkg`(최소 mission)는 만들지 않았다. 무수정 legacy mission + 이 orchestrator 조합이 목표 시나리오이고, 최소 mission이 검증할 내용은 `scripts/test_orchestrator_ros_smoke.py`가 자동으로 검증한다 (근거: `FeedBack/00_overall_design_feedback.md` 마지막 절).

## 경계

- 기존 `test_workspace/elevator_mission/src/`는 수정하지 않는다.
- 기존 `src/slam_pkg/`, `src/common_pkg/`, `src/drive_pkg/`, `src/robot_arm_pkg/`도 이 PoC에서는 읽기만 한다.
- legacy 수동 `floor_orchestrator_node`와 이 패키지의 노드를 동시에 실행하지 않는다 (같은 노드명/서비스 공유).
- production 승격은 L3/L5 검증 후 별도 결정한다.
