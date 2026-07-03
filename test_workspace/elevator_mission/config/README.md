# Elevator Mission Config

이 폴더에는 test workspace에서만 쓰는 설정을 둔다.

## 예정 파일

| 파일 | 역할 |
|------|------|
| `kku_nav_points.yaml` | F1/F2/F3 point registry |
| `delivery_missions.yaml` | 샘플 배송 mission |

좌표 값은 @src/common_pkg/config/kku_pre_simulation_map.yaml 의 설계 pose를 복사해서 시작하고, RViz/Nav2 테스트 후 이 테스트 워크스페이스 안에서만 보정한다.

실제 보정 절차는 @docs/simulation_test/05_elevator_state_machine/02_implementation_plan.md 의 `Task 1.5: Point Acquisition Workflow`를 따른다. 핵심은 `/amcl_pose` 기준 현재 로봇 pose를 `capture_nav_point.py`로 출력하고, 출력된 YAML 한 줄을 `kku_nav_points.yaml`에 반영하는 것이다.
