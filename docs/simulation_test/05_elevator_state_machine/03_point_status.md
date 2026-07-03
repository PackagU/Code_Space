# Elevator Mission Point Status

작성일: 2026-05-28

## 1. 목적

`test_workspace/elevator_mission/config/kku_nav_points.yaml` 의 각 좌표가 (a) `kku_pre_simulation_map.yaml` 기준 seed 값인지, (b) Nav2/AMCL로 보정된 실측 값인지 한 곳에서 추적한다. 캡처 작업 진행 상황을 표로 관리해서 어디까지 보정했는지 흩어지지 않게 한다.

## 2. 좌표 출처 규칙

- **seed**: `src/common_pkg/config/kku_pre_simulation_map.yaml` 의 설계 좌표를 그대로 옮긴 값.
- **captured**: 해당 층 Gazebo + Nav2 + RViz 실행 후 `scripts/capture_nav_point.py` 로 받은 `/amcl_pose` 기반 값.
- **manual**: 캡처가 어려운 경우 RViz 측정 등으로 수동 보정한 값. 사유 기록 필수.

## 3. 캡처 우선순위

| 순위 | 점 | 사용처 |
|------|----|--------|
| P0 (MVP 필수) | F1 `parcel_pickup`, F1 `elevator_entry`, F1 `elevator_inside` | mission `parcel_to_208` 시작 구간 |
| P0 (MVP 필수) | F2 `elevator_inside`, F2 `elevator_exit`, F2 `208` | mission `parcel_to_208` 종료 구간 |
| P1 (2번째 mission) | F1 동일 + F3 `elevator_inside`, F3 `elevator_exit`, F3 `307` | mission `parcel_to_307` |
| P2 (확장) | 각 층 나머지 방 번호 (201~207, 301~306, 308) | 다른 mission 정의 추가 시 |

## 4. F1 좌표

| Point | Type | 출처 | x | y | yaw_deg | 상태 | 캡처일 |
|-------|------|------|---|---|---------|------|--------|
| elevator_entry | elevator | seed | 1.60 | 0.00 | 180.0 | TODO (P0) | - |
| elevator_inside | elevator | seed | 0.00 | 0.00 | 0.0 | TODO (P0) | - |
| elevator_exit | elevator | seed | 1.60 | 0.00 | 0.0 | TODO (P1) | - |
| parcel_access | parcel | seed | 5.00 | -0.60 | -90.0 | TODO (P2) | - |
| parcel_pickup | parcel | seed | 5.00 | -1.40 | -90.0 | TODO (P0) | - |

## 5. F2 좌표

| Point | Type | 출처 | x | y | yaw_deg | 상태 | 캡처일 |
|-------|------|------|---|---|---------|------|--------|
| elevator_entry | elevator | seed | 1.60 | 0.00 | 180.0 | TODO (P1) | - |
| elevator_inside | elevator | seed | 0.00 | 0.00 | 0.0 | TODO (P0) | - |
| elevator_exit | elevator | seed | 1.60 | 0.00 | 0.0 | TODO (P0) | - |
| 201 | room | seed | 1.65 | 3.00 | 180.0 | TODO (P2) | - |
| 202 | room | seed | 2.35 | 3.00 | 0.0 | TODO (P2) | - |
| 203 | room | seed | 1.65 | 6.00 | 180.0 | TODO (P2) | - |
| 204 | room | seed | 2.35 | 6.00 | 0.0 | TODO (P2) | - |
| 205 | room | seed | 1.65 | 9.00 | 180.0 | TODO (P2) | - |
| 206 | room | seed | 2.35 | 9.00 | 0.0 | TODO (P2) | - |
| 207 | room | seed | 1.65 | 12.00 | 180.0 | TODO (P2) | - |
| 208 | room | seed | 2.35 | 12.00 | 0.0 | TODO (P0) | - |

## 6. F3 좌표

| Point | Type | 출처 | x | y | yaw_deg | 상태 | 캡처일 |
|-------|------|------|---|---|---------|------|--------|
| elevator_entry | elevator | seed | 1.60 | 0.00 | 180.0 | TODO (P1) | - |
| elevator_inside | elevator | seed | 0.00 | 0.00 | 0.0 | TODO (P1) | - |
| elevator_exit | elevator | seed | 1.60 | 0.00 | 0.0 | TODO (P1) | - |
| 301 | room | seed | 1.65 | 3.00 | 180.0 | TODO (P2) | - |
| 302 | room | seed | 2.35 | 3.00 | 0.0 | TODO (P2) | - |
| 303 | room | seed | 1.65 | 6.00 | 180.0 | TODO (P2) | - |
| 304 | room | seed | 2.35 | 6.00 | 0.0 | TODO (P2) | - |
| 305 | room | seed | 1.65 | 9.00 | 180.0 | TODO (P2) | - |
| 306 | room | seed | 2.35 | 9.00 | 0.0 | TODO (P2) | - |
| 307 | room | seed | 1.65 | 12.00 | 180.0 | TODO (P1) | - |
| 308 | room | seed | 2.35 | 12.00 | 0.0 | TODO (P2) | - |

## 7. 캡처 절차 요약

상세는 `test_workspace/elevator_mission/docs/point_capture_guide.md` 참고. 핵심 흐름.

```bash
# 층별로 1회씩
ros2 launch common_pkg gazebo.launch.py floor:=F1 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F1

# RViz: 2D Pose Estimate 로 초기 pose 정렬
# teleop/Nav2 goal 로 목표 point + yaw 위치에 로봇 정렬

python3 test_workspace/elevator_mission/scripts/capture_nav_point.py \
  --floor F1 --point elevator_entry --type elevator

# 출력 한 줄을 config/kku_nav_points.yaml floors.F1.points 아래에 반영
# 상태 표(§4~§6)에서 해당 행을 captured 로 갱신, 캡처일 기록

python3 test_workspace/elevator_mission/scripts/test_point_registry.py   # 회귀
```

## 8. 상태 갱신 규칙

- 캡처 직후 본 문서 표의 `출처` → `captured`, `x/y/yaw_deg` 실측 값으로 교체, `상태` → `OK`, `캡처일` 기입.
- 캡처 후 mission 실행에서 정지 위치가 어긋나면 `manual` 로 바꾸고 사유를 표 아래 노트에 기록.
- `kku_nav_points.yaml` 과 본 문서 표의 좌표는 항상 일치해야 한다. 둘 중 하나만 바꾸는 것 금지.

## 9. mission 노드 파라미터 참고

`elevator_mission_pkg/delivery_mission_node` 가 노출하는 파라미터.

| 파라미터 | 기본값 | 용도 |
|----------|--------|------|
| `mission_id` | `parcel_to_208` | `delivery_missions.yaml` 의 mission key |
| `dry_run_nav2` | `false` | `true` 면 NavigateToPoint 즉시 success — Nav2 없이 상태머신 검증 |
| `workspace_root` | 패키지 설치 경로 기반 자동 | YAML 탐색 루트 (`config/kku_nav_points.yaml`, `config/delivery_missions.yaml`) |
| `points_yaml` | `${workspace_root}/config/kku_nav_points.yaml` | point registry YAML 경로 override |
| `missions_yaml` | `${workspace_root}/config/delivery_missions.yaml` | mission YAML 경로 override |
| `tick_period_sec` | `0.5` | 시퀀서 tick 주기 |

콘솔 실행 예.

```bash
ros2 run elevator_mission_pkg delivery_mission_node \
  --ros-args \
  -p mission_id:=parcel_to_208 \
  -p dry_run_nav2:=true \
  -p workspace_root:=/ros2_ws/test_workspace/elevator_mission
```

## 10. 다음 액션

1. F1 Gazebo/Nav2 실행 → §4 P0 3개 캡처 → §4 표 갱신.
2. F2 Gazebo/Nav2 실행 → §5 P0 3개 캡처 → §5 표 갱신.
3. `test_point_registry.py` 회귀 통과 확인.
4. dry-run mission (`dry_run_nav2:=true`) 으로 상태머신 끝까지 확인.
5. F1→F2 수동 2-Phase 전체 데모.
6. F3 + `parcel_to_307` 진행 시 §6 P1 3개 캡처.
