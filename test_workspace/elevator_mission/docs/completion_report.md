# Elevator Mission MVP Completion Report

작성일: 2026-05-28

## 1. 구현 범위

- test workspace: `test_workspace/elevator_mission/`
- 기존 src 패키지 수정 여부: 없음 (`src/common_pkg`, `src/slam_pkg`, `src/drive_pkg`, `src/robot_arm_pkg` 모두 미수정)
- floor switch 방식: manual 2-Phase
- custom msg/srv 여부: 없음 (`std_msgs/String`, `std_srvs/Trigger`, `geometry_msgs/PoseWithCovarianceStamped`, Nav2 `NavigateToPose`만 사용)

## 2. 생성 파일

| 파일 | 역할 |
|------|------|
| `config/kku_nav_points.yaml` | 층별 point registry (F1/F2/F3 seed 좌표) |
| `config/delivery_missions.yaml` | `parcel_to_208`, `parcel_to_307` mission 정의 |
| `scripts/point_registry.py` | YAML 파서 + `PointRegistry`, `NavPoint` |
| `scripts/test_point_registry.py` | offline 회귀 테스트 |
| `scripts/capture_nav_point.py` | `/amcl_pose` 1회 캡처 후 YAML 한 줄 출력 |
| `scripts/test_behaviors_dryrun.py` | mission step 순서 회귀 테스트 |
| `scripts/run_demo.sh` | 수동 2-Phase 데모 안내 |
| `docs/point_capture_guide.md` | 좌표 캡처 절차 |
| `src/elevator_sim_pkg/` | mock elevator FSM (5Hz `/elevator/state` 발행, `/elevator/call` 수신) |
| `src/floor_orchestrator_pkg/` | `request_switch` / `ack` 2-Phase 서비스 + `/floor_orchestrator/status` |
| `src/elevator_mission_pkg/` | mission model + dry-run tree + behavior 6종 + delivery 노드 + demo launch |

## 3. 검증 결과 (host에서 수행)

```text
$ python3 test_workspace/elevator_mission/scripts/test_point_registry.py
PASS point registry

$ python3 test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
PASS behavior dry-run

$ bash -n test_workspace/elevator_mission/scripts/run_demo.sh
(exit 0)

$ source /opt/ros/humble/setup.bash && \
  PYTHONPATH=test_workspace/elevator_mission/src/elevator_mission_pkg \
  python3 -c "from elevator_mission_pkg import behaviors, mission_model, mission_tree, delivery_mission_node"
all imports OK
```

`colcon build`는 호스트에 colcon이 설치되어 있지 않아 수행하지 못했다. Docker 컨테이너(`ros2_humble`)에서 아래 절차로 빌드/실행 검증을 추가로 수행해야 한다.

```bash
docker exec -it ros2_humble bash
cd /ros2_ws/test_workspace/elevator_mission
colcon build --symlink-install
source install/setup.bash
ros2 launch elevator_sim_pkg elevator_sim_only.launch.py
# 다른 터미널
ros2 topic echo /elevator/state --once
ros2 topic pub --once /elevator/call std_msgs/msg/String "{data: F2}"
```

## 4. 수동 데모 결과

엘리베이터 좌표 캡처(Task 1.5 Step 3-4)와 F1→F2 mission end-to-end 데모는 실제 Gazebo/Nav2 세션이 필요하므로 본 세션에서는 수행하지 못했다. 다음 단계에서 다음 순서로 검증한다.

1. F1/F2/F3 각각 `gazebo.launch.py` + `kku_navigation.launch.py` 실행
2. `capture_nav_point.py`로 `elevator_entry`, `elevator_inside`, `elevator_exit`, `parcel_pickup`, `208` 좌표를 캡처해 `kku_nav_points.yaml` 갱신
3. `test_point_registry.py` 재실행으로 회귀 확인
4. `run_demo.sh` 안내대로 3개 터미널 + 사용자 ack 절차로 `parcel_to_208` mission 실행
5. mission 종료 로그(`MISSION COMPLETE`)와 RViz 상의 도착 확인

결과는 본 보고서 §4에 추가 기록한다.

## 5. 승격 판단

성공하면 다음 후보로 승격한다.

- `elevator_mission_pkg` → 기존 `src/drive_pkg` 또는 새 production package
- `elevator_sim_pkg` → 실제 엘리베이터 bridge로 교체 가능한 mock package
- `floor_orchestrator_pkg` → A3-lite teleport 구현으로 내부 교체

## 6. 한계/미해결

- **colcon build 미검증**: 호스트에 colcon 미설치. Docker 컨테이너 빌드 결과를 별도 기록 필요.
- **AMCL 캡처 미수행**: 좌표는 `kku_pre_simulation_map.yaml` 기준 seed 값. 실 mission 전 캡처 필수.
- **`py_trees` 미사용**: `package.xml`은 exec_depend로 선언했지만 실제 구현은 경량 자체 sequencer를 사용한다. py_trees / py_trees_ros 전환은 behavior 인터페이스(`tick`, `initialise`, `update`) 호환이므로 mission_tree 한 곳만 교체하면 된다.
- **WaitForAck mock**: parcel_loaded/delivered는 즉시 success. 추후 `/mission/ack` Trigger 서비스로 교체 가능.
