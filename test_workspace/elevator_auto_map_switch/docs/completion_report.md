# Auto Map Switch PoC Completion Report

작성일: 2026-06-12

## 1. 구현 범위

- test workspace: `test_workspace/elevator_auto_map_switch/`
- legacy `test_workspace/elevator_mission/` 수정 여부: 없음 (읽기만 함, 회귀 5종 재확인 PASS)
- 기존 `src/` 패키지 수정 여부: 없음
- custom msg/srv: 없음 (`std_msgs/String`, `std_srvs/Trigger`, `nav2_msgs/srv/LoadMap`, `nav2_msgs/srv/ClearEntireCostmap`, `geometry_msgs/PoseWithCovarianceStamped`)
- 판단 근거 기록: 워크스페이스 루트 `FeedBack/` 폴더 (파일별 이유 문서)

## 2. 생성 파일

| 파일 | 역할 |
|------|------|
| `config/floor_maps.yaml` | 층별 map yaml + initialpose용 포즈(elevator_inside/exit) |
| `src/auto_floor_orchestrator_pkg/auto_floor_orchestrator_pkg/floor_map_registry.py` | YAML 파서 + 3단계 후보 경로 resolve |
| `src/auto_floor_orchestrator_pkg/auto_floor_orchestrator_pkg/auto_switch_core.py` | ROS 무의존 상태머신 (idle → waiting → loading → finalizing → ready / failed) |
| `src/auto_floor_orchestrator_pkg/auto_floor_orchestrator_pkg/auto_floor_orchestrator_node.py` | ROS 노드. 노드명 `floor_orchestrator_node` (legacy 호환) |
| `src/auto_floor_orchestrator_pkg/launch/auto_floor_orchestrator.launch.py` | launch + 파라미터 인자 |
| `src/auto_floor_orchestrator_pkg/{package.xml,setup.py,setup.cfg,resource/}` | ament_python 패키지 메타 |
| `scripts/test_floor_map_registry.py` | L0 오프라인 테스트 |
| `scripts/test_auto_switch_state_machine.py` | L1 오프라인 테스트 |
| `scripts/test_switch_floor_compatibility.py` | L4 오프라인 호환성 테스트 (legacy 소스 고정 검사 포함) |
| `scripts/test_orchestrator_ros_smoke.py` | L2 인프로세스 ROS smoke 테스트 |
| `scripts/probe_nav2_services.sh` | L3 사전 Nav2 서비스명/타입 확인 |

## 3. 핵심 설계 결정

- **노드명은 `floor_orchestrator_node` 그대로**: legacy `SwitchFloor`가 `/floor_orchestrator_node/set_parameters`를 하드코딩하므로, 노드명이 다르면 mission이 첫 단계에서 영원히 대기한다. 같은 이름을 쓰는 대신 legacy 수동 orchestrator와 동시 실행은 금지한다.
- **`pending=false`는 map load 성공 + 마무리(costmap clear, initialpose) 완료 후에만**: 실패 시 `phase=failed`, `error` 기록, `pending=true` 유지 → mission은 이전 층 맵으로 주행을 재개하지 못한다.
- **`/initialpose` 발행 주체는 orchestrator**: map load 성공 시점을 아는 유일한 주체라 "load 후 발행" 순서가 구조적으로 보장된다. `publish_initialpose:=false`로 끌 수 있다. 포즈는 `spawn_point_id`(기본 `elevator_inside`) 기준 — 맵이 바뀌는 순간 로봇은 물리적으로 엘리베이터 안에 있기 때문이다.
- **map 경로 resolve 3단계**: 절대경로 → workspace 상대(`<root>/src/...`) → ament share(`install/slam_pkg/share/slam_pkg/...`). 첫 번째로 존재하는 파일 사용, 전부 실패하면 시도 목록과 함께 에러.
- **costmap clear 서비스명 파라미터화**: 기본값 `/global_costmap/clear_entirely_global_costmap`, `/local_costmap/clear_entirely_local_costmap`. clear 실패는 경고 후 계속(맵은 이미 로드됨), load 실패는 치명적.

## 4. 검증 결과

```text
host + container 공통:
PASS floor map registry
PASS auto switch state machine
PASS switch floor compatibility
PASS orchestrator ros smoke      (ROS_DOMAIN_ID=89, dry-run)

container:
colcon build --symlink-install   → Finished <<< auto_floor_orchestrator_pkg
ros2 pkg list                    → auto_floor_orchestrator_pkg
ros2 run ... (SIGINT 종료)       → traceback 없이 종료

legacy 회귀 (수정 없음 확인):
PASS point registry / behavior dry-run / navigate route behavior /
orthogonal router / nav2 orthogonal tuning
```

## 5. L3 / L5 실행 절차 (다음 세션, 사람 개입은 실행 명령뿐)

L3 (실제 Nav2 map load):

```bash
# 터미널 1: Gazebo F1 + Nav2 F1
ros2 launch common_pkg gazebo.launch.py floor:=F1 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F1 rviz:=true

# 터미널 2: 서비스명 확인 후 orchestrator 실행
bash test_workspace/elevator_auto_map_switch/scripts/probe_nav2_services.sh
ros2 launch auto_floor_orchestrator_pkg auto_floor_orchestrator.launch.py \
  dry_run_map_load:=false target_floor:=F2 use_sim_time:=true

# 터미널 3: 전환 트리거 (elevator sim이 없을 때의 수동 주입)
ros2 service call /floor_orchestrator/request_switch std_srvs/srv/Trigger
# 주의: --once 는 DDS discovery 레이스로 메시지가 버려질 수 있다. 여러 번 발행한다.
ros2 topic pub --times 10 --rate 2 /elevator/state std_msgs/msg/String \
  "{data: '{\"current_floor\":\"F2\",\"target_floor\":\"F2\",\"door_state\":\"open\",\"state\":\"ARRIVED_OPEN\"}'}"
ros2 topic echo /floor_orchestrator/status --once
```

L5 (legacy mission 무수정 + 자동 전환, ack 없이 끝까지):

```bash
# legacy workspace와 새 workspace를 모두 빌드/소스 (컨테이너)
cd /ros2_ws/test_workspace/elevator_mission && colcon build --symlink-install
cd /ros2_ws/test_workspace/elevator_auto_map_switch && colcon build --symlink-install
source /ros2_ws/test_workspace/elevator_mission/install/setup.bash
source /ros2_ws/test_workspace/elevator_auto_map_switch/install/setup.bash

# 터미널 1: Gazebo F1 / 터미널 2: Nav2 F1 (위 L3와 동일)
# 터미널 3: elevator sim (legacy 그대로)
ros2 run elevator_sim_pkg elevator_sim_node
# 터미널 4: 자동 orchestrator — legacy floor_orchestrator_node 대신 실행 (동시 실행 금지)
ros2 launch auto_floor_orchestrator_pkg auto_floor_orchestrator.launch.py \
  dry_run_map_load:=false use_sim_time:=true
# 터미널 5: legacy mission 그대로
ros2 run elevator_mission_pkg delivery_mission_node --ros-args -p mission_id:=parcel_to_208
```

성공 기준: `/floor_orchestrator/ack` 호출 없이 mission 로그가 `SwitchFloor ... acknowledged`를 지나 `MISSION COMPLETE`까지 도달, RViz 맵이 F2로 교체.

## 6. 한계 / 미해결

- **L3/L5 미수행**: 실제 Nav2 map load와 E2E 주행은 Gazebo 세션이 필요해 본 세션에서는 dry-run까지만 검증했다.
- **Gazebo world는 안 바뀜**: 이 PoC는 Nav2 쪽 맵 전환만 자동화한다. 층간 world swap/robot respawn은 별도 범위 (01_current_analysis.md 리스크 표 참조).
- **legacy `Relocalize`의 elevator_exit 발행**: legacy mission과 같이 돌리면 orchestrator가 `elevator_inside`로 정확한 initialpose를 먼저 쏘고, legacy가 직후 `elevator_exit`(1.6m 차이)로 덮어쓴다. legacy 단독 실행과 동일한 동작이라 회귀는 아니지만, E2E에서 AMCL 수렴 떨림이 보이면 `publish_initialpose` 관련 FeedBack 문서 참조.
- **실제 엘리베이터 브리지**: 문이 닫히기 전에 load가 끝나야 한다는 타이밍 제약은 mock에서는 없음. 실물 연동 시 도착 이벤트 latch가 필요할 수 있다.
