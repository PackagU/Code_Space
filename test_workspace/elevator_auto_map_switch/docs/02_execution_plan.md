# Auto Map Switch Execution Plan

작성일: 2026-06-02

## 목표

자동 맵 전환 PoC를 기존 수동 엘리베이터 mission PoC와 분리해서 준비한다. 오늘은 구현하지 않고, 다음 세션에서 실행할 순서와 검증 기준을 명확히 한다.

## 작업 원칙

- `test_workspace/elevator_mission/`은 legacy로 유지한다.
- 자동 맵 전환 관련 코드는 `test_workspace/elevator_auto_map_switch/` 아래에만 만든다.
- 기존 `src/` 패키지는 읽기만 한다.
- 기능 구현 전에는 dry-run 테스트부터 작성한다.
- 실제 Gazebo/Nav2 검증은 dry-run 통과 후 진행한다.

## 단계별 시행 계획

### 1단계: 서비스와 config 확인

컨테이너에서 Nav2가 어떤 map server 서비스를 노출하는지 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch slam_pkg kku_navigation.launch.py floor:=F1 rviz:=false
ros2 service list
ros2 service type /map_server/load_map
```

성공 기준:

- `/map_server/load_map` 또는 동등한 load map 서비스가 확인된다.
- 서비스 타입이 `nav2_msgs/srv/LoadMap`인지 확인된다.
- costmap clear 서비스명이 확인된다.

### 2단계: 새 config 설계

`config/floor_maps.yaml`에 층별 map yaml과 재초기화 point를 분리해 둔다.

예상 구조:

```yaml
schema_version: 1
frame_id: map
floors:
  F1:
    map_yaml: /ros2_ws/install/slam_pkg/share/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml
    initial_pose_id: elevator_exit
  F2:
    map_yaml: /ros2_ws/install/slam_pkg/share/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml
    initial_pose_id: elevator_exit
  F3:
    map_yaml: /ros2_ws/install/slam_pkg/share/slam_pkg/maps/kku_virtual/f3/kku_f3.yaml
    initial_pose_id: elevator_exit
```

성공 기준:

- 오프라인 테스트에서 F1/F2/F3 map yaml이 모두 resolve된다.
- 없는 층을 요청하면 명확한 에러가 난다.

### 3단계: 자동 orchestrator dry-run

새 `auto_floor_orchestrator_pkg`를 만든다. 초기 구현은 Nav2 서비스 호출 없이 상태머신만 검증한다.

상태 전이:

```text
idle
  -> request_switch
waiting_elevator
  -> elevator state current_floor == target_floor and door_state == open
loading_map
  -> dry-run success
ready
```

성공 기준:

- `/floor_orchestrator/request_switch` 호출 후 `pending=true`가 발행된다.
- mock `/elevator/state`에서 목표층 open 상태가 들어오면 `map_loaded=true`, `pending=false`가 발행된다.

### 4단계: Nav2 map load 연결

dry-run 상태머신이 통과하면 `nav2_msgs/srv/LoadMap` client를 붙인다.

호출 순서:

```text
elevator arrived
  -> call /map_server/load_map
  -> wait response
  -> clear global costmap
  -> clear local costmap
  -> publish ready status
```

성공 기준:

- map load response가 성공이면 status에 `map_loaded=true`가 포함된다.
- 실패하면 `phase=failed`, `error` 필드에 원인이 들어간다.
- 실패 상태에서는 mission이 다음 주행으로 넘어가지 않는다.

### 5단계: initialpose 연결

맵 전환 후 목표층 `elevator_exit` pose를 `/initialpose`로 발행한다.

성공 기준:

- `/initialpose`가 map load 이후에만 발행된다.
- pose의 frame_id는 `map`이다.
- yaw는 point registry 기준으로 quaternion 변환된다.

### 6단계: 최소 mission과 연결

legacy mission 전체를 복사하지 않고, 자동 전환만 검증하는 최소 mission을 만든다.

검증 sequence:

```text
CallElevator(F2)
RequestAutoSwitch(F2)
WaitMapReady(F2)
PublishInitialPose(elevator_exit@F2)
Done
```

성공 기준:

- 수동 `/floor_orchestrator/ack` 없이 sequence가 끝난다.
- status 로그에 목표층, map yaml, map_loaded, initialpose_sent가 남는다.

### 7단계: legacy mission과 호환성 검토

새 PoC가 통과하면 legacy mission의 `SwitchFloor` 계약과 호환되는지 검토한다.

성공 기준:

- legacy `SwitchFloor`가 기다리는 status 조건과 새 auto orchestrator의 status JSON이 충돌하지 않는다.
- 수동 legacy와 자동 PoC를 동시에 실행하지 않는 운영 규칙이 문서화된다.

## 오늘 하지 않는 것

- 실제 ROS2 package 생성
- Python node 구현
- Nav2 service client 구현
- legacy folder 수정
- production `src/` 승격

## 다음 세션 첫 명령

```bash
cd /home/hsm/2026_graduation_project
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
python3 test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
python3 test_workspace/elevator_mission/scripts/test_navigate_route_behavior.py
python3 test_workspace/elevator_mission/scripts/test_orthogonal_router.py
python3 test_workspace/elevator_mission/scripts/test_nav2_orthogonal_tuning.py
```

위 회귀가 통과하면 자동 맵 전환 PoC 구현을 시작한다.
