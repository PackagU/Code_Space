# Auto Map Switch Verification Plan

작성일: 2026-06-02

## 목적

자동 맵 전환 PoC 구현 후 무엇을 통과해야 성공으로 볼지 정한다. 오늘은 검증 실행을 하지 않고, 다음 구현 세션의 기준만 남긴다.

## 검증 레벨

| 레벨 | 이름 | 목적 |
|------|------|------|
| L0 | Offline parser test | config와 registry가 올바른지 확인 |
| L1 | Offline state machine test | 엘베 도착 이벤트가 자동 switch ready로 이어지는지 확인 |
| L2 | ROS dry-run smoke | ROS topic/service 환경에서 ack 없이 ready가 되는지 확인 |
| L3 | Nav2 map service smoke | 실제 `/map_server/load_map` 호출 성공 확인 |
| L4 | Mission compatibility | mission behavior가 `map_loaded=true` 상태에서 다음 단계로 넘어가는지 확인 |
| L5 | End-to-end simulation | Gazebo/Nav2에서 F1 탑승 후 F2/F3 맵 전환과 주행 확인 |

## L0 Offline Parser Test

명령:

```bash
python3 test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py
```

기대:

```text
PASS floor map registry
```

실패 시 확인:

- `config/floor_maps.yaml`의 floor key
- map yaml 파일명
- workspace 상대경로 기준

## L1 Offline State Machine Test

명령:

```bash
python3 test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py
```

기대:

```text
PASS auto switch state machine
```

핵심 검증:

- 목표층이 아닌 elevator state에서는 계속 pending
- 목표층과 문 열림 상태가 동시에 만족되면 ready
- dry-run에서는 map_loaded가 true

## L2 ROS Dry-Run Smoke

터미널 1:

```bash
cd /ros2_ws/test_workspace/elevator_auto_map_switch
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run auto_floor_orchestrator_pkg auto_floor_orchestrator_node \
  --ros-args -p dry_run_map_load:=true
```

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/test_workspace/elevator_auto_map_switch/install/setup.bash
ros2 service call /floor_orchestrator/request_switch std_srvs/srv/Trigger
# 주의: --once 는 DDS discovery 레이스로 메시지가 버려질 수 있다. 여러 번 발행한다.
ros2 topic pub --times 10 --rate 2 /elevator/state std_msgs/msg/String \
  "{data: '{\"current_floor\":\"F2\",\"target_floor\":\"F2\",\"door_state\":\"open\",\"state\":\"ARRIVED_OPEN\"}'}"
ros2 topic echo /floor_orchestrator/status --once
```

기대:

```json
{"current_floor":"F2","pending":false,"phase":"ready","map_loaded":true}
```

## L3 Nav2 Map Service Smoke

전제:

- Gazebo와 Nav2가 F1에서 실행 중이다.
- `/map_server/load_map` 서비스가 존재한다.

명령:

```bash
ros2 service list | grep load_map
ros2 service type /map_server/load_map
```

기대:

```text
nav2_msgs/srv/LoadMap
```

자동 orchestrator 실행:

```bash
ros2 run auto_floor_orchestrator_pkg auto_floor_orchestrator_node \
  --ros-args \
  -p dry_run_map_load:=false \
  -p target_floor:=F2 \
  -p load_map_service:=/map_server/load_map
```

성공 기준:

- elevator arrival 이벤트 이후 load map response가 성공
- `/map` topic이 새 맵으로 갱신
- status에 `map_loaded=true`

## L4 Mission Compatibility

목표:

legacy mission의 `SwitchFloor`가 기대하는 status 조건과 새 auto orchestrator status가 호환되는지 확인한다.

성공 기준:

- `pending=false`
- `current_floor=target_floor`
- `map_loaded=true`
- `phase=ready`
- 수동 `/floor_orchestrator/ack` 호출 없이 다음 단계로 진행

## L5 End-to-End Simulation

최종 목표 흐름:

```text
F1 parcel_pickup
  -> F1 elevator_entry
  -> F1 elevator_inside
  -> elevator sim moves to F2
  -> auto map switch to F2 map
  -> initialpose at elevator_exit@F2
  -> F2 destination, e.g. 208
```

성공 기준:

- mission log에 map switch ready가 기록된다.
- RViz에서 map이 목표층 맵으로 바뀐다.
- initialpose 이후 Nav2가 목표층 경로를 생성한다.
- 목적지 도착 로그가 나온다.

## 검증 결과 (2026-06-12)

구현 세션에서 아래 결과를 기록한다. 상세 내용은 `completion_report.md` 참조.

| 레벨 | 결과 | 수행 환경 | 비고 |
|------|------|----------|------|
| L0 | PASS | host + container | `test_floor_map_registry.py` — 경로 resolve 3단계 후보 검증 포함 |
| L1 | PASS | host + container | `test_auto_switch_state_machine.py` — 실패 시 `pending=true` 유지 검증 포함 |
| L2 | PASS | host + container | `test_orchestrator_ros_smoke.py` — legacy와 동일한 `set_parameters` → `request_switch` 순서 재현 |
| L3 | PASS | container (Gazebo F1 + Nav2 F1) | 2026-06-25 실측. 실제 `/map_server/load_map` 성공, `/map` F1→F2 교체, status `phase=ready/pending=false`. 아래 "검증 결과 (2026-06-25, L3)" 참조 |
| L4 | PASS (오프라인) | host + container | `test_switch_floor_compatibility.py` — legacy `SwitchFloor` 게이트 조건 + legacy 소스 고정 검사 |
| L5 | 미실행 | - | 시뮬 E2E. 실행 절차는 `completion_report.md` 참조 |

빌드 검증:

```text
container$ colcon build --symlink-install
Finished <<< auto_floor_orchestrator_pkg [0.60s]
container$ ros2 pkg list | grep auto_floor_orchestrator_pkg
auto_floor_orchestrator_pkg
```

L2 비고: 검증 계획의 수동 터미널 절차 대신, 같은 내용을 자동화한 인프로세스 smoke 테스트로 수행했다. 수동 절차도 여전히 유효하다.

## 검증 결과 (2026-06-25, L3)

환경: `ros2_humble` 컨테이너 (ghcr.io/packagu/ros2-humble-slam:humble), 호스트 DISPLAY=:1.

실행: Gazebo F1(`common_pkg gazebo.launch.py floor:=F1 use_sim_time:=true`) + Nav2 F1
(`slam_pkg kku_navigation.launch.py floor:=F1 rviz:=false use_sim_time:=true`) +
orchestrator(`auto_floor_orchestrator.launch.py dry_run_map_load:=false target_floor:=F2 use_sim_time:=true`).

서비스명 확인(`probe_nav2_services.sh`): 세 기본값이 모두 실측과 일치, launch 인자 교정 불필요.

- `/map_server/load_map` -> nav2_msgs/srv/LoadMap
- `/global_costmap/clear_entirely_global_costmap` -> nav2_msgs/srv/ClearEntireCostmap
- `/local_costmap/clear_entirely_local_costmap` -> nav2_msgs/srv/ClearEntireCostmap

전환 트리거: `request_switch` (armed target=F2) + `/elevator/state` F2 ARRIVED_OPEN 주입.

결과:

| 항목 | 트리거 전 | 트리거 후 |
|------|-----------|-----------|
| status phase | (idle) | ready |
| status pending | true | false |
| current_floor | F1 | F2 |
| map_loaded | false | true |
| /map 크기 | 334x264 | 498x348 |
| /map origin | [-1.26, -4.76] | [-2.46, -2.96] |

(맵 치수는 복도 5m 확장 후 값 — 정확한 현재값은 `src/slam_pkg/maps/kku_virtual/f*/kku_f*.yaml`
과 `test_workspace/gazebo_world_swap/scripts/map_expectations.py` 가 단일 소스다.)

orchestrator 로그 순서: `map loaded: .../kku_f2.yaml` -> `costmap cleared`(global, local) ->
`initialpose published: F2:elevator_inside (0.0, 0.0)` -> `floor switch READY: current_floor=F2`.

최종 status:

```json
{"current_floor": "F2", "error": "", "initialpose_sent": true, "map_loaded": true, "map_yaml": "/ros2_ws/src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml", "mode": "auto_map_switch", "pending": false, "phase": "ready", "spawn_point_id": "elevator_inside", "target_floor": "F2"}
```

범위 밖(예상된 노이즈): Gazebo world는 F1 그대로라 AMCL이 map->odom TF/initialpose 미수렴
경고를 냈고, gzclient/RViz는 GL 드라이버(nouveau dri3) 문제로 렌더 실패. 맵 교체와 status
전환 자체에는 영향 없음 — `/map` 토픽으로 객관 검증.

## 실패 로그 기록 규칙

실패 시 아래 형식으로 `completion_report.md` 또는 별도 debug log에 남긴다.

```text
date: 2026-06-XX
level: L3
command: ros2 run ...
expected: map_loaded=true
actual: load_map service unavailable
next: service name probe
```
