# KKU 저장 맵 기반 Point Goal 자율주행 개발 절차

작성일: 2026-05-27

## 1. 목표

F1, F2, F3 맵 저장이 끝난 뒤의 다음 단계는 "로봇이 방 번호나 택배 보관함 같은 이름을 입력받아, 저장된 맵 위에서 해당 point까지 이동하는 것"이다.

여기서 말하는 자율주행의 범위는 다음처럼 나눈다.

- 사용자는 `201`, `208`, `301`, `parcel_pickup`, `elevator_exit` 같은 goal id를 준다.
- 시스템은 goal id를 실제 좌표 `(floor, x, y, yaw)`로 변환한다.
- 같은 층에서는 Nav2가 저장 맵, AMCL 위치추정, costmap을 사용해 경로를 만들고 장애물을 피하며 이동한다.
- 층이 다르면 우선 현재 층 엘리베이터 point까지 이동하고, 엘리베이터 내부 동작과 층 전환은 별도 상태머신으로 처리한다.
- 엘리베이터 내부의 정교한 동작은 나중 단계로 미룬다. 지금은 "엘리베이터 앞/안 point까지 이동 가능"을 먼저 만든다.

즉, 전체 시스템은 자율주행이지만 처음부터 모든 것을 AI처럼 풀지 않는다. 방 번호를 point로 해석하는 부분은 우리가 명시적으로 만들고, 복도 주행/회피/도착 제어는 Nav2에 맡긴다.

## 2. 현재 준비된 것

현재 repo 기준으로 이미 준비된 자산:

| 항목 | 위치 | 상태 |
|------|------|------|
| F1 저장 맵 | `src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml` | 완료 |
| F2 저장 맵 | `src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml` | 완료 |
| F3 저장 맵 | `src/slam_pkg/maps/kku_virtual/f3/kku_f3.yaml` | 완료 |
| 가상 건물/방/택배존 설계 좌표 | `src/common_pkg/config/kku_pre_simulation_map.yaml` | 완료 |
| Gazebo 층별 실행 | `ros2 launch common_pkg gazebo.launch.py floor:=F1` | 완료 |
| SLAM 통합 실행 | `ros2 launch slam_pkg kku_simulation.launch.py floor:=F1` | 완료 |
| WASD teleop | `ros2 run drive_pkg keyboard_teleop` | 완료 |

아직 필요한 것:

| 항목 | 권장 위치 | 역할 |
|------|-----------|------|
| Nav2 params | `src/slam_pkg/config/nav2_params.yaml` | AMCL, planner, controller, costmap 설정 |
| Nav2 launch | `src/slam_pkg/launch/kku_navigation.launch.py` | 저장 맵 + Gazebo + Nav2 실행 |
| point registry | `src/common_pkg/config/kku_nav_points.yaml` | 방/엘리베이터/택배존 id와 좌표 연결 |
| goal sender | `src/drive_pkg/drive_pkg/point_goal_client.py` | `208` 같은 id를 Nav2 goal로 전송 |
| mission state machine | 이후 추가 | 택배존 -> 엘리베이터 -> 방 앞 배송 흐름 |

## 3. 전체 구조

처음 구현할 구조는 아래처럼 잡는다.

```text
사용자 입력
  예: 208
    |
    v
Point Goal Client
  208 -> {floor: F2, pose: room_208_approach}
    |
    v
Floor Manager
  현재 층과 목표 층 비교
    |
    +-- 같은 층: Nav2 NavigateToPose 실행
    |
    +-- 다른 층:
          1. 현재 층 elevator_entry로 이동
          2. 엘리베이터 상태머신 실행
          3. 목표 층 맵으로 Nav2 재시작 또는 재초기화
          4. 목표 층 elevator_exit에서 위치 초기화
          5. 방 앞 point로 이동
```

M1/M2에서는 같은 층 주행을 먼저 완성한다. 층간 이동은 "세계 전환/로봇 respawn 기반 논리 전환"으로 시작하고, 실제 엘리베이터 내부 제어는 나중에 붙인다.

## 4. Point 설계

### 4.1 Point의 의미

point는 로봇이 실제로 도착해야 하는 pose이다. 방 문의 중심이 아니라, 로봇이 문 앞에서 멈출 수 있는 접근 pose를 goal로 둔다.

예:

- `201`: 201호 문 앞 접근 pose
- `208`: 208호 문 앞 접근 pose
- `parcel_pickup`: 1층 택배 보관함 앞 pose
- `elevator_entry`: 엘리베이터에 들어가기 전 복도 pose
- `elevator_inside`: 엘리베이터 내부 정렬 pose
- `elevator_exit`: 엘리베이터에서 나온 직후 pose

### 4.2 초기 point 후보

`src/common_pkg/config/kku_pre_simulation_map.yaml`에는 이미 설계 좌표가 있다.

F1:

| id | 의미 | 설계 pose |
|----|------|-----------|
| `parcel_access` | 택배존 입구 | `(5.0, -0.6, -90 deg)` |
| `parcel_pickup` | 택배 보관함 앞 | `(5.0, -1.4, -90 deg)` |
| `elevator_inside` | 엘리베이터 내부 중앙 | `(0.0, 0.0, 0 deg)` |
| `elevator_exit` | 엘리베이터 문 바깥 | `(1.6, 0.0, 0 deg)` |

F2:

| id | 설계 pose |
|----|-----------|
| `201` | `(1.65, 3.0, 180 deg)` |
| `202` | `(2.35, 3.0, 0 deg)` |
| `203` | `(1.65, 6.0, 180 deg)` |
| `204` | `(2.35, 6.0, 0 deg)` |
| `205` | `(1.65, 9.0, 180 deg)` |
| `206` | `(2.35, 9.0, 0 deg)` |
| `207` | `(1.65, 12.0, 180 deg)` |
| `208` | `(2.35, 12.0, 0 deg)` |

F3:

| id | 설계 pose |
|----|-----------|
| `301` | `(1.65, 3.0, 180 deg)` |
| `302` | `(2.35, 3.0, 0 deg)` |
| `303` | `(1.65, 6.0, 180 deg)` |
| `304` | `(2.35, 6.0, 0 deg)` |
| `305` | `(1.65, 9.0, 180 deg)` |
| `306` | `(2.35, 9.0, 0 deg)` |
| `307` | `(1.65, 12.0, 180 deg)` |
| `308` | `(2.35, 12.0, 0 deg)` |

주의: 이 값은 Gazebo world를 만들 때 사용한 설계 좌표다. SLAM으로 저장한 맵의 `map` frame 좌표와 완전히 같다고 바로 가정하면 안 된다. 맵 저장 과정에서 SLAM의 `map` origin이 달라질 수 있으므로, 첫 Nav2 테스트 때 RViz에서 point 좌표를 확인하고 `kku_nav_points.yaml`에 확정값을 저장해야 한다.

### 4.3 권장 point registry 형식

추가할 파일 후보:

`src/common_pkg/config/kku_nav_points.yaml`

```yaml
schema_version: 1
frame_id: map

floors:
  F1:
    map_yaml: src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml
    initial_pose:
      id: elevator_exit
      x: 1.6
      y: 0.0
      yaw_deg: 0.0
    points:
      elevator_exit:
        type: elevator
        x: 1.6
        y: 0.0
        yaw_deg: 0.0
      elevator_inside:
        type: elevator
        x: 0.0
        y: 0.0
        yaw_deg: 0.0
      parcel_pickup:
        type: parcel
        x: 5.0
        y: -1.4
        yaw_deg: -90.0

  F2:
    map_yaml: src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml
    initial_pose:
      id: elevator_exit
      x: 1.6
      y: 0.0
      yaw_deg: 0.0
    points:
      "201":
        type: room
        x: 1.65
        y: 3.0
        yaw_deg: 180.0
      "208":
        type: room
        x: 2.35
        y: 12.0
        yaw_deg: 0.0
```

처음에는 설계 좌표를 넣고, RViz/Nav2 테스트 후 실제 도착이 좋은 좌표로 보정한다.

## 5. 개발 단계

### 5.1 1단계: Nav2가 설치되어 있는지 확인

컨테이너 attach 터미널에서:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 pkg list | grep nav2_bringup
```

아무것도 안 나오면 Nav2 설치:

```bash
apt update
apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

### 5.2 2단계: 저장 맵으로 한 층 Nav2 실행

SLAM을 다시 켜는 것이 아니라, 저장된 맵을 map server가 읽고 AMCL이 위치추정을 한다.

권장 실행 구조:

터미널 1: Gazebo만 실행

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
```

터미널 2: Nav2 실행

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
```

위 명령은 `kku_navigation.launch.py`가 준비된 뒤의 목표 실행 명령이다.
파일 생성 방법은 [../03_nav2_one_floor/02_create_kku_navigation_launch.md](../03_nav2_one_floor/02_create_kku_navigation_launch.md)에 따로 정리했다.

그 launch는 내부에서 아래를 처리해야 한다.

- floor 인자로 F1/F2/F3 선택
- 선택된 `kku_f*.yaml` 맵을 map server에 전달
- `use_sim_time:=true`
- AMCL 실행
- planner/controller/bt_navigator/lifecycle_manager 실행
- RViz2 선택 실행

### 5.3 3단계: RViz에서 수동 goal 테스트

첫 성공 기준은 goal client를 만들기 전에 RViz에서 직접 goal을 찍어 로봇이 이동하는 것이다.

절차:

1. RViz에서 Map, TF, LaserScan, Global Costmap, Local Costmap을 표시한다.
2. `2D Pose Estimate`로 로봇 초기 위치를 elevator_exit 근처에 찍는다.
3. `Nav2 Goal`로 같은 층 복도 끝 또는 방 앞을 찍는다.
4. 로봇이 경로를 만들고 움직이는지 본다.
5. 도착하면 `/odom`, `/tf`, local costmap이 끊기지 않았는지 확인한다.

확인 명령:

```bash
ros2 topic echo /amcl_pose --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /tf --once
ros2 action list | grep navigate
```

성공 기준:

- `map -> odom -> base_footprint -> base_link -> laser` TF가 이어진다.
- RViz에서 global path가 나온다.
- `/cmd_vel`이 Nav2에서 발행된다.
- 로봇이 goal 근처에서 멈춘다.

### 5.4 4단계: point 좌표 확정

처음에는 `kku_pre_simulation_map.yaml`의 설계 좌표를 사용해도 되지만, 저장 맵 기준 좌표는 반드시 확인한다.

권장 방법:

1. RViz에서 방 앞에 `Nav2 Goal`을 찍는다.
2. 로봇이 실제로 멈춘 위치가 자연스러운지 본다.
3. 너무 벽에 붙거나 문과 어긋나면 goal을 다시 찍는다.
4. 확정된 pose를 `kku_nav_points.yaml`에 저장한다.
5. `201`, `202`, `parcel_pickup`, `elevator_exit`처럼 id로 관리한다.

좌표를 확인할 때 쓸 수 있는 토픽:

```bash
ros2 topic echo /goal_pose
ros2 topic echo /amcl_pose
```

RViz 설정에 따라 goal 토픽 이름이 다를 수 있다. 토픽 목록에서 `goal`이 들어간 토픽을 확인한다.

```bash
ros2 topic list | grep goal
```

### 5.5 5단계: goal id 입력으로 이동

다음으로 만들 노드는 이런 형태가 좋다.

```bash
ros2 run drive_pkg point_goal_client --ros-args -p goal_id:=208
```

동작:

1. `kku_nav_points.yaml`을 읽는다.
2. `208`이 F2의 room point임을 찾는다.
3. 현재 층이 F2면 Nav2 `NavigateToPose` action으로 바로 보낸다.
4. 현재 층이 F1/F3면 elevator flow로 넘긴다.

처음 구현은 같은 층만 지원해도 된다.

```text
지원:
  현재 F2, goal_id=208 -> 이동
  현재 F3, goal_id=307 -> 이동

아직 미지원:
  현재 F1, goal_id=208 -> 엘리베이터 상태머신 필요
```

### 5.6 6단계: 장애물 회피 테스트

장애물 회피는 두 종류로 나눠서 테스트한다.

| 종류 | 예시 | Nav2 처리 |
|------|------|-----------|
| 맵에 이미 있는 장애물 | 벽, 방, 택배존 외벽 | global costmap / planner |
| 맵에 없지만 센서에 보이는 장애물 | 복도에 갑자기 놓은 박스 | local costmap / controller |

테스트 순서:

1. 장애물 없는 상태에서 `elevator_exit -> 208` 이동 성공.
2. 복도 중앙에 작은 box model을 추가하고 같은 goal 반복.
3. global path가 그대로여도 local planner가 가까운 장애물을 피하는지 확인.
4. 피하지 못하면 local costmap inflation, obstacle layer, robot footprint를 조정.
5. 벽에 너무 붙으면 inflation radius를 키운다.
6. 좁은 구간을 못 지나가면 footprint나 costmap resolution을 점검한다.

장애물 회피 튜닝에서 가장 먼저 볼 값:

```yaml
robot_radius 또는 footprint
inflation_radius
cost_scaling_factor
obstacle_layer.scan.topic
controller_server FollowPath plugin
max_vel_x
max_vel_theta
```

## 6. 시뮬레이션 시나리오

### 6.1 같은 층 기본 시나리오

가장 먼저 해야 할 테스트:

```text
F2 elevator_exit -> 201
F2 elevator_exit -> 208
F3 elevator_exit -> 301
F3 elevator_exit -> 308
F1 elevator_exit -> parcel_pickup
```

각 시나리오에서 기록할 것:

- goal id
- 시작 floor
- 목표 point
- 성공/실패
- 실패 시 마지막 위치
- RViz global path가 나왔는지
- local costmap이 장애물을 봤는지
- `/cmd_vel`이 계속 나왔는지

### 6.2 택배 배송 데모 시나리오

엘리베이터 내부는 나중에 하더라도, 최종 데모 흐름은 아래처럼 잡는다.

```text
1. F1 elevator_exit에서 시작
2. parcel_pickup으로 이동
3. 택배 적재 완료 이벤트 처리
4. F1 elevator_entry로 이동
5. 엘리베이터 탑승/층 이동 상태머신 실행
6. F2 또는 F3 world로 전환
7. 목표 층 elevator_exit에서 초기 pose 설정
8. 방 번호 point로 이동
9. 배송 완료 이벤트 처리
```

처음에는 5번을 실제 엘리베이터 물리 동작 대신 "목표 층 Gazebo 재실행/robot respawn"으로 처리한다.

## 7. 구현 우선순위

추천 순서:

1. `src/slam_pkg/config/nav2_params.yaml` 추가
2. `src/slam_pkg/launch/kku_navigation.launch.py` 추가
3. 저장 맵 F2 하나로 RViz `Nav2 Goal` 테스트
4. F1/F2/F3 floor 인자 전환 테스트
5. `src/common_pkg/config/kku_nav_points.yaml` 추가
6. `point_goal_client.py`로 같은 층 goal id 이동
7. 장애물 회피 튜닝
8. `floor_manager` 또는 `delivery_mission_node`로 층간 흐름 연결

이 순서가 좋은 이유는, Nav2 기본 주행이 안 되는 상태에서 point client나 엘리베이터 로직을 먼저 만들면 어디가 문제인지 분리하기 어렵기 때문이다.

## 8. 실패할 때 보는 순서

### 8.1 로봇이 아예 안 움직임

확인:

```bash
ros2 topic echo /cmd_vel --once
ros2 topic info /cmd_vel -v
ros2 topic echo /odom --once
ros2 topic echo /clock --once
```

- `/cmd_vel`이 안 나오면 Nav2 action/controller 문제.
- `/cmd_vel`은 나오는데 `/odom`이 안 바뀌면 Gazebo/diff_drive 문제.
- `/clock`이 안 나오면 Gazebo가 멈췄거나 launch가 깨진 것.

### 8.2 RViz에서 path가 안 나옴

확인:

```bash
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /tf --once
```

- `/map`이 없으면 map server 문제.
- `/amcl_pose`가 없으면 AMCL/initial pose 문제.
- TF가 끊기면 `map -> odom -> base_footprint` 연결 문제.

### 8.3 goal을 찍으면 바로 실패함

가능성이 큰 원인:

- goal이 벽이나 unknown 영역에 찍힘.
- footprint가 복도 폭보다 너무 크게 잡힘.
- costmap inflation이 너무 커서 통로가 막힌 것으로 판단됨.
- initial pose가 실제 로봇 위치와 다름.

### 8.4 장애물을 못 피함

확인:

```bash
ros2 topic echo /scan --once
ros2 topic list | grep costmap
```

- LiDAR가 장애물을 보고 있는지 먼저 확인.
- local costmap에 obstacle layer가 켜져 있는지 확인.
- `/scan` topic 이름이 Nav2 params와 일치하는지 확인.

## 9. 이번 단계의 완료 기준

이 문서 기준 첫 번째 완료 기준:

- F2 저장 맵으로 Nav2가 실행된다.
- RViz에서 `2D Pose Estimate` 후 `Nav2 Goal`을 찍으면 로봇이 방 앞까지 이동한다.
- `/cmd_vel`, `/odom`, `/amcl_pose`, `/scan`, `/map`이 정상이다.
- `201`, `208` 중 최소 하나를 point id로 보내 이동할 수 있다.
- 복도에 임시 장애물을 둬도 충돌하지 않고 멈추거나 우회한다.

그 다음 완료 기준:

- F1 `parcel_pickup` point 이동 성공.
- F2 `201~208` point 이동 성공.
- F3 `301~308` point 이동 성공.
- 같은 코드에서 `floor:=F1|F2|F3`를 바꿔 테스트 가능.
- 층간 배송 시나리오에서 엘리베이터 전/후 point까지는 자동으로 이동한다.

## 10. 요약

이제부터 할 일은 "맵을 더 만드는 것"이 아니라 "저장된 맵을 사용해 goal id 기반 이동을 만드는 것"이다.

핵심은 세 파일이다.

```text
src/slam_pkg/launch/kku_navigation.launch.py
src/slam_pkg/config/nav2_params.yaml
src/common_pkg/config/kku_nav_points.yaml
```

이 세 개가 잡히면, 그 다음에 `point_goal_client.py`를 붙여서 `208` 같은 방 번호를 바로 자율주행 goal로 바꿀 수 있다. 엘리베이터는 같은 방식으로 `elevator_entry`, `elevator_inside`, `elevator_exit` point들을 지나가는 상태머신으로 확장하면 된다.
