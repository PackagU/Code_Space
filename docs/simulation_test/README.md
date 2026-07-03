# Simulation Test 문서 인덱스

이 폴더는 KKU Gazebo 시뮬레이션을 기준으로, 맵 생성부터 저장 맵 기반 Nav2 자율주행까지 이어지는 문서만 모아둔 곳이다.

## 먼저 읽을 것

현재 상황에 따라 아래에서 바로 들어가면 된다.

| 지금 하려는 일 | 읽을 문서 |
|---|---|
| Docker/VSCode attach부터 헷갈림 | [01_environment](01_environment/README.md) |
| Gazebo에서 F1/F2/F3를 돌리고 SLAM 맵 저장 | [02_gazebo_slam_mapping](02_gazebo_slam_mapping/README.md) |
| 저장된 맵으로 한 층 Nav2 실행 | [03_nav2_one_floor](03_nav2_one_floor/README.md) |
| 방 번호/택배함/elevator point를 goal로 보내기 | [04_point_goal_delivery](04_point_goal_delivery/README.md) |

## 전체 단계

```text
1. 컨테이너 attach / ROS2 workspace 준비
2. Gazebo + SLAM Toolbox 실행
3. WASD teleop으로 층별 맵 작성
4. F1/F2/F3 map 저장
5. SLAM을 끄고 저장 맵 + map_server + AMCL + Nav2 실행
6. RViz에서 수동 goal 테스트
7. point id(room_201 등)를 goal pose로 변환
8. 장애물 회피와 배송 시나리오 테스트
```

## 중요한 구분

SLAM 모드는 맵을 새로 만드는 모드다.

```bash
ros2 launch slam_pkg kku_simulation.launch.py floor:=F2
```

Nav2 모드는 이미 저장한 맵을 읽고 그 위에서 이동하는 모드다.

```bash
ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
```

두 번째 명령의 `kku_navigation.launch.py`는 `src/slam_pkg/launch/`에 있어야 한다. 파일을 새로 만들거나 확인하는 법은 [03_nav2_one_floor/02_create_kku_navigation_launch.md](03_nav2_one_floor/02_create_kku_navigation_launch.md)에 정리했다.
