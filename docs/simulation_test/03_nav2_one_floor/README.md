# 03 Saved Map Nav2 One-Floor Test

이 단계는 SLAM을 다시 켜지 않고, 저장된 맵으로 한 층 안에서 Nav2 자율주행을 테스트하는 단계다.

## 추천 순서

1. 저장 맵 Nav2 구조가 뭔지 먼저 이해  
   [01_nav2_saved_map_concepts.md](01_nav2_saved_map_concepts.md)

2. `kku_navigation.launch.py`와 `nav2_params.yaml` 만드는 법  
   [02_create_kku_navigation_launch.md](02_create_kku_navigation_launch.md)

3. Gazebo와 Nav2를 실제로 실행하고 RViz goal 테스트  
   [03_run_one_floor_nav2.md](03_run_one_floor_nav2.md)

4. 안 움직이거나 localization/path가 이상할 때  
   [04_troubleshooting.md](04_troubleshooting.md)

## 핵심

맵을 만드는 명령과 저장 맵으로 주행하는 명령은 다르다.

```bash
# 맵 생성 모드
ros2 launch slam_pkg kku_simulation.launch.py floor:=F2

# 저장 맵 Nav2 모드
ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
```

