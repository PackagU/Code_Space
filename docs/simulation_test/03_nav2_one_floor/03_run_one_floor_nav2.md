# 저장 맵으로 한 층 Nav2 실행

이 문서는 `kku_navigation.launch.py`와 `nav2_params.yaml`을 만든 뒤 실행하는 절차다.

## 터미널 1: Gazebo만 실행

SLAM을 켜지 않는다. 저장 맵 기반 Nav2에서는 Gazebo가 센서, odom, tf만 제공하면 된다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
```

## 터미널 2: Nav2 실행

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
```

## RViz에서 해야 할 일

1. `2D Pose Estimate`로 로봇의 초기 위치를 찍는다.
2. 로봇 위치는 Gazebo spawn 위치와 비슷한 곳에 찍는다.
3. `Nav2 Goal`로 같은 층 복도 안의 목표점을 찍는다.
4. global path가 생기고 로봇이 움직이는지 확인한다.

## 정상 동작 확인 명령

터미널을 하나 더 열고 확인한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /cmd_vel --once
```

`/cmd_vel`이 나오는데 로봇이 움직이지 않으면 drive/Gazebo 쪽 문제일 가능성이 크다. `/cmd_vel`이 안 나오면 Nav2 planning/controller 쪽을 먼저 본다.

## F1/F3도 같은 방식

```bash
ros2 launch common_pkg gazebo.launch.py floor:=F1 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F1
```

```bash
ros2 launch common_pkg gazebo.launch.py floor:=F3 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F3
```

Gazebo와 Nav2의 `floor` 값은 반드시 같아야 한다.

