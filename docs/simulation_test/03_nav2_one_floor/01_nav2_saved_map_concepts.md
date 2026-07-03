# 저장 맵 기반 Nav2 개념

## 한 줄 요약

SLAM은 맵을 만드는 단계고, Nav2는 이미 저장된 맵 위에서 목표 지점까지 이동하는 단계다.

## SLAM 실행과 Nav2 실행의 차이

SLAM 실행:

```text
Gazebo -> /scan, /odom, /tf
SLAM Toolbox -> /map 생성
map_saver_cli -> kku_f*.yaml + kku_f*.pgm 저장
```

Nav2 실행:

```text
Gazebo -> /scan, /odom, /tf
map_server -> 저장된 kku_f*.yaml 읽어서 /map 발행
AMCL -> /scan + /map + /tf로 현재 위치 추정
planner -> global path 생성
controller -> /cmd_vel 생성
Gazebo diff_drive -> 로봇 이동
```

## 왜 `kku_navigation.launch.py`가 필요한가

지금 있는 `kku_simulation.launch.py`는 Gazebo와 SLAM Toolbox를 같이 켜는 파일이다. 저장된 맵을 기준으로 자율주행하려면 SLAM Toolbox 대신 Nav2 bringup을 켜야 한다.

그래서 새 launch 파일이 필요하다.

```text
src/slam_pkg/launch/kku_navigation.launch.py
```

이 파일은 `floor:=F1/F2/F3`를 받아서 아래 맵 중 하나를 선택해야 한다.

```text
src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml
src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml
src/slam_pkg/maps/kku_virtual/f3/kku_f3.yaml
```

그리고 Nav2가 쓸 파라미터 파일도 필요하다.

```text
src/slam_pkg/config/nav2_params.yaml
```

## 첫 목표

처음부터 방 번호 goal까지 가지 말고, 먼저 RViz의 `Nav2 Goal` 버튼으로 한 층 안에서 이동이 되는지만 확인한다.

성공 기준:

```text
/map 발행됨
/amcl_pose 발행됨
RViz에서 initial pose 지정 가능
RViz에서 Nav2 Goal 지정 가능
/cmd_vel 발행됨
로봇이 저장 맵 위에서 목표까지 이동함
```

