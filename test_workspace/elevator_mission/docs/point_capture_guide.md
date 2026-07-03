# Elevator Mission Point Capture Guide

작성일: 2026-05-28

## 목적

`kku_nav_points.yaml`의 초기 좌표는 설계 좌표다. 실제 Nav2 mission 전에 각 층의 `elevator_entry`, `elevator_inside`, `elevator_exit`, 배송지 방 번호, `parcel_pickup` 좌표를 AMCL 기준 pose로 보정한다.

## 준비

해당 층의 Gazebo, Nav2, RViz를 실행한다.

```bash
ros2 launch common_pkg gazebo.launch.py floor:=F3 use_sim_time:=true
ros2 launch slam_pkg kku_navigation.launch.py floor:=F3
```

RViz에서 `2D Pose Estimate`로 현재 위치를 맞춘 뒤, Nav2 goal 또는 teleop으로 로봇을 기록할 위치에 세운다.

## 좌표 캡처

로봇이 목표 지점에서 문 또는 엘리베이터 출구를 바라보게 한 뒤 아래 명령을 실행한다.

```bash
python3 test_workspace/elevator_mission/scripts/capture_nav_point.py --floor F3 --point 303 --type room
```

출력 예시는 다음과 같다.

```yaml
# Paste under floors.F3.points in kku_nav_points.yaml
"303": {type: room, x: 1.650, y: 6.000, yaw_deg: 180.0}
```

출력된 한 줄을 `test_workspace/elevator_mission/config/kku_nav_points.yaml`의 해당 floor 아래에 반영한다. 기존 값을 바꿀 때는 이 테스트 워크스페이스 파일만 수정한다.

## 필수 캡처 목록

| Floor | Points |
|------|--------|
| F1 | `parcel_pickup`, `elevator_entry`, `elevator_inside` |
| F2 | `elevator_inside`, `elevator_exit`, `201`부터 `208` 중 테스트 대상 |
| F3 | `elevator_inside`, `elevator_exit`, `301`부터 `308` 중 테스트 대상 |

## 검증

좌표 반영 후 point registry 테스트를 먼저 실행한다.

```bash
python3 test_workspace/elevator_mission/scripts/test_point_registry.py
```

그 다음 단층 Nav2에서 해당 point로 이동시켜 문 앞 정지 위치와 yaw를 확인한다. 로봇이 문에 너무 붙거나 복도 중앙을 벗어나면 `x`, `y`, `yaw_deg`를 다시 캡처하거나 수동 보정한다.
