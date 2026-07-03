# 04 Point Goal Delivery

이 단계는 Nav2가 한 층에서 움직이는 것을 확인한 뒤, 사람이 방 번호나 지점 ID를 입력하면 해당 위치로 이동하게 만드는 단계다.

## 현재 문서

전체 설계와 개발 순서:

[01_kku_nav2_point_goal_workflow.md](01_kku_nav2_point_goal_workflow.md)

## 이 단계로 넘어오기 전 조건

먼저 아래가 되어야 한다.

```text
Gazebo만 실행 가능
저장 맵 map_server 로딩 가능
AMCL 위치추정 가능
RViz Nav2 Goal 이동 가능
장애물 앞에서 멈추거나 회피 가능
```

그 다음에 point registry를 만든다.

```text
src/common_pkg/config/kku_nav_points.yaml
```

예시 goal id:

```text
room_201
room_208
parcel_pickup
elevator_inside
elevator_exit
```

처음에는 같은 층 point 이동만 구현하고, 엘리베이터 내부 동작과 층간 상태머신은 나중에 붙인다.

