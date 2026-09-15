# F1/F2 navigation map handover


## 2026-09-14 SSH 반영 완료

젯슨 hsm-desktop에 재접속해 F1 기본 맵을 `f1_manual_clean_v1.yaml`, F2 기본 맵을 `f2_nav_unknown_v1.yaml`로 반영했다. 기존 latest_map과 waypoint registry를 백업하고 F2 waypoint의 지도 참조만 변경했다. 좌표는 보존했다. 재접속 재조회 SHA256이 로컬 전달 묶음과 모두 일치한다.

판정: **오프라인 검증(실제 Jetson 컨테이너)**. Nav2 `libmap_io`를 호출하는 독립 실행 파일로 지도 로딩 PASS. ROS 노드·publisher 생성과 로봇 구동은 하지 않았다. [측정값] F1 unknown/free/occupied = 146780/67110/2686, F2 = 1312411/123548/16453. AMCL 정합·실차 주행은 미확인이다.

백업: `/ros2_ws/logs/field_execution/navigation_defaults_20260914-111752` (컨테이너 UTC 시각으로 생성된 이름). F1/F2 원본 posegraph 및 data도 원격에서 존재·SHA256 확인했다. F1 보정본 전용 posegraph가 생긴 것은 아니다.

Initial pose must match the actual robot location and heading. Verify scan/map alignment before any goal. Use fieldctl waypoint save NAME FLOOR X Y YAW to register a stopping point; goal commands require separate field authorization. Elevator entry/inside/exit points are image-derived proposals, not validated boarding goals.
