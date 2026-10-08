# 2026-09-15 자율주행 현장 테스트 리뷰와 개선 계획

## 판정과 근거 범위

- **F2 주행:** 사용자 현장 보고 기준 목적지 왕복 구간은 대부분 **실물 검증**. 로그상 두 goal 모두 성공했지만 복귀 중 controller의 collision-ahead 경고가 795회, 361.75초 동안 이어졌다. 엘리베이터 도착 전 마지막 문 부근에서 벽에 과도하게 접근했다는 현장 보고와 시간상 부합한다. 이 시험의 ROS bag은 생성되지 않았다.
- **F1 주행:** 23:32 KST 보관함 주행은 `f1_manual_clean_v2`를 사용했으나 취소됐고, 끝부분에서 시작점이 lethal space라는 계획 실패가 4회 기록됐다. 00:09 KST 엘리베이터→idle 복귀는 v2가 아닌 `f1_manual_clean_v1`을 로드했으며 Nav2는 성공을 반환했지만 실제 정지 위치는 사진 2장과 같이 어긋났다.
- **F1 v2 지도:** 로컬 파일과 manifest 기준 **오프라인 검증**. v1 대비 유리 고정문을 나타내는 occupied 선분 51셀이 추가됐다. 2026-09-15 00:41 KST `hsm@10.141.228.26`에서 Jetson의 v2 PGM/YAML 해시가 로컬과 일치함을 확인했다. PGM SHA256은 `3f9a40c42bbdb73933eac76cd187853b8502d2587e398b24b7537e4b8e3ab9d1`이다. 리뷰 작성 시점에는 `latest_map.txt`가 v1을 가리켰으나 **2026-09-15 백업(17:50:43 KST) 직후 v2로 교체했다**(아래 실행 결과).
- **엘리베이터 문턱:** 보조 바퀴 2개 추가 후 하차 성공·탑승 실패는 사용자 현장 보고 기준 **실물 검증**. 이후 보조 바퀴 장착 위치를 `[사용자 보고] 약 2 mm 하향`했으며, 조정 후 지상고·접지 여부와 재시험 결과는 **미확인**이다.

현장 사진은 `artifacts/autonomy_test_review_20260915/`에 보존했다.
수집한 원본 로그와 세션별 분석은 `logs/field_execution/20260915_autonomy_test_collection/README.md`에 보존했다.

## 2026-09-15 17:48~18:20 KST 후속 실행 결과

근거: `logs/field_execution/autonomy_improvement_20260915_175043_KST/README.md`(Jetson 같은 경로), 현장 절차: [2026-09-15 개선 후 절차](2026-09-15_autonomy_field_procedure.md).

| 계획 항목 | 결과 | 판정 |
|---|---|---|
| 1. 무구동 기준선·백업 | 해시·pointer·registry·설정 백업. 원본 로그 재집계 5개 goal 수치가 `session_summary.csv`와 일치 | 오프라인 검증 |
| 2. F1 v2 기본 선택 | pointer v2, `map validate` VALID, `nav start F1` 선택 경로 v2 재현. `map_pins.json` + guard로 nav start·goal·pose capture fail-closed | 오프라인 검증 |
| 3. F2 staging | `f2_elevator_staging_v1` 등록(중심 여유 0.886 m, footprint–벽 0.729 m, 연결성 PASS). entry 불변. keepout·inflation 미변경 | 오프라인 검증 |
| 4. F1 재측정 준비 | `fieldctl pose capture`(구독 전용 안정도·lethal footprint 검사)와 측정표. 로봇 오른쪽 0.8 m 시드 `(-4.518, 2.179)`는 참고값 | 코드 존재 / 현장 미실행 |
| 5. rosbag 계측 | Nav2 진단 토픽 추가 기록, `record check`, `analyze_nav_bag.py`(합성 bag PASS) | 오프라인 검증 / 실제 토픽 이름 미확인 |
| 6. 문턱 시험 | H0 측정표·속도 계단 시험표 작성 | 계획만 |
| 7. 증속 | 0.11·0.12 m/s 후보 params 별도 파일, 기본값 불변. 9/13 21:27 피드백 48.5 rpm으로 당시 펌웨어 30 rpm 초과 수락 확인, 9/14 이후 탑재본 미확인 | 코드 존재 |
| 8. 실차 검증 | 실행하지 않음 | 미확인 |

오프라인 계산에서 새로 드러난 점: Nav2 footprint가 `x∈[-0.327, 0.033]`라 inscribed radius가 0.033 m다. 차체 대부분이 `base_footprint` 뒤에 있어서 costmap의 INSCRIBED(253) 띠가 장애물 둘레 0.033 m에 불과하다. 중심점만 보는 계획기라면 벽에 가까운 경로가 쉽게 나온다. F2 벽 근접과 F1 `Starting point in lethal space`의 후보 요인으로 기록하되, 계획대로 bag 근거 전에는 footprint·inflation을 바꾸지 않았다. Nav2 Humble SmacPlanner2D 소스(GitHub humble 브랜치)에서 footprint를 반지름으로 두고 중심 셀 cost ≥ INSCRIBED만 충돌로 보는 것을 확인했다. 사용자가 그림에서 지정한 F2 벽 근접 지점은 복도에서 로비로 꺾기 직전 왼쪽 벽의 움푹 들어간 곳(지도 `(-12.95, -5.25)`)이다. 처음 추정한 `(-11.05, -1.85)`는 틀렸다. 기본 설정 근사 경로는 이 코너를 inflation 경계(0.552 m)에 붙어 돌아서, 후보 `nav2_params_wall_push_v1.yaml`(global inflation 1.0 m·scaling 2.0, travel multiplier 3.0)을 만들었다([절차 5.1](2026-09-15_autonomy_field_procedure.md)). F1은 v2를 직각화하고 엘리베이터를 네모로 바꾼 `f1_manual_clean_v3`를 만들었다. 20:39~20:45 KST 사용자 결정으로 F1 기본 지도는 v3, Nav2 기본값에는 wall_push_v1이 적용됐다(이전 기본값은 `nav2_params_pre_wallpush_20260915.yaml`).

## 원인 분석

### F2 벽 접근

현재 `f2_elevator_entry`는 지도상 비이동 영역까지 약 0.474 m이고 Nav2 footprint 외접은 약 0.39 m다. 중심 기준 여유가 약 0.084 m뿐이므로 지도 잡음, AMCL 오차, local costmap 장애물 한 셀만 더해져도 벽에 붙거나 정지하기 쉽다. F2 나머지 경로가 양호했으므로 전역 Nav2 튜닝을 먼저 바꾸기보다 해당 문 앞 접근점과 마지막 경유점부터 수정한다.

원본 F2 지도에서 같은 표시 구역 안의 복도 중심 후보 `(-11.175, -3.075)`는 비이동 영역까지 약 0.941 m다. 이를 `[제안값] f2_elevator_staging_v1`으로 추가해 기존 entry를 보존한 채 비교한다. staging까지만 자율주행하고 실제 엘리베이터 진입은 계속 수동으로 한다.

### F1 위치와 보관함 경로

F1 문제는 하나의 waypoint 오차만으로 설명하기 어렵다.

1. 직접 그린 외곽선은 실제 SLAM 지도의 절대 위치·각도를 보존하지 않는다.
2. 이미지에서 계산한 `f1_idle`을 실제 바닥 표식 없이 초기 pose로 사용해 시작 오차가 반복됐다.
3. 시작 오차가 난 상태에서 보관함 goal을 보내면서 문 위치도 같은 방향으로 어긋나 보였다.
4. LiDAR가 유리 고정문을 안정적으로 검출하지 못해, 정적 지도에 없는 경우 local costmap만으로 막힌 문을 피하기 어렵다.

`f1_manual_clean_v2`에 유리 고정문을 정적 벽으로 추가한 방향은 맞다. 다만 사진만으로 실제 map 좌표를 다시 계산하면 카메라 위치·렌즈·기준 길이가 없어 같은 절대 오차가 반복된다. 현장에서 로봇을 정확한 위치에 놓고 scan 정합 후 `/amcl_pose` 또는 `map → base_footprint`를 캡처해야 한다.

기존 `f1_idle`과 `f1_locker`는 즉시 덮어쓰지 않는다. 먼저 `f1_idle_v2`, `f1_locker_v2`로 저장하고 왕복 시험을 통과한 뒤 기본 이름을 교체한다.

### 엘리베이터 보조 바퀴

속도 증가보다 보조 바퀴 높이 조정이 우선이다. 첫 시험 뒤 사용자가 장착 위치를 약 2 mm 낮췄다. 이 조정은 볼캐스터가 틈에 빠지기 전에 보조 바퀴가 하중을 받게 하려는 방향과 맞지만, 현재 실제 지상고와 평지 접지 여부는 아직 측정하지 않았다.

무조건 바닥에 강하게 닿게 낮추면 평지에서 차체가 흔들리거나 구동륜 하중이 줄고 제자리 회전이 나빠질 수 있다. 추가 조정 전에 현재 높이를 기준점 `H0`로 표시하고 평지 지상고를 mm로 측정한다. `H0`가 `[제안값] 1~2 mm`이고 평지 비접촉이면 먼저 그대로 시험한다. 접지하거나 흔들리면 더 낮추지 말고 높이를 되돌려 비교한다.

속도를 높여 충격량으로 넘기는 방식은 문턱 충격, 미끄러짐, odom 오차와 전복 위험을 키운다. 전체 safety gate 상한은 0.12 m/s지만 문턱 시험 상한은 0.10 m/s로 별도 유지한다. 높이 조정 후 모형 문턱에서 0.03 → 0.05 → 0.08 → 0.10 m/s 순으로 한 번씩 시험하고, 낮은 속도에서 통과하는 설정을 채택한다. 걸렸을 때 반복 가속하지 않는다.

### 자율주행 순항 속도

2026-09-15 SSH로 읽은 Jetson 설정은 이미 bridge `max_wheel_rpm=48.0`, 일반 OpenCR firmware source `MAX_ABS_RPM=48.0`이다. 다만 실제 flashed firmware 변형과 Dynamixel Velocity Limit 레지스터는 미확인이고, IMU firmware 변형 source에는 아직 30 rpm 제한이 남아 있다.

현재 Nav2는 `[설정값] desired/max linear=0.10 m/s`, 최대 각속도 0.25 rad/s, safety gate 선속도 상한 0.12 m/s다. `[계산값] wheel_radius=0.033 m`, `wheel_separation=0.4323 m`를 적용하면 0.10 m/s 직진은 약 28.9 rpm, 0.10 m/s와 0.25 rad/s가 동시에 걸린 바깥 바퀴는 약 44.6 rpm이다. 즉 소프트웨어 바퀴 상한은 이미 48 rpm이지만 평상시 직진 순항은 약 29 rpm이다.

첫 증속 후보는 `[제안값] 0.12 m/s·0.20 rad/s`다. 직진 약 34.7 rpm, 동시 명령의 바깥 바퀴 약 47.2 rpm이므로 48 rpm 제한 안에 남고 순항은 20% 빨라진다. 0.12 m/s는 현재 safety gate 상한과도 같다. 직진 자체를 48 rpm인 약 0.166 m/s로 설정하면 회전 여유가 사라지므로, 곡률에 따라 선·각속도를 함께 축소하는 wheel-RPM limiter와 부하·제동 시험 전에는 적용하지 않는다. 엘리베이터 문턱 구간은 순항 증속과 분리해 계속 0.10 m/s 이하로 시험한다.

## 개선 실행 순서

### P0. 증거와 버전 고정

1. Jetson 연결 후 F1 `latest_map.txt`, v2 PGM/YAML SHA256, waypoint registry를 백업한다.
2. 확인된 v2 해시를 유지한 채 `latest_map.txt`를 v2로 원자적으로 변경하고 `fieldctl map validate`로 검사한다.
3. 가능하면 현장 bag에서 `/amcl_pose`, `/tf`, `/plan`, costmap, `/cmd_vel`, `/odom`을 보존한다.

### P1. F2 국소 수정

1. 기존 `f2_elevator_entry`를 보존한다.
2. `[제안값] f2_elevator_staging_v1 = (-11.175, -3.075, 2.312744)`를 별도 등록한다.
3. 목적지 → staging 단일 구간을 3회 시험한다.
4. 계획 경로 중심의 지도상 여유 `[제안 기준] ≥0.55 m`, 실제 로봇 측면 여유 `[제안 기준] ≥0.20 m`를 확인한다.
5. 여전히 벽을 타면 해당 구간에 복도 중심 경유점 또는 keepout 영역을 추가한다. F2 전체 inflation 값 변경은 마지막 선택으로 둔다.

### P2. F1 v2 재기준화

1. 바닥에 idle 중심과 정면 방향을 테이프로 표시한다.
2. 사용자가 말한 대로 기존 위치보다 오른쪽 벽 쪽의 안전한 지점에 로봇을 놓되, 실제 벽과 footprint 여유를 줄자로 확인한다.
3. RViz `2D Pose Estimate`로 scan을 벽에 맞춘 뒤 10초간 안정화한다.
4. 안정된 `map → base_footprint`를 읽어 `f1_idle_v2`로 저장한다.
5. teleop으로 올바른 고정문 통로를 지나 보관함 앞 정차 위치에 놓고 같은 방식으로 `f1_locker_v2`를 저장한다.
6. v2 지도에서 유리 고정문 occupied 선분과 열린 통로 폭이 footprint를 수용하는지 검사한다.
7. idle_v2 → locker_v2 → idle_v2를 3회 반복한다. `[제안 기준]` 같은 바닥 표식에서 위치 편차 0.15 m 이하, 방향 편차 5° 이하, 잘못된 문 진입 0회를 합격으로 한다.

### P3. 보조 바퀴와 문턱

1. 캐빈 외부 바닥과 내부 바닥 높이 차, 틈 폭, 볼캐스터 지름, 보조 바퀴 지름과 평지 지상고를 mm 단위로 측정한다.
2. 실제 엘리베이터 전에 같은 치수의 모형 틈·문턱을 만든다.
3. `[사용자 보고] 약 2 mm 하향`한 현재 상태를 `H0`로 기록한다. H0의 실제 평지 지상고와 접지를 측정한 뒤, 필요할 때만 1 mm 단위 후보를 추가한다.
4. 각 높이에서 평지 직진·후진·제자리 회전·흔들림을 확인한다.
5. 통과 시험은 문턱에 수직으로 정렬하고 0.03 m/s부터 올린다. 한 조건에서 걸리면 정지 후 회수하고 같은 조건으로 연속 재가속하지 않는다.
6. 모형에서 합격한 조합만 실제 엘리베이터에서 탑승·하차 각 3회 확인한다.

### P4. 자율주행 증속

1. 실제 flashed firmware가 48 rpm 변형인지, bridge가 `max_wheel_rpm=48.0`을 로드했는지 확인한다.
2. 현재 0.10 m/s를 기준으로 `/cmd_vel`, `/cmd_vel_safe`, 좌우 feedback rpm, `/odom`, `/drive/ready`, firmware error를 bag에 기록한다.
3. 현장 승인 후 0.10 → 0.11 → 0.12 m/s 순으로 직선·완만한 곡선·정지를 각 3회 시험한다. 가속도·감속도 값은 먼저 올리지 않는다.
4. 0.12 m/s 단계에서는 각속도 상한을 `[제안값] 0.20 rad/s`로 묶어 계산상 바깥 바퀴가 48 rpm을 넘지 않게 한다.
5. drive-ready 이탈, feedback/firmware error, 명령 48 rpm 초과, 정지거리 악화, 경로 이탈, 모터 이상음·온도 상승 중 하나라도 발생하면 직전 합격값으로 되돌린다.
6. 직진 48 rpm 순항은 curvature-aware wheel-RPM limiter와 전류·온도·제동거리 근거를 확보한 뒤 별도 후보로 다룬다.

### P5. 전체 시나리오 재시험

F1 v2 초기화 → 보관함 → F1 엘리베이터 앞 정지 → 수동 탑승 → F2 하차·초기화 → 목적지 → F2 staging 정지 → 수동 탑승 → F1 하차 → idle_v2 복귀 순으로 실행한다. 각 엘리베이터 앞과 캐빈 내부에서는 기존 runbook대로 software stop을 별도 확인한다.

## 완료 조건

- Jetson 기본 F1 맵과 로컬 v2 해시 일치.
- 새 waypoint는 기존 registry 백업 후 별도 이름으로 등록.
- F1 왕복 3회 동안 잘못된 문 진입과 강제 정지 없음.
- F2 마지막 문 구간에서 실제 측면 여유 0.20 m 이상을 3회 유지.
- 엘리베이터 탑승·하차 각 3회 성공, 캐스터 빠짐·차체 충격·구동륜 공회전 없음.
- 각 결과에 날짜, 지도 해시, waypoint 값, 속도, 보조 바퀴 지상고와 실제 결과를 기록.
