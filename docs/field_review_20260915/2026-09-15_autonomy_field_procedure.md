# 2026-09-15 자율주행 개선 후 현장 시험 절차와 기록 양식

판정: **오프라인 검증** (Jetson 파일 변경·무구동 명령·오프라인 시험까지). 이 문서의 모든 주행 단계와 합격 기준은 **실물 미검증**이다. 원인 분석은 [2026-09-15 리뷰](2026-09-15_autonomy_test_review.md), 이전 절차는 [2026-09-14 포인트 안내](2026-09-14_navigation_points.md)를 본다.

`base start --drive`, `resume`, `goal`, `teleop`, 보조 바퀴 문턱 시험은 사용자의 별도 현장 승인 후에만 실행한다. 위험하면 순서와 관계없이 D 터미널에서 `./scripts/fieldctl stop`을 입력한다.

## 1. 이번에 Jetson에서 바뀐 것

세션 백업: `/ros2_ws/logs/field_execution/autonomy_improvement_20260915_175043_KST/` (`before/`, `after/`, `map_guard_records/`, 각 폴더 `SHA256SUMS`). 로컬 사본은 `logs/field_execution/autonomy_improvement_20260915_175043_KST/`에 있다.

| 항목 | 변경 | 판정 |
|---|---|---|
| F1 기본 맵 | `f1/latest_map.txt`를 `f1_manual_clean_v2.yaml`로 원자적 교체. `fieldctl map validate` VALID, `nav start F1` 선택 경로 재현 v2 | 오프라인 검증 |
| 지도 fail-closed | `maps/field/map_pins.json` 신규(F1 v2, F2 `f2_nav_unknown_v1`, F3 현행). `nav start`는 pointer·명시 지도·YAML/PGM SHA256이 핀과 같아야 시작하고, `goal`·`pose capture`는 실행 중 map_server 로그의 로드 경로까지 같아야 진행 | 오프라인 검증 (goal 단계 PASS 경로는 합성 시험만) |
| F2 staging | registry에 `f2_elevator_staging_v1 = (-11.175, -3.075, 2.312744)` `[제안값]` 추가. 기존 `f2_elevator_entry` 불변 | 오프라인 검증 |
| bag 계측 | `fieldctl record start`가 존재하는 Nav2 진단 토픽을 추가 기록, 없는 토픽은 `absent_nav_topics.txt`. cache 256 KiB. `fieldctl record check` 신규 | 오프라인 검증 (실제 토픽 이름은 현장 확인) |
| bag 분석 | `scripts/analyze_nav_bag.py`: collision-ahead 등 로그 시점의 map pose, goal 상태, 종료 후 비영 명령, 명령/피드백 역산 RPM, 정지거리 | 오프라인 검증 (합성 bag) |
| pose 재등록 | `fieldctl pose capture NAME F1 [--save]`: 구독 전용 10초, 안정도·AMCL/TF 일치·costmap footprint lethal 검사, PASS일 때만 새 이름 저장 | 오프라인 검증 (순수 함수만) |
| gate 상한 override | `start_field_base.sh`·`field_base.launch.py`에 `GATE_MAX_LINEAR_SPEED`(비움·0.12·0.13만 허용) 추가. 비우면 `nav_safety.yaml` 0.12 그대로. launch를 실행하지 않고 setup 함수만 호출해 0.13 override 추가·0.20 거부를 확인 | 오프라인 검증 |
| 증속 후보 | `nav2_params_speed_v011.yaml`(0.11 m/s·0.22 rad/s), `nav2_params_speed_v012.yaml`(0.12·0.20). `NAV2_PARAMS_FILE`로 명시할 때만 사용. 기본 `nav2_params.yaml` 해시 `e8cf213b…` 불변 | 코드 존재 |

주요 SHA256(after): `fieldctl 6f9855a8…`, `start_field_navigation.sh c74460b8…`, `record_field_bag.sh 22dfb354…`, `field_map_guard.py 5e485347…`, `field_pose_capture.py 5bc3791e…`, `analyze_nav_bag.py d79f4a3e…`, `waypoints.json ebcfb132…`, `map_pins.json 0aeeb3e9…`, F1 `latest_map.txt 8736d5ba…`. 전체 목록은 세션 `after/SHA256SUMS`에 있다.

오프라인 시험: Jetson 호스트 `run_offline_tests.sh` `[측정값]` PASS 37 / SKIP 7 / FAIL 4. 변경 전 기준선은 PASS 35 / SKIP 6 / FAIL 4이고 FAIL 4건(`test_opencr_firmware_contract` 30 rpm 기대, `test_map_cleanup_review` 호스트 Python 3.8 `newline`, `test_kku_navigation_launch` 0.05 m/s 기대, `test_nav2_orthogonal_tuning` 허용오차 기대)은 변경 전부터 같다. 컨테이너에서 신규 3개와 `bag_contract`·`p07_scripts_contract`·`field_scripts_contract`를 노드 없이 실행해 모두 PASS했다. 노드를 띄우는 runtime 시험은 이번에 실행하지 않았다.

## 2. 매 세션 공통 사전 점검

터미널 A(base)·B(Nav2)·C(pose/goal/teleop/record)·D(stop/status)를 연다. 자율주행 시험 Wi-Fi는 `ssh hsm@10.141.228.26`, 평소는 `ssh hsm@192.168.0.7`이다.

1. D: `./scripts/fieldctl doctor`, `./scripts/fieldctl status`, `./scripts/fieldctl waypoint list`
2. D: `./scripts/fieldctl map guard F1` (Nav2 정지 중에는 pre-nav 단계, 실행 중에는 goal 단계로 판정). `MAP_GUARD=PASS`가 아니면 그 층은 시작하지 않는다.
3. A: `./scripts/fieldctl base start --drive` (승인 경계)
4. D: bridge와 gate가 실제로 읽은 상한을 기록한다.

```bash
docker exec ros2_humble bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 param get /packagu_opencr_bridge max_wheel_rpm && ros2 param get /packagu_opencr_bridge feedback_max_abs_rpm && ros2 param get /nav_safety_gate max_linear_speed && ros2 param get /nav_safety_gate max_angular_speed"
```

5. D: 펌웨어 인사말이 로그에 있으면 기록한다. `0.2-minimal`은 일반(48 rpm source) 계열, `0.3-imu`는 IMU(30 rpm source) 계열이다. `0.2-minimal` 문자열만으로는 30/48 빌드를 구분하지 못한다. OpenCR가 리셋되지 않으면 인사말이 없으므로 없을 때는 `미확인`으로 적는다.

```bash
docker exec ros2_humble bash -lc 'grep -ah "HELLO opencr" /root/.ros/log/python3_*.log | tail -3'
```

6. B: `./scripts/fieldctl nav start F1` (지도 인자 생략 → pointer v2 사용. 증속 시험만 `NAV2_PARAMS_FILE=... ./scripts/fieldctl nav start F1`)
7. C: `./scripts/fieldctl record start <세션명>` → `./scripts/fieldctl record check`가 `RECORD_CHECK=PASS`여야 그 주행을 진단 합격 판정에 넣는다.
8. D: 첫 goal 직전에 bag 토픽 이름을 확인한다. `absent_nav_topics.txt`는 record 종료 뒤 생성된다.

```bash
docker exec ros2_humble bash -lc "source /opt/ros/humble/setup.bash && ros2 topic list | sort && ros2 topic info -v /amcl_pose | grep -i durab"
```

9. 주행 후: 완전 정지 → D `./scripts/fieldctl stop` → C `./scripts/fieldctl record stop` → 분석:

```bash
docker exec -w /ros2_ws ros2_humble bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 scripts/analyze_nav_bag.py /ros2_ws/logs/field_bags/<세션명>"
```

## 3. 세션 기록 양식

주행 1회마다 한 줄을 채운다. action 결과와 현장 물리 결과는 반드시 별도 열이다. `nonzero_cmd_after_end`, `collision_ahead`, `max_cmd_rpm`은 분석 JSON에서 옮긴다.

| 세션명 | 시작/종료 KST | 층 | map guard 기록 | 로드 YAML·PGM SHA256 앞 8자 | initial pose | goal | params 파일 | record check | action 결과 | 물리 결과 | 수동 정지 시각·이유 | 실제 정차 오차(위치 m / 방향 °) | collision_ahead 수·최초 map pose | nonzero_cmd_after_end | max_cmd_rpm / max_fb_rpm | bag 경로 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | | | | | | | |

실패로 기록하는 조건: 강제 정지, 잘못된 문 진입, 벽 접촉, 지속 collision-ahead(action `SUCCEEDED`여도), 로드 지도 불일치, `record check` FAIL(진단 판정 제외).

## 4. F1 idle_v2·locker_v2 재등록

기존 `f1_idle`, `f1_locker`는 덮어쓰지 않는다. `f1_idle_v2`, `f1_locker_v2`를 새로 만들고 왕복 3회 합격 뒤에만 기본 이름 교체를 검토한다.

### 4.1 배치와 정합

1. 바닥 테이프로 idle 중심점(로봇 `base_footprint` 원점 = 차체 앞쪽 끝에서 약 0.033 m 뒤)과 정면 방향 화살표를 표시한다. 사용자 보고대로 기존 위치보다 로봇 기준 **오른쪽 벽 쪽**에 둔다.
2. 줄자로 로봇 측면과 오른쪽 벽의 최소 거리를 mm로 잰다. `[제안 기준]` 0.30 m 이상.
3. 참고 시드(지도 계산값, 실제 좌표 아님): 기존 `f1_idle`에서 로봇 오른쪽으로 0.8 m 옮긴 `(-4.518, 2.179, -1.701)`은 v2 지도상 footprint–벽 여유 0.362 m, 1.0 m 옮긴 `(-4.717, 2.205, -1.701)`은 0.178 m다. 0.8 m 쪽을 시드로 쓴다(2026-09-15 사용자 확인: "0.8 m 정도가 적당"). 저장 좌표는 이 시드가 아니라 6단계 capture 값이다. 근거: `artifacts/autonomy_improvement_20260915/waypoint_offline_check.json`.
4. D: `./scripts/fieldctl stop`으로 software stop을 건 상태에서 C: `./scripts/fieldctl pose set -4.518 2.179 -1.701`
5. scan이 벽과 맞는지 눈으로 확인한다. 맞지 않으면 원격 RViz 또는 현장 웹 UI의 `2D Pose Estimate`로 다시 맞춘다. 웹 UI는 `/cmd_vel`을 발행하므로 software stop 유지 중에만 쓰고, 끝나면 종료한 뒤 D `./scripts/fieldctl diagnose`의 `/cmd_vel` publisher 목록에 웹 UI가 없는지 확인한다. 웹 UI만 재시작하면 gate ready가 false로 남을 수 있으니 `fieldctl status`로 확인한다.
6. C: `./scripts/fieldctl pose capture f1_idle_v2 F1 --duration 10 --floor-mark IDLE-A` → 모든 항목 PASS면 같은 명령에 `--save`를 붙여 저장한다. FAIL이 하나라도 있으면 저장하지 않고 goal도 보내지 않는다. 특히 `global_costmap_footprint_non_lethal` FAIL은 9/14 v2 `Starting point in lethal space`와 같은 상태다.

`pose capture` PASS 조건 `[제안값]`: AMCL·TF 표본 5개 이상, TF 표준편차 xy 0.02 m·yaw 1° 이하, AMCL 공분산 xx·yy 0.04 m² 이하·yaw (5°)² 이하, AMCL–TF 차 0.05 m·3° 이하, global·local costmap footprint에 lethal(≥253)·unknown·격자 밖 셀 0개.

### 4.2 locker_v2

1. D `resume` → C `teleop`으로 **열린 절반**(v2 지도에서 유리 고정문 선분이 없는 쪽, 보관함 방향을 볼 때 왼쪽)을 0.10 m/s 이하로 지나 보관함 앞 실제 정차점에 둔다. 사진의 잠긴 문 쪽으로 가지 않는다.
2. C `k` → `Ctrl+C` → D `./scripts/fieldctl stop`.
3. 테이프 표시 후 C: `./scripts/fieldctl pose capture f1_locker_v2 F1 --floor-mark LOCKER-A` → PASS면 `--save`.

### 4.3 측정표

| 이름 | 테이프 ID | 로봇–오른쪽 벽 최소 거리(mm) | 정면 방향 확인 | TF x | TF y | TF yaw | TF std xy / yaw | AMCL cov xx / yy / yaw | global/local lethal 셀 | map guard 기록 | capture 기록 | 저장 여부 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f1_idle_v2 | | | | | | | | | | | | |
| f1_locker_v2 | | | | | | | | | | | | |

### 4.4 왕복 3회

`f1_idle_v2 → f1_locker_v2 → f1_idle_v2`를 3회 반복한다. 매 goal 도착 후 D `stop`, 테이프 기준 실제 오차를 줄자로 잰다.

| 회차 | 구간 | action 결과 | 물리 결과 | 테이프 기준 위치 오차(m) | 방향 오차(°) | 잘못된 문 진입 | 강제 정지 | collision_ahead | bag |
|---|---|---|---|---|---|---|---|---|---|
| 1 | idle_v2→locker_v2 | | | | | | | | |
| 1 | locker_v2→idle_v2 | | | | | | | | |

`[제안 기준]` 합격: 실제 로드 지도 v2(map guard goal 단계 PASS), 시작점 non-lethal(capture PASS), 6개 구간 모두 위치 0.15 m·방향 5° 이하, 잘못된 문 진입·강제 정지 0회.

## 5. F2 staging

기존 `f2_elevator_entry`는 그대로 두고 `f2_elevator_staging_v1`까지만 자율주행한다. 오프라인 계산 `[측정값]`(지도 사본 기준): staging 중심–비이동 영역 0.886 m, footprint 가장자리–벽 0.729 m, footprint 안 occupied/unknown 0셀, 목적지에서 circumscribed 0.394 m 여유로 연결. 기존 entry는 각각 0.461 m, 0.240 m다. 근사 경로(비용 가중 Dijkstra, Nav2 재현 아님)의 staging 마지막 3 m 최소 중심 여유는 0.552 m다. 그림: `artifacts/autonomy_improvement_20260915/f2_staging_overlay.png`(빨강 entry, 초록 staging, 파랑 근사 경로).

벽에 붙어 멈춘 곳(2026-09-15 사용자 확인): 마지막 문틀이 아니라 **엘리베이터를 바라볼 때 오른쪽, 벽 안으로 움푹 들어간 곳**이다. 지도에서는 `(-11.05, -1.85)` 부근의 들어간 벽 모양으로 추정한다(그림 `artifacts/autonomy_improvement_20260915/f2_recess_candidates.png`의 A). 지도 계산으로 기존 entry 중심은 A에서 0.749 m, staging_v1은 1.231 m이며, 목적지→staging 근사 경로는 A 쪽으로 가지 않는다. 현장에서는 A 앞을 지날 때의 실제 측면 여유를 따로 적는다.

절차: `goal f2_delivery_destination F2` 후 D `stop` → 하차 → D `resume` → C `./scripts/fieldctl goal f2_elevator_staging_v1 F2` → 도착 즉시 D `./scripts/fieldctl stop` → D `./scripts/fieldctl nav stop` → 문 열림 확인 후 D `resume` → C `teleop`으로 수동 진입.

| 회차 | 목적지→staging action | 완전 정지 | 움푹 들어간 곳(A) 앞 실제 측면 여유 최소(m) | 벽 접촉 | 강제 정지 | collision_ahead 수·최초 map pose | bag |
|---|---|---|---|---|---|---|---|
| 1 | | | | | | | |
| 2 | | | | | | | |
| 3 | | | | | | | |

`[제안 기준]` 합격: 3회 모두 staging 완전 정지, 측면 여유 0.20 m 이상, 벽 접촉·강제 정지 0회, 지속 collision-ahead 없음. 3회 중 벽을 타면 bag의 `/plan`·costmap에서 벽 쪽 경로가 처음 생긴 위치를 찾아 복도 중심 경유점 또는 작은 keepout을 검토한다. F2 전체 inflation은 먼저 바꾸지 않는다.

## 6. 보조 바퀴 H0와 문턱 시험

로봇 전원을 끈 상태에서 먼저 잰다. 값은 모두 mm, 측정자·도구를 적는다.

| 항목 | 값(mm) | 측정 방법·도구 | 측정자 |
|---|---|---|---|
| 엘리베이터 외부 바닥 – 캐빈 바닥 높이 차(F1) | | | |
| 같은 값(F2) | | | |
| 문 틈 폭(F1 / F2) | | | |
| 볼캐스터 지름 | | | |
| 보조 바퀴 지름 | | | |
| H0 좌 보조 바퀴 평지 지상고 | | 평판 위 틈새 게이지 | |
| H0 우 보조 바퀴 평지 지상고 | | | |
| 구동륜 하중 변화 느낌·흔들림 | | 손으로 누름 | |

H0는 `[사용자 보고] 약 2 mm 하향`한 현재 장착 상태다. 지상고가 `[제안값]` 1~2 mm이고 평지에서 닿지 않으면 그대로 시험한다. 닿거나 흔들리거나 구동륜이 들리면 더 낮추지 않고 이전 높이와 비교한다.

평지 확인(H0): 직진·후진·제자리 회전 각 1회, 보조 바퀴 접지음·흔들림·구동륜 공회전 여부를 적는다.

모형 틈·문턱(실측 치수와 동일)에서 수직 접근, 0.03 → 0.05 → 0.08 → 0.10 m/s 순서로 한 번씩. 걸리면 즉시 정지·회수하고 같은 조건으로 반복 가속하지 않는다. 문턱 시험 상한은 0.10 m/s다.

| 높이 | 방향(탑승/하차) | 속도(m/s) | 결과(통과/걸림) | 캐스터 빠짐 | 차체 충격 | 구동륜 공회전 | 비고 |
|---|---|---|---|---|---|---|---|
| H0 | 탑승 | 0.03 | | | | | |

모형에서 통과한 높이·최저 속도 조합만 실제 엘리베이터에서 탑승·하차 각 3회 시험한다.

## 7. 순항 증속

전제: 2절 4·5단계 기록. `max_wheel_rpm`이 48.0으로 읽히지 않으면 증속 시험을 하지 않는다. 9/13 21:27 KST 로그의 피드백 `F 12.137 48.548`은 당시 펌웨어가 30 rpm보다 큰 명령을 받았다는 근거지만 9/14 이후 탑재 펌웨어는 여전히 미확인이다.

bridge는 좌우 목표 중 하나라도 48 rpm을 넘으면 감속하지 않고 0을 쓰며 drive ready를 끈다. 그래서 후보는 선속도·각속도 동시 최대에서도 48 rpm 아래로 잡았다(`wheel_radius=0.033`, `wheel_separation=0.4323` 계산).

| 단계 | params | 선속도·각속도 | 직진 rpm | 동시 최대 바깥 rpm |
|---|---|---|---:|---:|
| 기준 | 기본 `nav2_params.yaml` | 0.10·0.25 | 28.9 | 44.6 |
| v011 | `/ros2_ws/src/slam_pkg/config/nav2_params_speed_v011.yaml` | 0.11·0.22 | 31.8 | 45.6 |
| v012 | `/ros2_ws/src/slam_pkg/config/nav2_params_speed_v012.yaml` | 0.12·0.20 | 34.7 | 47.2 |

0.12 m/s는 기본 safety gate `max_linear_speed=0.12`와 같아 경계값이다. 2026-09-15 사용자 승인에 따라 **v012 단계에서만** base를 다음처럼 시작해 gate 상한을 0.13으로 올린다(허용값은 비움·0.12·0.13뿐이며 그 외는 시작 거부). 기준·v011 단계와 다른 모든 주행은 변수 없이 시작해 0.12를 유지한다. v012가 끝나면 base를 멈추고 변수 없이 다시 시작한다.

```bash
GATE_MAX_LINEAR_SPEED=0.13 ./scripts/fieldctl base start --drive
```

시작 출력의 `[field-base] gate max_linear_speed=0.13`과 2절 4단계 `ros2 param get /nav_safety_gate max_linear_speed` 값을 기록한다. 가속·감속 값은 올리지 않는다. 엘리베이터 문턱 속도는 이 표와 무관하게 0.10 m/s 이하다.

각 단계마다 직선 5 m·완만한 곡선·정지를 3회. Nav2는 `./scripts/fieldctl nav stop` 후 `NAV2_PARAMS_FILE=<params> ./scripts/fieldctl nav start F1`로 재시작한다.

| 단계 | 회차 | 구간 | max_cmd_rpm | max_fb_rpm | 48 rpm 초과 샘플 | drive ready 이탈 | firmware error | 정지거리(m) | 경로 이탈 | 모터 이상음 | 모터 온도(측정 방법) | 판정 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 기준 | 1 | 직선 | | | | | | | | | | |

중단·복귀 조건: drive ready 이탈, feedback/firmware error, 48 rpm 초과, 기준 대비 정지거리 악화, 경로 이탈, 이상음·온도 상승 중 하나. 합격값은 3회 모두 위 조건이 없고 기록이 남은 가장 높은 단계다. 직진 48 rpm(약 0.166 m/s) 순항은 선·각속도를 같은 비율로 줄이는 wheel-RPM limiter 구현·검증 전에는 시도하지 않는다(**계획만**).

## 8. 전체 시나리오 0~9 재시험 순서

구간별 합격(4.4, 5, 6) 뒤에만 실행한다. 층마다 2절 사전 점검을 다시 한다.

| 단계 | 명령(터미널) | 반드시 멈추는 곳 |
|---|---|---|
| 0 | B `nav start F1` → 4.1 정합 → C `pose capture f1_idle_v2 F1`(저장 없이 확인) | capture PASS 전 goal 금지 |
| 1~2 | D `resume` → C `goal f1_locker_v2 F1` | 도착 후 D `stop`, 수동 적재 |
| 3 | D `resume` → C `goal f1_elevator_entry F1` | D `stop` → D `nav stop` |
| 4 | D `resume` → C `teleop` 탑승 | 캐빈 안 C `k`→`Ctrl+C` → D `stop` |
| 5 | D `resume` → C `teleop` 하차 → B `nav start F2` → C `pose set -11.375 -2.525 -0.828849` | 하차 후 D `stop`, 정합 확인 전 goal 금지 |
| 6~7 | D `resume` → C `goal f2_delivery_destination F2` | D `stop`, 수동 하차 |
| 8 | D `resume` → C `goal f2_elevator_staging_v1 F2` | D `stop` → D `nav stop` → 수동 탑승 → 캐빈 안 D `stop` |
| 9 | 하차 → B `nav start F1` → C `pose set -8.725 -0.175 0.918927` → D `resume` → C `goal f1_idle_v2 F1` | 도착 후 D `stop` |
| 종료 | C `record stop` → D `nav stop` → D `base stop` → D `status` | |

`goal`은 매번 map guard(goal 단계)를 먼저 실행한다. `MAP_GUARD=FAIL`이면 goal이 전송되지 않는다.
