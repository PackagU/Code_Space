# 젯슨 현장 스킬 시험 (엘리베이터 입장 · 내부 버튼·하차 · 택배 승차)

로봇이 사람 손 없이 다음 세 동작을 스스로 하는지 현장에서 하나씩 시험하는 코드다. 상태기계(`skilltests/`)는 시뮬레이션(`sim/real_maps`)과 젯슨이 **같은 코드**를 쓰고, 하드웨어에 닿는 어댑터만 다르다. 시뮬 어댑터는 `sim/real_maps/sim_adapters.py`, 젯슨 어댑터는 `skilltests/ros_adapters.py`다.

## 시험과 상태 순서

| 시험 | 상태 순서 |
|---|---|
| `elevator_entry` 엘리베이터 입장 | `ELEVATOR_ENTRY_STATE` → 버튼 구역(호출 버튼 1 m 앞)으로 이동 → 정해진 보기 자세로 호출 버튼 찾기(안 보이면 0.1 m씩 물러나 다시) → 누르기 사이클의 팔 끝이 버튼에 닿을 정지 자세 계산 → 0.25 m 앞 정렬점에서 버튼 다시 보기(`RECHECK_BUTTON`, 크게 바뀌면 다시 정렬) → 곧게 다가감 → 팔 끝 착지 오차 확인(깊이만 조금 틀리면 앞뒤로만 맞춤 `ADJUST_DEPTH`, 아니면 물러나서 다시) → 티칭 사이클로 누르기 → 후진 → 기존 팔 카메라로 캡 재인식·누름 자리 정렬 확인(`POST_PRESS_LOOK`) → 문 열림 반응 확인 → 문 밖 대기점 → 문 열림 대기(정면 LiDAR 또는 문 틈 인식) → LiDAR 로 본 실제 문 틈에 맞춰 다시 섬(`ALIGN_TO_DOOR`) → 문에 수직으로 들어가 문 축 위 캐빈 깊이에 섬. 문이 안 열리거나 들어가는 중 막히면 물러나서 다시 호출 |
| `elevator_ride` 내부 버튼·하차 | `ELEVATOR_RIDE_STATE` → 오른쪽 90° → 정해진 캐빈 보기 자세로 층 버튼 찾기 → 위와 같은 정렬·다시 보기·착지 확인 → 티칭 사이클 2로 누르기 → 그 자리(캐빈 중앙)로 후진하며 버튼 벽 방향으로 다시 정렬 → 오른쪽 90°(문 정면) → 팔을 올려 층수 화면 보기 → 목표층을 연속 N번 새 프레임으로 확인 → 도착층 지도 전환·AMCL 수렴 확인 → 나가기 직전 층수 다시 확인 → 문 열림 확인 → 실제 문 틈에 맞춰 팔 접고 하차 → 하차 위치(마지막 방향은 Nav2 몫). 층이 바뀌었으면 나가지 않는다. 문이 닫혀 있으면 같은 층 버튼을 다시 눌러 연다(문 열림으로 확인) |
| `parcel_pickup` 택배 승차 | 리프트 사전 점검(움직이기 전) → `PARCEL_PICKUP_STATE` → 택배 구역 도착 확인 → 오른쪽 90° → 지게팔 내리기 → 몸체를 좌·중·우로 돌려 정해진 보기 자세로 택배 찾기 → 여러 개면 가장 오른쪽(입구에 가까운) 것 → 팔레트 0.85 m 앞에서 두 번 다시 보고 앞면 맞춤 → 지게팔을 포켓에 넣기 → 짚기 → 리프트 올리기(초음파가 택배 윗면을 보면 멈춤, 또는 조금만 올림. 매 단계 전에 택배 윗면 상한 검사) → 택배 윗면이 팔·카메라를 가리지 않는 높이인지 확인 → 그때만 '실음' 상태 |

팔은 필요할 때마다 계산해 움직이지 않는다. 보기 자세(`arm.views` → 티칭된 `front_view`/`floor_view` 등)와 누르기 사이클(`arm.press_cycle`)처럼 미리 정한 각도로만 움직이고, 방향은 몸통이 맞춘다. 버튼은 그 누르기 사이클의 팔 끝 높이(`press_tip.*.z`)에 붙인다(로비 ▲, 목표층 ▼, 층 버튼 1·2 줄).

각 상태 전환은 `states.jsonl`에 시각과 근거(카메라 결과, LiDAR 거리, 주행 결과)와 함께 남는다.

## 폴더 구성

| 파일 | 내용 |
|---|---|
| `skilltests/machine.py` | 상태기계 기반, 어댑터 계약, 실패 시 정지 |
| `skilltests/elevator_entry.py`, `elevator_ride.py`, `parcel_pickup.py` | 세 시험 |
| `skilltests/button.py` | 버튼 찾기·접근·누르기·확인 공통 절차 |
| `skilltests/control.py` | odom 기반 회전-직진-회전 주행(명령은 `/cmd_vel` → nav_safety_gate) |
| `skilltests/geometry.py`, `vision_core.py` | 기하 계산, 영상 처리·투영(시뮬과 공용) |
| `skilltests/ros_adapters.py` | 젯슨 어댑터(ROS 2 Humble) |
| `config/skill_tests.yaml` | 토픽, 속도 제한, 카메라 보정, 시험별 위치 |
| `run_test.py` | 실행기 |
| `tests/test_machines.py` | 가짜 어댑터로 하는 오프라인 시험 |

## 시뮬레이션에서 먼저 보기

같은 상태기계가 시뮬 왕복 전체에서 돈다. 장식 월드(버튼·층수 화면)와 팔 카메라를 쓴다.

```bash
cd cs_sim_e2e
python3 sim/real_maps/run_e2e.py --cabin-mode direct --arm-sim --load pallet --speed 3 --gui
python3 sim/real_maps/autonomy_smoke.py --steps pickup,entry,ride --speed 3 --gui   # 앞부분만
```

## 젯슨 준비

1. 컨테이너·빌드·카메라 구성은 `docs/deployment/08_elevator_camera_mission.md`를 따른다.
2. 기존 주행 bring-up을 띄운다: `/odom`, `/scan`, TF `map→odom`(AMCL), `nav_safety_gate`. 이 시험은 `/cmd_vel`(게이트 입력)만 발행한다.
3. 팔 노드: `ros2 launch robot_arm_pkg arm_sequence.launch.py serial_port:=/dev/arm_servo`. `--execute` 실행은 시작할 때 `home` 명령을 먼저 보내고 완료를 확인한다. 같은 포트를 여는 `tools/button_arm_test/app.py`, `servo_test.py`는 함께 실행하지 않는다.
4. 층수 인식기: `python3 tools/floor_reader/app.py --source /dev/video0 --host 127.0.0.1 --port 8765`. ROI와 층 숫자를 등록한다. 이 시험은 웹캠을 직접 열지 않고 `/frame.jpg`로 같은 영상을 받는다.
5. `config/skill_tests.yaml`의 `[현장 측정]`, `[현장 보정]`, `[현장 티칭]` 값을 채운다(아래 체크리스트).

## 실행

먼저 dry-run으로 감지만 확인한다. 주행·팔·리프트 명령은 기록만 하고 보내지 않는다.

```bash
python3 tools/jetson_skill_tests/run_test.py elevator_entry --load-state loaded                 # 로비(F1, ▲)
python3 tools/jetson_skill_tests/run_test.py elevator_entry --load-state loaded --execute
python3 tools/jetson_skill_tests/run_test.py elevator_ride --load-state loaded --execute       # F1 -> F2
python3 tools/jetson_skill_tests/run_test.py elevator_ride --load-state loaded --execute --phase exit
python3 tools/jetson_skill_tests/run_test.py parcel_pickup --execute
python3 tools/jetson_skill_tests/run_test.py elevator_entry --load-state empty --floor F2 --execute   # 목표층(F2, ▼)
python3 tools/jetson_skill_tests/run_test.py elevator_ride --load-state empty --floor F2 --execute    # F2 -> F1
```

`--load-state`(팔레트를 실었는지)는 엘리베이터 시험에 필수다. 앞 끝 길이가 달라져 접근 거리와 충돌 검사가 바뀐다.

결과는 `logs/jetson_skill_tests/<시각>_<시험>[_dryrun]/`(로컬 전용)에 남는다: `states.jsonl`, `adapter.jsonl`, `result.json`, `config_used.json`.

## 안전

- 기본이 dry-run이다. `--execute`가 있어야 움직인다. 설정 오류(허용되지 않는 팔 자세 이름, 리프트 모드 등)는 시작 전에 거부한다.
- 시작 때 odom·LiDAR·TF `map→odom`이 모두 신선한지 확인하고, `--execute`면 팔 `home`을 먼저 끝낸다.
- 모든 주행 명령은 보내기 전에 검사한다. 앞으로 갈 때는 앞 통로, 후진할 때는 뒤 통로, 제자리 회전은 짐을 포함한 몸체 사각형이 쓸리는 영역을 LiDAR로 본다. 막히면 즉시 0을 보내고 그 자리에서 끝낸다. odom·LiDAR가 오래됐거나 유효한 빔이 모자라면 '알 수 없음'으로 보고 멈춘다. LiDAR NaN은 '문 열림'이 되지 않는다.
- 팔이 접히지 않았으면(stow 실패) 주행하지 않는다. 팔 명령이 시간을 넘기면 `cancel`을 보내고, `--execute`에서 시뮬레이션 모드나 하드웨어 미연결 상태의 완료 신호는 성공으로 보지 않는다.
- 속도는 `limits.max_v`, `limits.max_w`로 다시 제한하고, 명령은 항상 `nav_safety_gate`를 거친다.
- 실패·예외·Ctrl+C가 나면 주행 정지, 진행 중인 팔 명령 취소, 리프트 현재 높이 유지를 각각 따로 시도하고 `FAILED`로 끝난다(ROS 종료 전에 정리한다).
- 리프트 토픽이 비어 있거나 신선한 위치·초음파 값이 없으면 택배 승차는 **움직이기 전에** 실패한다. 리프트는 다음 한 단계가 택배 윗면 상한을 넘을 것 같으면 명령하지 않는다.
- 문 열림은 LiDAR 로 본 문 틈으로 판단한다: 폭(`door_width_m`), 틈 안 빈 빔 10% 이하, 양쪽 문틀 거리 차(`door_jamb_range_diff_m`), 티칭한 문과의 위치·방향 일치(`door_match_*`)를 모두 만족하고 2번 연속 보여야 열림이다. 진입·하차는 그 문 틈에 맞춰 다시 선 뒤 곧게 한다. **유리문은 LiDAR 에 안 보이므로 자율 진입하지 않는다.** 문 센서 모드(`require_door_confirmation`)는 쓰지 않는다.
- 회전·곡선 주행은 실제 원호를 따라 몸체(실은 짐 포함)가 쓸고 가는 영역을 점마다 검사한다. 진행 방향 가운데 빈 LiDAR 구간이 넓으면 멈춘다. 스캔 지연·제동 거리 보정은 아직 없다(`docs/improvement_report.md` §1.39).
- 층수는 같은 영상 스트림에서 프레임 번호와 촬영 시각이 계속 늘어나는 새 프레임만 센다. 팔 자세를 바꾼 뒤 찍힌 프레임만 쓴다.

## 현장 보정 체크리스트

| 항목 | 설정 키 | 방법 |
|---|---|---|
| 티칭 자세별 카메라 자세 | `vision.views.<view>` | 바닥·벽의 표식으로 카메라 위치·yaw·pitch·화각을 맞춘다. 버튼·택배 3D 위치가 여기에 달려 있다 |
| 보는 자세 | `arm.views` | `arm_sequence`는 `front_view`/`floor_view`만 받는다. 누르는 방향에서 다시 보는 자세(`hall_press_view`·`cabin_press_view`: 누르기 사이클과 같은 받침 각도)와 택배·팔레트를 내려다보는 자세가 따로 필요하면 Kim이 `camera_views.json`과 `arm_sequence` 허용 목록을 함께 늘려야 한다. 팔 끝을 몸체 위로 세운 자세여야 실은 택배를 건드리지 않는다 |
| 누르기 팔 끝 | `tests.common.press_tip.call/destination` | 받침·손목은 1500 PWM 정면 고정, 어깨·팔꿈치만 위아래로 움직인다. 차체를 버튼판에 정면으로 세우고 '누름 유지' 순간 팔 끝 위치(바퀴축 기준 xy, 높이 z)를 잰다. yaw는 0이어야 하며, 회전이 필요한 자세는 거부한다. 버튼판은 이 높이에 붙인다 |
| 버튼판 높이 | 시연 버튼판 부착 | 로비는 ▲, 목표층은 ▼ 캡 중심, 층 버튼판은 1·2 줄이 `press_tip.*.z` 에 오게 붙인다(▲·▼ 간격 8.5 cm 라 한 사이클로 둘 다 못 누름) |
| 앞 끝 | `robot.front_extent_*` | 빈 지게팔, 팔레트를 실은 상태 |
| 누름 확인 | `tests.common.confirm_by` | 기존 팔 카메라만 사용. 호출은 `[door]`와 후진 뒤 캡 재인식, 층 버튼은 새 목표층 영상 확인을 필수로 한다. 40 s 무응답이면 추가 1회 입력. 표시창 미확인/재시도 소진은 실패 |
| 멈춘 자리 허용 범위 | `tests.common.press_envelope_m` | 누르기 사이클이 버튼에 맞는 거리·옆·방향 오차 |
| 문 | `door_centre_map`, `door_normal_map`, `door_width_m` | 문 중심과 문 선에 수직인 방향(층마다). 실제 진입·하차는 LiDAR 로 본 문 틈에 맞추므로 대략이면 된다. 문 폭 범위 밖이면 문으로 믿지 않는다 |
| 캐빈 정지점 | `cabin_centre_map` | 정지점에서 오른쪽 90° 돌았을 때 층 버튼 패널이 팔 카메라 화면 가운데 근처에 와야 한다. 패널이 문 모서리 쪽에 있으면 짐 실은 로봇(폭 약 0.44 m)이 마주 설 자리가 없을 수 있으니 정지점을 패널 쪽으로 옮긴다 |
| 층 전환 | `floors.*` | orchestrator 상태 토픽·서비스, AMCL 공분산 기준 |
| 택배·팔레트 색 | `vision.parcel_hsv`, `pallet_hsv` | 실제 택배·팔레트 |
| 위치 | `tests.*_map` | 지도 위 버튼·화면·문 위치(waypoint 이름 참조 가능) |
| 리프트·초음파 | `lift.*_topic`, `ultrasonic_z_m` | 리프트 드라이버와 초음파 토픽이 생기면 연결 |

## 시뮬에서 확인한 것과 현장에서만 확인할 것

- **같음**: 상태 순서, 판단 조건(연속 프레임, LiDAR 문 열림, 정렬 범위), 주행 제어기, 영상 처리·투영 코드, LiDAR 벽 찾기(버튼 방향 광선이 닿는 점을 지나는 직선만 쓰고 이웃한 벽은 섞지 않음, 벽 길이 0.25 m 미만이면 못 찾은 것으로 봄).
- **다름**:
  - 시뮬 팔은 가정 기하의 역기구학으로 버튼 면에 닿는다. 젯슨은 티칭된 누르기 사이클을 쓴다. 따라서 계산한 자리에 섰을 때 사이클이 버튼에 맞는지는 현장 티칭으로 확인해야 한다.
  - 시뮬 버튼은 파란 LED, 팔레트는 초록이라는 색 약속을 쓴다. 젯슨 버튼은 `button_arm_test/vision.py`의 흰 원형 캡과 숫자·화살표 판독을 쓴다.
  - 시뮬 층수는 7-세그먼트 판독, 젯슨은 `floor_reader` 템플릿 판독이다.
  - 시뮬 택배 짚기는 attach mock이다(접촉·하중 미검증).
- **아직 없음**: 리프트 드라이버 토픽, 몸체 앞 초음파, 실제 팔 역기구학, 지게팔 체결 감지.

## 오프라인 시험

```bash
python3 tools/jetson_skill_tests/tests/test_machines.py
```

## 누름 확인 범위 (2026-10-14)

사용자 지정: 실물 버튼은 종이컵이지만 시뮬에서는 누르면 해당 버튼 불이 켜진다고 가정한다. 시뮬 어댑터는 기존 팔 카메라 영상에서 요청한 버튼 위치의 램프 패치만 확인한다. 층 표시창은 버튼 누름 성공 근거로 쓰지 않으며 도착층 확인에만 쓴다.

실물 종이컵에는 램프가 없으므로 시뮬 확인 결과를 물리 눌림 검증으로 해석하지 않는다. 현장 어댑터의 눌림 확인 방식은 별도 과제다.
