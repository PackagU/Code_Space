# 기존 주행과 카메라 웹사이트를 이용한 목표층 배송

## 동작과 지도 연결

기존 2026-09-16 현장 지도·배송 좌표·Nav2 설정을 사용한다. 같은 `8091` 웹사이트에서 버튼 등록·보정·누르기 시험과 층수 표시창 인식을 수행한다. 웹캠은 한 프로세스만 열고 같은 프레임을 두 인식기에 전달한다.

| 항목 | 설정 |
|---|---|
| 출발 지도 | 기존 `F1` |
| 출발 위치 / 택배 위치 | `f1_initial_test` / `f1_locker` |
| 목적지 지도 | 기존 `F2` |
| 실제 목적층 버튼·표시층 | **4 / F4** (`F2` 지도는 실제 4층) |
| 배송 위치 | 기존 `f2_delivery_left_room4` |
| 버튼·층수 웹사이트 | `http://127.0.0.1:8091` |
| 미션용 현재 층수 API | 같은 서버의 `/floor/api/state` |

자동 흐름은 다음과 같다.

```text
카메라·팔 연결 확인 → home → 정면 자세 → 택배 위치 → 엘리베이터 앞
→ 웹 인식·보정 자세로 호출 버튼 누르기 → 정면 자세 → 내부 waypoint
→ 웹 인식·보정 자세로 4층 버튼 누르기 → 층수 자세 → 4층 연속 확인
→ 기존 F2 지도 로드·초기 위치 설정 완료 → 정면 자세 확인 → 하차 → 기존 배송 위치
```

`prepare_camera_delivery.py`는 기존 `waypoints.json`, `map_pins.json`에서 설정을 만든다. 좌표를 새로 추정하지 않고 x/y를 그대로 복사하며 yaw의 단위만 변환한다. 원본 지도·좌표 파일은 수정하지 않는다. 다른 기존 배송 위치는 `--destination`으로 선택할 수 있다. 내부·출구 좌표 중 지도 이미지에서 계산된 값은 현장에서 실차 위치·방향을 확인해야 한다.

기존 택배 적재·수령 단계의 `WaitForAck`는 mock ack 동작을 유지한다. 실제 적재·수령 센서가 자동 확인되는 기능은 추가하지 않았다. 기존 현장 프로파일은 엘리베이터 탑승·하차를 수동으로 시연했으며, 이번 자동 전환은 별도 실차 검증 대상이다.

## 팔과 버튼 인식

| 자세 | 000 | 001 | 002 | 003 |
|---|---|---|---|---|
| front_view | 1500 | 1500 | 2000 | 1500 |
| floor_view | 1500 | 1480 | 1670 | 1500 |
| ROS home | 1500 | 1100 | 2400 | 1500 |

자세 이동은 기본 2000ms이며 `src/robot_arm_pkg/config/camera_views.json`을 사용한다. 팔 노드가 재시작되면 home 응답 확인부터 다시 해야 한다.

단일 `arm_sequence` 노드가 서보 포트를 소유한다. 웹사이트는 `--ros-arm`으로 실행하여 ROS 명령을 보낸다. 같은 포트를 직접 여는 `--execute`, `servo_test.py`, 다른 팔 메뉴 프로그램을 함께 실행하지 않는다.

자동 미션도 웹사이트의 현재 목표 버튼·안정된 인식·보정된 PWM을 사용한다. 숫자 1/4는 기존 메뉴 6/7 자세, ▲/▼는 웹사이트에서 저장한 자세, 숫자 2/3은 픽셀 보정점을 사용한다. 인식 실패·카메라 종료·오래된 프레임·중복 버튼·보정 미등록 상태에서는 누르기를 시작하지 않는다. 버튼 탐색은 최대 45초, 팔 명령 응답은 최대 30초 기다린 뒤 미션을 중단한다.

누르기는 기존 ROS 접근·누르기·유지·후퇴·home 순서를 사용하되 웹사이트의 누름 자세와 접근 오프셋을 전달한다. ROS home은 웹 독립 시험의 home과 다를 수 있다. 기존 직접 시리얼 `--execute` 방식은 독립 시험용으로 유지한다. 관절 완료는 컨트롤러 위치 응답 기준이며 실제 버튼 접촉이나 엔코더 측정을 증명하지 않는다.

## Jetson 업데이트와 카메라 준비

Jetson 호스트에서 실행한다. 보정·숫자 등록 파일을 현장에서 수정했다면 `git status`에 나온 변경을 보관하고 병합한다. `reset --hard`로 덮어쓰지 않는다.

```bash
cd ~/Code_Space-2026-10-06
git status --short --branch
git pull --ff-only origin 2026-10-06
```

8091 독립 카메라 앱과 8765의 이전 인식기는 먼저 종료한다. 이전 Jetson 홈 서비스가 실행 중이면 다음 명령으로 멈춘다.

```bash
systemctl --user stop floor-reader.service
```

기존 `packagu_camera_test` 컨테이너에 새 저장소 전체가 `/ros2_ws`로 마운트되어 있고 `/dev/arm_servo`, `/dev/video0`가 전달되어 있으면 그대로 사용한다. 카메라·팔만 확인할 때는 아래 절차로 충분하다.

Compose 기반 전체 주행 컨테이너에는 새 카메라 overlay를 추가할 수 있다. **기존 LiDAR·OpenCR·팔 장치 설정과 지도 마운트를 유지**하고, 프로젝트 루트에서 실행한다. 새 Git 체크아웃에는 gitignore된 PGM 지도 이미지가 없을 수 있다. 기존 실제 지도 폴더를 `PACKAGU_MAPS`로 전달하고 `/ros2_ws/maps/field`에 필요한 지도·포인터·좌표·pin 파일이 있는지 확인한다.

```bash
export CAMERA_DEVICE=/dev/video0
export ARM_SERVO_DEVICE=/dev/arm_servo
docker compose -f docker/compose/docker-compose.jetson.yml \
  -f docker/compose/docker-compose.jetson.camera.yml up -d ros2
```

이 overlay는 카메라를 `/dev/packagu_camera`로 전달한다. 별도 `--profile camera`의 `floor_reader`를 함께 기동하지 않는다. 컨테이너를 재생성했다면 기존 베이스와 Nav2도 다시 시작한다. CSI 카메라 구성은 이 USB 예시에 포함하지 않는다.

## 같은 사이트에서 먼저 수동 시험

컨테이너 첫 터미널에서 팔 노드만 시작한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm_pkg --parallel-workers 1
source install/setup.bash
ros2 launch robot_arm_pkg arm_sequence.launch.py serial_port:=/dev/arm_servo \
  camera_pose_config:=/ros2_ws/src/robot_arm_pkg/config/camera_views.json
```

컨테이너 두 번째 터미널에서 웹사이트를 실행한다. `packagu_camera_test`는 `/dev/video0`, 카메라 overlay는 `/dev/packagu_camera`를 사용한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
python3 tools/button_arm_test/app.py --source /dev/video0 \
  --host 127.0.0.1 --http-port 8091 --ros-arm
```

Windows PowerShell에서 SSH 전달을 유지한다. VS Code SSH의 Ports에서 원격 8091을 전달해도 된다.

```powershell
ssh -N -L 18091:127.0.0.1:8091 hsm@192.168.0.7
```

브라우저에서 `http://localhost:18091`을 연다.

1. `home` 완료 후 `팔 정면 보기`를 확인한다.
2. 기존 숫자 등록·버튼 보정·▲/▼ 자세 저장·누르기 시험을 같은 사이트에서 수행한다.
3. `팔 층수 보기`를 선택한다. 001=1480, 002=1670 자세와 표시창이 보이는지 확인한다.
4. `층수 영역 다시 지정` 후 표시창 네 모서리를 좌상 → 우상 → 우하 → 좌하로 클릭한다. 필요한 실제 층수 글자를 해당 이름으로 등록한다.
5. 목표 표시층을 `4`로 적용한다. 3층에서는 조건 대기, 4층을 연속 확인하면 조건 성립, 다른 층·가림·영상 끊김에서는 다시 대기해야 한다.

숫자 버튼 등록과 표시창 글자 등록은 서로 다른 템플릿이다. 웹사이트의 `버튼 인식 화면` / `층수 인식 화면`은 표시 방식만 바꾸며, `팔 정면 보기` / `팔 층수 보기`가 실제 팔 자세를 바꾼다. 등록·보정 데이터는 기존 `tools/button_arm_test/data`, `tools/floor_reader/data`에 보존한다.

이 수동 시험은 하차 명령을 보내지 않는다. 같은 미션 조건을 터미널에서 확인하려면 다음 명령을 사용한다.

```bash
python3 scripts/floor_arrival_probe.py --target 4 \
  --reader-url http://127.0.0.1:8091/floor/api/state
```

`motion_authorized`는 항상 false다. 같은 사이트의 `/floor/test`에서 시험용 숫자를 열어 모니터에 표시할 수도 있다. ROI·버튼 접촉 자세 등록은 실제 영상과 접촉 확인이 필요하므로 수동으로 수행한다.

## 기존 베이스·Nav2에 자동 미션 추가

수동 시험의 웹사이트와 팔 launch를 Ctrl+C로 종료한다. 원래 `start_field_navigation.sh`는 팔 노드가 켜져 있으면 시작을 거부하므로 **기존 베이스·Nav2를 먼저 기동하고 새 미션을 마지막에 추가**한다.

Jetson 호스트의 기존 주행 터미널 두 개에서 실행한다. 기존 실행 당시 사용했던 장치·IMU·속도 설정은 그대로 사용한다. 구동부·LiDAR가 현재 연결되지 않았다면 전체 미션을 실행하지 말고 위 수동 시험만 수행한다.

```bash
# 첫 터미널: 기존 현장 베이스
ENABLE_DRIVE=1 bash scripts/start_field_base.sh
```

```bash
# 두 번째 터미널: 기존 F1 지도 Nav2, 목표는 아직 보내지 않음
FLOOR=F1 bash scripts/start_field_navigation.sh
```

기존 컨테이너 이름이 `packagu_camera_test`라면 두 명령 앞에 `PACKAGU_CONTAINER_NAME=packagu_camera_test`를 붙인다. 기존 초기 위치 지정과 위치 추정 확인도 수행한다.

세 번째 터미널은 ROS 컨테이너 안에서 실행한다. 필요한 패키지 빌드·설정 생성·팔·카메라 웹사이트·층 인식 bridge·맵 전환·미션을 한 줄로 시작한다. **이 명령부터 실제 주행 목표가 발행된다.**

```bash
cd /ros2_ws
bash scripts/start_camera_delivery.sh --target-map F2 --display-floor 4
```

카메라 overlay는 `CAMERA_SOURCE`를 설정하므로 같은 스크립트를 그대로 사용한다. 다른 장치는 `ARM_SERVO_PORT`, `CAMERA_SOURCE`로 선택한다. 시작 조건은 기존 `/nav_safety_gate`, `/bt_navigator`, `/map_server` 존재와 지도 pin 검사 통과다. 팔·웹·맵 전환·미션 노드 중복도 거부한다. 각 Nav2 목표 전에는 기존 `field_map_guard.py`로 실제 로드된 지도·PGM/YAML hash·포인터를 다시 확인한다. 기본 현장 경로는 기존 목표 좌표를 Nav2에 전달하고, 기존 데모의 직각 waypoint 라우터는 별도 옵션으로 유지한다.

코드·지도 선택만 검토하려면 주행 없이 설정을 생성한다.

```bash
bash scripts/start_camera_delivery.sh --prepare-only --target-map F2 --display-floor 4
```

생성 파일은 `logs/camera_delivery/config`에 저장한다. 목적지는 `--destination`, 출발 위치는 `--start`, 택배 위치는 `--pickup`으로 기존 waypoint 이름을 지정한다. 지도 전환 API의 `current_floor=F2`와 카메라의 `observed_floor=F4`는 의도적으로 별개다.

자동 미션 중에는 웹사이트의 수동 조그·자세 변경·누르기를 막고, 화면에 목표 표시층·배송 지도·현재 단계를 표시한다. `중지`는 미션 중단과 팔 취소를 요청한다. 전체 미션을 재시작하려면 기존 미션 launch를 종료하고 위치·팔 상태를 확인한 뒤 다시 실행한다. 비상 정지 후 실제 정지는 현장에서 확인한다.

## 목표층 조건과 주행 억제

층수 자세 완료 후 0.5초가 지난 영상부터 목표층을 **새 프레임 5개 연속**으로 확인한다. 영상 최대 나이는 0.8초, 일치 점수는 0.70 이상, 차순위 점수와 차이는 0.08 이상이다. 다른 층·UNKNOWN·카메라 종료·재시작·중복 프레임은 확인 횟수를 초기화하거나 늘리지 않는다. 과거 `event` 감지 기록은 하차 근거로 사용하지 않는다.

실제 맵 전환 완료까지 목표층 인식을 유지한 후 정면 복귀를 시작한다. 정면 자세가 완료되어야 하차 Nav2 목표를 보낸다. 정면 전환 중 확인 근거는 최대 30초 유지한다. 만료 시 하차 전에는 다시 층수를 확인하고, 하차 시작 후에는 goal을 취소하고 정지를 유지한다. 준비·하차 총 제한은 360초다.

기본 `require_door_confirmation=false`는 현재 카메라만 쓰는 구성이다. 층수 숫자는 문 열림이나 엘리베이터 정지를 판별하지 않는다. 문·장애물 처리는 기존 LiDAR/Nav2에 맡긴다. `/elevator/camera_state`의 `door_state=camera_confirmed`는 지도 전환용 내부 확인 값이다. 실제 문 입력이 준비되면 `require_door_confirmation:=true`와 `/elevator/door_state`의 다음 실제 센서 관측을 사용한다.

```json
{"source":"sensor","stamp":0,"door_state":"open","stopped":true,"exit_clear":true}
```

`stamp`는 매 관측의 UNIX 초로 갱신하며 권장 10Hz, 최대 나이 0.8초다. 선택형 문 모드에서 관측이 끊기거나 문이 닫히면 하차 허가를 해제한다. 팔 이동·층수 대기 중에는 기존 `/mission/drive_inhibit`로 주행을 억제한다. 베이스가 기존 `nav_safety_gate`를 거쳐야 이 기능이 적용된다.

## 검증

2026-10-09 개발 환경에서 기존 버튼 시험 19개, 공유 카메라 HTTP·보정 재저장 시험 3개, 층 인식기 6개, 새 현장 설정·ROS 웹 제어 시험 6개와 기존 팔·층수 gate 시험을 확인했다. 관련 ROS 패키지 5개 colcon 빌드와 launch 인자 확인도 수행했다.

실제 ROS/DDS·HTTP·버튼/층수 인식을 사용한 가짜 장치 통합 시험에서는 3층 대기 → 4층 확인 → F2 지도 전환 → 정면 자세 → 하차 → 기존 배송 좌표 순서를 통과했다. 가짜 모터·Nav2·맵 서버를 사용했으므로 실물 버튼 접촉·엘리베이터 문·Jetson 실제 통합 주행 성공을 의미하지 않는다.

```bash
bash scripts/run_camera_delivery_checks.sh
```

이 검사에는 실제 장치를 열지 않는 ROS 통합 시험이 포함되며 별도 시험 domain 229를 사용한다. 해당 domain이 사용 중이면 중단한다. ROS가 없으면 순수 인식·계약 시험만 수행한다. 전체 저장소 회귀 검사의 기존 Nav2/펌웨어 검사 기준 불일치와 환경별 제외 사항은 `docs/improvement_report.md` 1.28에 기록한다. 실제 탑승·층 인식·하차·배송의 연결 시험은 현장에서 수행해야 한다.
