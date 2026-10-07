# 카메라 자세 전환과 목표층 하차

## 구현한 흐름

정면 보기 → 엘리베이터 앞 주행 → 호출 버튼 조작 → 정면 보기 → 탑승 → 목적층 버튼 조작 → 층수 보기 → 목표층 연속 확인 → 실제 맵 전환 → 정면 보기 → 하차 → 목적지 주행.

`elevator_camera_mission.launch.py`가 새 실제 카메라 모드다. 기존 `elevator_mission_demo.launch.py`는 시뮬레이션 모드로 유지한다. 새 모드에는 엘리베이터 시뮬레이터가 없다. 주행 베이스와 Nav2는 기존 실제 주행 launch로 별도 실행한다.

## 팔 자세

`src/robot_arm_pkg/config/camera_views.json`에 사용자가 제공한 값을 저장했다.

| 자세 | 000 | 001 | 002 | 003 |
|---|---|---|---|---|
| front_view | 1500 | 1500 | 2000 | 1500 |
| floor_view | 1500 | 1480 | 1670 | 1500 |

두 자세의 기본 이동 시간은 2000ms다. 기동 후 기존 home 확인을 먼저 수행하고 front_view로 이동한다. 버튼 사이클이 home으로 복귀한 다음에는 다시 front_view 또는 floor_view를 명시적으로 실행한다. 실제 관절 응답이 일치해야 다음 단계로 진행한다. 이 응답만으로 엔코더 검증이나 실제 버튼 눌림을 증명하지는 않는다.

하나의 `arm_sequence` 노드가 서보 포트를 사용한다. 같은 포트를 여는 `tools/button_arm_test/app.py`, `servo_test.py`, 다른 팔 메뉴 프로그램을 함께 실행하지 않는다. 새 미션의 호출/목적층 버튼 동작은 기존 ROS 사이클 1/2를 사용하며, `call_press_cycle`, `destination_press_cycle`로 선택한다. 이 두 사이클의 실제 버튼 위치 보정은 기존 `servo_protocol.py`에서 확인해야 한다. 카메라 버튼 시험 UI가 ROS 미션에 자동 연결된 것으로 해석하면 안 된다.

## 층수 카메라

기존 `tools/floor_reader/app.py`의 ROI·숫자 등록 화면을 사용한다. 카메라를 floor_view에 놓았을 때 표시창의 네 모서리를 다시 지정하고 실제 각 층의 숫자를 등록한다. 표시창용 인식기이며 버튼의 인쇄된 숫자를 읽는 `button_arm_test/vision.py`와는 다르다.

USB 카메라 예시이며 `/dev/video0`는 실제 장치로 변경한다. CSI 카메라의 드라이버/GStreamer 구성은 이 USB 프로파일에 포함하지 않는다.

```bash
export PACKAGU_UID=$(id -u)
export PACKAGU_GID=$(id -g)
mkdir -p tools/floor_reader/data
export ARM_SERVO_DEVICE=/dev/arm_servo
export FLOOR_CAMERA_DEVICE=/dev/video0
export FLOOR_CAMERA_GID=$(stat -c %g "$FLOOR_CAMERA_DEVICE")
export FLOOR_READER_SOURCE=/dev/floor_camera
docker compose -f docker/compose/docker-compose.jetson.yml --profile camera up -d ros2 floor_reader
```

`ARM_SERVO_DEVICE`는 호스트의 실제 팔 장치로 지정한다. 기존 주행에 필요한 LiDAR·MCU 장치 설정도 유지한다. 컨테이너를 재생성했다면 기존 베이스·Nav2를 다시 기동한다.

Jetson의 `http://127.0.0.1:8765`에서 ROI와 숫자 템플릿을 설정한다. `floor_reader_bridge`는 `/api/state`의 현재 프레임을 `/elevator/vision_floor`에 전달한다. 과거 `event`는 하차 판단에 사용하지 않는다. 같은 프레임을 여러 번 읽어도 연속 확인 횟수가 늘지 않는다.

## 카메라만 사용하는 기본 동작

사용자 요청에 따라 `require_door_confirmation=false`가 기본값이다. 별도 문 센서나 `/elevator/door_state` 발행 노드가 필요하지 않다. 탑승 완료는 엘리베이터 내부 waypoint에 대한 Nav2 성공 결과로 판단하고, 그 후 팔을 floor_view로 이동한다. 다른 층 또는 UNKNOWN이면 정지 상태를 유지한다. 영상이 끊기거나 오래된 영상이면 목표층 확인 횟수를 초기화한다.

층수 자세 이동 완료 후 0.5초가 지난 영상부터 같은 목표층을 5개 새 프레임에서 확인한다. 확인 영상의 최대 나이는 0.8초다. 맵 전환 완료까지 층수 인식을 계속하고, 완료 시에도 목표층이 확인되어야 정면 복귀를 시작한다. 정면 복귀 동안에는 확인 결과를 최대 30초 유지하여 하차 주행을 허용한다. 시간이 만료되면 하차 시작 전에는 다시 층수를 확인하고, 하차 시작 후에는 goal을 취소하고 정지한다. 360초 안에 준비·하차가 끝나지 않으면 미션을 중단하고 정지를 유지한다.

카메라의 목표층 숫자는 문 열림이나 엘리베이터 정지를 판별하지 않는다. 기본 모드는 목표층 확인으로 하차 주행을 요청하고, 닫힌 문과 장애물에 대한 정지·회피는 기존 LiDAR/Nav2 설정에 맡긴다. 이 모드의 `/elevator/camera_state`에서 `door_state=camera_confirmed`는 기존 맵 전환 인터페이스에 전달하는 내부 확인 표시이며 실제 문 열림을 뜻하지 않는다.

베이스는 기존 `nav_safety_gate`의 `/mission/drive_inhibit`를 거쳐야 한다. 이 게이트를 우회해 `/cmd_vel`을 직접 모터로 전달하면 층수 대기 중 주행 억제가 적용되지 않는다. 팔 이동과 대기 중에는 주행을 억제한다.

## 선택 사항: 문 상태를 함께 확인하는 모드

나중에 실제 센서를 추가하는 경우에만 `require_door_confirmation:=true`를 지정한다. 이 모드는 탑승·하차에 문 열림·정지·통과 경로 확인을 추가로 요구한다.

이 옵션을 켠 경우 실제 센서 어댑터가 `/elevator/door_state`에 `std_msgs/String` JSON을 주기적으로 발행해야 한다. 권장 10Hz, 최대 관측 나이 0.8초다. `stamp`는 해당 센서 관측의 UNIX 초이며 매 새 관측마다 갱신한다. 아래 0 값은 형식 설명용이다.

```json
{"source":"sensor","stamp":0,"door_state":"open","stopped":true,"exit_clear":true}
```

`stopped`는 엘리베이터 정지, `exit_clear`는 통과 경로 확보를 의미한다. 센서가 없는 현재 구성에서는 이 옵션을 켜지 않는다. 이 모드에서 문이 닫히거나 관측이 끊기면 하차 전에는 다시 층수를 확인하고, 하차가 이미 시작되었다면 goal을 취소하고 정지한다.

## 실행

컨테이너 안에서 빌드한다. 실제 실측 지도와 waypoint 설정이 준비되어 있어야 한다. 아래 경로는 컨테이너에 마운트된 프로젝트 기준이다.

```bash
cd /ros2_ws
colcon build --symlink-install --base-paths src test_workspace/elevator_mission/src test_workspace/elevator_auto_map_switch/src
source install/setup.bash
ros2 launch elevator_mission_pkg elevator_camera_mission.launch.py serial_port:=/dev/arm_servo mission_workspace:=/ros2_ws/test_workspace/elevator_mission floor_maps_yaml:=/ros2_ws/test_workspace/elevator_auto_map_switch/config/floor_maps.yaml
```

`mission_id`의 YAML `target_floor`가 목표층이다. `mission_workspace/config/kku_nav_points.yaml`과 지정한 `floor_maps_yaml`을 실제 지도/탑승·내부·출구 좌표로 맞춘다. 기본 예시 좌표를 실측 좌표로 간주하지 않는다. 팔 설정 파일은 `camera_pose_config`, 층 인식 HTTP 주소는 `reader_url`로 변경할 수 있다. 선택형 문 입력 토픽은 `door_state_topic`으로 변경한다.

새 launch는 `dry_run_map_load=false`로 실제 map_server 응답을 기다린다. 기존 주행 launch에서 팔 노드나 floor orchestrator를 함께 실행 중이라면 중복 실행을 제거한다. 이 두 노드는 새 미션 launch가 실행한다.

## 오프라인 검증

```bash
python3 scripts/test_elevator_camera.py
```

가짜 장치로 문 입력 없는 목표층 하차, 잘못된 층, 중복 프레임, 카메라 재시작, 영상 끊김, 팔 응답 불일치, 맵 전환/정면 자세 이전 하차 방지, Nav2 ABORT 실패를 검사한다. 선택형 문 모드의 미래/NaN 시각, 문 닫힘·재열림, 오래된 문 입력, 하차 중 goal 취소도 검사한다. 실제 버튼 접촉·Jetson ROS 기동·물리 하차는 현장에서 검증해야 한다.

2026-10-06 Windows 오프라인 검증: 새 미션 테스트 13개와 floor_reader 테스트 4개 통과. 초기 관련 검사에서 기존 Nav2 허용오차 기준 불일치가 발견됐고, 사용자 요청으로 `test_nav2_orthogonal_tuning.py`를 현재 현장 설정에 맞췄다. 위치 허용오차는 양수이면서 0.20m 이하, 방향 허용오차는 양수이면서 0.15rad 이하를 요구한다. `nav2_params.yaml`의 실제 주행 설정은 유지했으며 관련 검사 스크립트 15개 모두 재검증을 통과했다. ROS 실행·colcon 빌드·실물 주행은 이 Windows 환경에서 검증하지 못했다.
