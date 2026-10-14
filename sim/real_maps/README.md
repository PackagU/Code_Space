# 실측 지도 Gazebo 시뮬 (sim/real_maps)

모든 판정은 **시뮬레이션 검증**이다. 실물 검증으로 쓰지 않는다. 월드 치수 대부분이 아직 모름(임시값)이라 결과에는 "임시 치수"를 붙인다. 현장 기본 파일(`nav2_params.yaml`, `nav_safety.yaml`, `drive_calib.yaml`, `fieldctl`, `start_field_*.sh`, `map_pins.json`, `latest_map.txt`, `waypoints.json`)과 이성덕 미션 코드는 고치지 않는다.

## 안전: 실제 로봇과 섞이지 않게

같은 LAN에 Jetson이 있다. 모든 시뮬은 `sim_container.sh`가 만든 격리 컨테이너 안에서만 돈다.

- Docker `--internal` 네트워크 `packagu_sim_isolated` (LAN·인터넷 경로 없음)
- `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=77`, 장치 전달·privileged·host network 없음
- 위 조건이 하나라도 빠지면 셸(`sim_container.sh verify`)과 파이썬(`isolation.assert_isolated`) 양쪽에서 기동을 거부한다
- `scripts/fastdds_lan_peers.xml`, `scripts/run_sim_host.sh`(Jetson 분산 모드)는 쓰지 않는다
- 컨테이너는 호스트 uid로 돌아 root 소유 파일을 남기지 않는다. 필드용 `ros2_humble` 컨테이너는 건드리지 않는다

## 바로 실행 (복사해서 쓰기)

저장소 루트(`cs_sim_e2e/`)에서 실행한다. 결과는 `logs/real_map_sim/<날짜시간>_e2e_...`에 새로 쌓인다. 각 왕복은 실제 약 18~22분이다.

```bash
# 1) 검증된 기본 왕복 (캐빈 진입·하차만 시뮬 운전자 직접 주행, 나머지 Nav2) — 2026-10-08 완주 확인
python3 sim/real_maps/run_e2e.py --cabin-mode direct

# 2) 팔·지게팔 포함 왕복 (리프트 상승·하강, 팔 버튼 누르기·접기, 상자 붙이기/놓기 mock, 모두 가정값)
python3 sim/real_maps/run_e2e.py --cabin-mode direct --arm-sim

# 3) 전 구간 Nav2 왕복 (캐빈 진입·하차도 Nav2) — 10/8 최신 모델에서 F2 캐빈 진입 실패 재현용
python3 sim/real_maps/run_e2e.py --cabin-mode nav2

# 4) GUI로 보며 한 층 띄우기 (gzclient + RViz, 컨테이너 안에서 같은 uid X11 소켓 사용, xhost 변경 없음)
#    10/8 확인: RViz 지도·costmap·LiDAR·로봇 20 fps, Gazebo 3D 51 FPS (DISPLAY=:1, SI:localuser 허용 환경)
bash sim/real_maps/start_sim.sh --floor F1 --params P0 --spawn f1_initial_test --gui
#    두 층 + 엘리베이터 + 팔: --world building --arm-sim 추가
python3 sim/real_maps/sim_goal.py --name <출력된 이름> --to f1_locker

# 5) 자율 왕복 시연: 사람 개입 없이 로봇이 카메라로 찾고 팔로 누르고 지게팔로 집는다 (아래 "자율 시퀀스")
#    3배속 + 장식 월드(자동) + 팔레트 택배 2개 + GUI (Gazebo 추적 카메라·RViz·팔 끝 카메라 창, 화면 왼쪽 절반 자동 배치)
python3 sim/real_maps/run_e2e.py --cabin-mode direct --arm-sim --load pallet --parcels 2 --speed 3 --gui

# 5-1) 짧은 자율 점검 (왕복 아님): 택배 집기 → 엘리베이터 입장 → 탑승(내부 버튼·층 확인·하차)
python3 sim/real_maps/autonomy_smoke.py --steps pickup,entry,ride --speed 3 [--gui]

# 6) 종료: 이 시뮬이 띄운 프로세스·컨테이너만 정리 (현장 ros2_humble 컨테이너는 건드리지 않음)
bash sim/real_maps/start_sim.sh --stop
for c in $(docker ps -a --format '{{.Names}}' | grep '^packagu_sim'); do docker rm -f "$c"; done
```

시연 옵션:

- `--speed X`: 목표 실시간 배율. 물리 step(1 ms)은 그대로 두고 Gazebo `real_time_update_rate`만 측정하며 맞춘다. 2026-10-12 측정에서 전체 스택(Nav2 포함)은 제한을 풀면 약 5.9배까지 나왔고, 1배속이 0.92배였던 것은 1000 Hz 대기 때문이었다. step을 키워 빠르게 하면(5 ms) 엘리베이터 안 정지 중 미끄럼으로 odom 연속성 기준을 넘었다(0.0203 m > 0.02 m). 1이 아닌 배속 실행은 `EXPLORATORY.txt`를 남겨 1배속 통계와 분리한다. 목표 시간 제한은 모두 시뮬 시간 기준이라 배속과 무관하다
- `--dressed`: `building_dressed.world`. 캐빈 바닥·스테인리스 벽 마감·손잡이·조작반, 승강장 호출 버튼·문틀, F1 픽업 구역 바닥 테이프, F2 배달 매트. 모두 `<visual>`만 있고 충돌체가 없어 LiDAR(ray 센서)·물리·판정은 `building.world`와 같다. 생성 때 두 월드의 충돌체가 같은지 검사하고 다르면 멈춘다. 위치·치수는 `world_params.yaml dressing`의 [가정값]
- `--load pallet` (기본): f1_locker 정차 자세 앞 바닥에 2방향 팔레트(가운데 포켓 폭 0.32 m·높이 0.10 m)와 택배. 리프트 0에서 캐리어가 포켓에 들어간 상태로 붙이고 0.08 m 들어 운반(높이 유지), 배달 지점에서 0으로 내린 뒤 놓는다. 보관함은 모델하지 않는다. `--load box`는 10/8 선반 상자 방식(재현용)
- `--parcels 1|2|3`: 팔레트 위 택배 수(3번째는 첫 상자 위에 쌓임). 모두 attach/detach mock이라 접촉·하중·포켓 정렬은 검증하지 않는다. Nav2 도착 오차(약 0.3 m)만큼 팔레트가 캐리어로 순간 이동하며 그 거리는 `attach_gap_m`으로 기록된다
- `--gui`: gzclient(카메라가 로봇을 위에서 추적) + RViz + `--arm-sim`이면 팔 끝 카메라(`/arm_camera/image_raw`, `rqt_image_view`). Gazebo에는 카메라 시야 범위가 선으로 그려진다. 카메라 센서는 화면(GL)이 있어야 렌더링되므로 GUI 실행에서만 `robot_arm_cam.urdf`를 쓴다. 카메라 장착 위치·화각·해상도는 [가정값]. 호스트 X11 소켓을 같은 uid로 마운트(xhost 변경 없음). 창 배치는 호스트의 `arrange_gui.py`(Wnck)가 이 컨테이너 창만 옮긴다

- `--lift-stop ultrasonic|fixed`: 운반 높이 정하는 방식. `ultrasonic`(기본)은 몸체 앞 가상 초음파(마스트 x 0.15 m·높이 0.55 m, 시뮬은 적재물 상자와의 기하 광선 계산)가 택배 윗면을 보면 리프트를 멈춘다. `fixed`는 정해진 높이만 든다. 어느 쪽이든 택배 윗면 상한 0.60 m(`lift.carry`, 팔·카메라를 가리지 않는 높이 [가정값])를 넘지 않는다

결과 표 다시 만들기: `python3 sim/real_maps/summarize_runs.py --pretest 'pre3_*' --e2e '*e2e*' --out results/20261008`

## 자율 시퀀스 (`--arm-sim --load pallet`)

Nav2 구간 사이의 모든 동작을 `tools/jetson_skill_tests/skilltests/`의 상태 머신이 한다. Jetson 현장 시험과 **같은 코드**이고, 시뮬은 `sim_adapters.py`(SimIO), 현장은 `ros_adapters.py`(RosIO)만 다르다. 운전자 직접 주행·mock 버튼·문 명령은 쓰지 않는다. 로봇이 쓰는 정보는 팔 카메라 영상, 관절, encoder odom, LiDAR, 사전에 가르친 위치(prior)뿐이다.

| 단계 | 상태 순서 |
|---|---|
| 택배 집기 | 보관 구역 도착 → 오른쪽 90° → 지게팔 내림 → 몸체를 0°·±20°로 돌리며 고정 보기 자세로 택배 찾기 → 가장 오른쪽(입구 쪽) 택배 → 팔레트 앞면 두 번 보고 정렬 → 포켓 진입 → 붙이기 → 초음파로 멈출 때까지 들기 |
| 엘리베이터 입장 | 버튼 구역 → 고정 자세로 호출 버튼판 찾기(로비 ▲, 목표층 ▼) → 누르기 사이클 팔 끝이 버튼에 닿는 정지 자세 계산 → 정렬점에서 다시 보기 → 곧게 접근 → 착지 오차 확인(깊이만 조금 틀리면 앞뒤 조정) → 티칭 사이클로 누르기 → 램프·문 열림 확인 → 문 밖 대기 → LiDAR 문 틈 2회 확인 → 실제 문 틈에 맞춰 다시 섬 → 문 축으로 곧게 진입 |
| 탑승(누르기) | 오른쪽 90° → 고정 캐빈 자세로 층 버튼 찾기 → 버튼판을 정면으로 바라보도록 차체 위치·방향 정렬 → 팔 받침·손목은 정면 고정, 어깨·팔꿈치 위아래 각도만 사용해 누르기 → 후진 후 요청 버튼 램프 영상 확인 → 캐빈 중앙으로 복귀 → 문 정면 → 팔 들어 문 위 표시창 보기 |
| 탑승(하차) | 표시창 7-세그먼트로 목표층 확인 → 지도 전환·로봇 쪽 초기 자세 시드·재정위 확인 → 층 재확인 → 문 틈 2회 확인 → 실제 문 틈에 맞춰 곧게 하차 → 하차 지점. 문이 닫히면 왼쪽으로 돌아 현재층 버튼을 다시 누른다 |

팔은 미리 정한 각도로만 움직인다(사용자 2026-10-13): 보기 자세 7개는 `world_params.yaml arm.view_poses`(PWM), 누르기는 젯슨과 같은 `servo_protocol.py` 사이클. 시연 버튼판(층 버튼 20.5 x 16 cm, 승하차 13.5 x 20.5 cm)은 그 사이클의 팔 끝 높이(약 0.68 m)에 붙는다. 팔·LiDAR·지게팔 배치는 사용자 사진 추정값(`photo_body()`).

엘리베이터 노드는 팔 끝 위치(관절 상태 + 정운동학)가 캡 중심 2.5 cm 안·캡 면 0.4 cm 앞~6 cm 뒤에 들어올 때만 눌림으로 처리한다. 버튼 램프·층 표시창은 노드가 월드 모델을 움직여 켠다. 엘리베이터 문 열림 유지는 60 s(`elevator.timing.hold_open_s`)다. 실제 엘리베이터의 5~10 s로는 로봇의 누르기→대기점→진입 순서가 끝나기 전에 문이 닫힌다. 현장 측정이나 유지 연장 방법이 필요하다(아직 모름 U-E02).

## 한 명령 실행 (세부)

누름 성공이 카메라로 확인된 버튼은 `logs/button_press_memory/sim.json`에 층·종류·라벨별로 기억한다. 버튼 map 좌표와 누름 순간 차체 map 위치·방향, 팔 PWM을 저장한다. 다음 실행에서도 현재 카메라로 같은 버튼을 확인해야 한다. 지도·팔 설정 변경 시 이전 기억은 무효다. 실행별 사본은 결과 폴더의 `button_press_memory.json`이다.

```bash
# A층: 한 층 실측 지도 + 로봇 + OpenCR shim + nav_safety_gate + Nav2 (준비 판정까지)
bash sim/real_maps/start_sim.sh --floor F1 --params P0 --spawn f1_initial_test [--gui]
python3 sim/real_maps/sim_goal.py --name <출력된 이름> --to f1_locker     # goal 1개 + 측정
bash sim/real_maps/start_sim.sh --stop                                    # 모든 시뮬 프로세스 정리

# 토요일 사전 시험 (실행마다 새 스택)
python3 sim/real_maps/run_pretest.py --cases G1,G2 --params P0 --repeats 5 --prefix pre3

# C층: End-to-End 왕복 (두 층 한 월드 + 엘리베이터 + 지도 전환)
python3 sim/real_maps/run_e2e.py --name e2e_r1 [--cabin-mode nav2|direct] [--repeats 3]

# 결과 요약 표 (호스트, ROS 불필요)
python3 sim/real_maps/summarize_runs.py --pretest 'pre3_*' --e2e 'e2e_*' --out results/20261008
```

호스트에서 실행한 파이썬 러너는 격리 컨테이너를 올린 뒤 그 안에서 자신을 다시 실행한다(`host_exec.py`). 병렬 실행은 `PACKAGU_SIM_CONTAINER=packagu_sim_e2e_b`처럼 컨테이너 이름을 다르게 준다. 컨테이너마다 `ROS_LOCALHOST_ONLY`라 서로도 섞이지 않는다. 처음 실행하면 `colcon build`(common_pkg·slam_pkg·drive_pkg·auto_floor_orchestrator_pkg)와 월드 생성을 자동으로 한다. 강제로 다시 하려면 `SIM_REBUILD=1`, `SIM_REGENERATE=1`.

옵션: `--params P0|P1|v011|v012`(P0 = `nav2_params_pre_wallpush_20260915.yaml`, P1 = 현재 Jetson 기본 wall_push `nav2_params.yaml`), `--lidar-noise on|off`, `--odom encoder|world`. params는 `src/slam_pkg/config/` 절대경로로 넘긴다(install share에 빌드 뒤 추가한 파일이 없을 수 있음).

## 구성

| 파일 | 역할 |
|---|---|
| `world_params.yaml` | 월드·엘리베이터·LiDAR·로봇·판정 기준 치수. 값마다 출처(`[측정값]`·`[제안값]`·`아직 모름`). 토요일 측정 뒤 값만 바꾸고 다시 생성 |
| `generate_worlds.py` | occupied 셀 → 행 run-length 병합 상자 벽(높이 1.0 m). raster 복원·모서리·층 오프셋 오차 검사(0.05 m 넘으면 멈춤). F1 noglass, F2 raw, F2 door_open, 두 층 `building.world`, 로봇 URDF 3종 |
| `sim_container.sh` / `isolation.py` | 격리 컨테이너와 격리 검사 |
| `sim_stack.py` | 스택 기동(9/15 순서 STARTUP: discovery 대기 → localization → initial pose → navigation), 준비 판정(scan 주기·TF `map→odom→base_footprint`·AMCL·Nav2 lifecycle·`/drive/ready`), 측정(지면 진실 footprint–벽 거리, 코너, collision-ahead, recovery, 48 rpm 차단, 자연스러움 지표, RTF), 정리 |
| `opencr_shim.py` | 실제 `OpencrBridgeNode.send_command_tick` 재사용(0.5 s timeout, 48 rpm 초과 시 비율 축소 없이 0·ready false, 60 rpm/s slew) |
| `elevator_sim_node.py` | 시뮬 전용 엘리베이터: 닫힘 → 호출 → 도착 대기 → 열림 → 열림 유지 → 닫힘 → 이동 → 목적층 열림. 미닫이 문짝 2장, 끼임 시 재열림 on/off, 캐빈 안 로봇을 위치·yaw 그대로 다른 층 캐빈으로 이동. `/elevator/state` 발행 |
| `fake_floor_reader.py` | 가짜 층수 인식기. `tools/floor_reader/app.py`와 같은 `GET /api/state` 형식·이벤트 gate, 값은 `/elevator/state`에서 |
| `floor_maps_real.yaml` | orchestrator용 시뮬 전용 층 지도 표(F1 v3, F2 nav_unknown, 캐빈 초기 pose). 가상 KKU `floor_maps.yaml`은 그대로 |
| `run_pretest.py` | G1·G2·G3·G5 사전 시험 |
| `run_e2e.py` | 왕복 러너(goal당 300 s [제안값], 실패 시 정지·기록·중단, 지도 전환 뒤 새 AMCL 표본 판정). `--arm-sim --load pallet`이면 자율 시퀀스 |
| `autonomy_smoke.py` | 자율 시퀀스 짧은 점검(택배 집기·입장·탑승), 통계 제외 |
| `sim_adapters.py` | 공용 상태 머신용 시뮬 어댑터(주행·안전 검사, 팔 자세·누르기, 카메라 인식, 리프트, 적재) |
| `perception.py` / `arm_geometry.py` | 카메라 인식(공용 `vision_core` 사용) / 팔 [가정값] 기하·정운동학·역운동학 |
| `arm_lift.py` | 팔 궤적·리프트 명령, 초음파 정지 들기(`carry_up`) |
| `parcel_mock.py` | 팔레트·택배 배치, attach/detach mock, 가상 초음파 `/sim/ultrasonic_front` |
| `dressing.py` | 시각 전용 장식, 버튼·램프·7-세그먼트 층 표시창 모델 |
| `arrange_gui.py` | 호스트에서 이 컨테이너 창만 화면 왼쪽 절반에 배치 |
| `probe_drive_slip.py` | encoder odom 대 지면 진실 미끄럼·크리프 진단 |
| `summarize_runs.py` | 결과 표(`results/`) |
| `run_scenarios.py`, `run_manual_mission.py`, `run_suite.sh` 등 | 9/15 S1~S6·수동 왕복 러너(WORLD odom 재현용, 격리 컨테이너 안에서만) |

## 명령 경로

```text
Nav2 /cmd_vel → nav_safety_gate(실제 코드·nav_safety.yaml) → /cmd_vel_safe → OpenCR shim → /sim/cmd_vel_drive → Gazebo diff drive
```

지면 진실은 p3d `/sim/ground_truth`(world 기준, 20 Hz)이며 평가에만 쓴다. Nav2에는 넣지 않는다. 두 층 월드에서는 층별 `world_offset`(F2 +100 m)을 빼서 지도 좌표로 바꾼다.

## odom과 접촉 모델

- 새 시험(사전 시험·E2E)은 **encoder odom**(diff drive `odometry_source` 0). 엘리베이터 이동 때 바퀴가 돌지 않으므로 실차처럼 odom이 연속이다. E2E는 이동 전후 odom 변화와 캐빈 기준 상대 자세(위치·yaw)를 검사한다
- 9/15 재현(`run_scenarios.py`)은 `robot_world.urdf`(WORLD odom = 참값이 odom)
- encoder odom에서는 9/15 캐스터 모델(고정 구, 미끄럼 마찰)이 바퀴를 끌어 odom이 직진 25%·회전 38% 과대였다. 뒤 볼캐스터를 구르는 공(x·y 회전 관절)으로 바꾸고 보조 바퀴를 2 mm 띄웠다(`world_params.yaml robot.contact`, [제안값]). 근거는 `probe_drive_slip.py` 결과

## 지도 전환 뒤 진행 조건

orchestrator READY나 `/initialpose` 발행만으로 하차 goal을 보내지 않는다. `/initialpose` 이후 stamp의 새 AMCL 표본(`/request_nomotion_update`로 요청)으로 위치·방향 오차(지면 진실 대비), 공분산, `map→base_footprint` TF 안정 구간을 모두 확인한다. 기준은 `world_params.yaml relocalization`([제안값]). 제한 시간 안에 못 맞추면 정지·기록·중단한다.

## 시험

```bash
bash sim/real_maps/run_sim_tests.sh   # ROS 없으면 해당 시험 SKIP (scripts/run_offline_tests.sh 규칙)
```

## 기록 위치

bag·dense CSV·로그는 gitignore된 `logs/real_map_sim/<실행 이름>/`(실행마다 새 이름, 덮어쓰지 않음). 공유용 작은 요약만 `results/`에 둔다.

## 2026-10-13 Codex 수정본

- F2 두 층 월드만 `generated/f2_sim_flat.yaml`과 같은 평면 벽을 사용한다. 지정 영역 밖 0셀 변경, 원본 F2 지도와 단층 G2 기준 월드는 유지한다. 설정은 `world_params.yaml f2_cleanup`, 정리 근거는 `geometry_validation.json F2_flat`. 호출판은 F1/F2 모두 오른쪽이다.
- Nav2는 빈 포크·적재 팔레트까지 포함하는 실행별 프로파일을 사용한다. 적재/하역 뒤 다음 goal 전에 양쪽 costmap 외곽을 갱신한다. 최소 벽 거리 평가도 같은 외곽이다.
- 카메라는 기존 팔 끝 카메라 하나다. 누름 확인은 요청한 버튼 위치의 램프 영상만 사용한다. 누름 유지 중 또는 물러나 다시 보는 영상에서 불을 확인하고 미확인 시 제한된 횟수만 재시도한 뒤 정지·중단한다. 층 표시창은 도착층 확인 전용이다. 사용자 요청에 따라 시뮬 램프는 실행 초기화 전까지 켜짐 상태를 유지하며 실물 종이컵에 불이 있다는 뜻은 아니다.

```bash
PACKAGU_SIM_CONTAINER=packagu_sim_codex_20261013 python3 sim/real_maps/run_e2e.py --cabin-mode direct --arm-sim --load pallet --parcels 2 --speed 3 --gui
```

새 체크아웃에서는 먼저 `bash sim/real_maps/start_sim.sh --floor F1 --gui`로 빌드·월드 생성을 한다. 원커맨드 재시험 스크립트는 `run_codex_validation.sh`를 사용한다.

## 엘리베이터 내부부터 종료까지

사용자 2026-10-13 요청: 이미 탑승·적재한 상태를 시험 초기 조건으로 만들고 F1 층 버튼 → F2 하차·배달·하역 → F2 호출·탑승 → F1 복귀·출발점 도착만 검증한다. 앞부분 픽업·F1 호출/입장은 이 모드의 검증 범위 밖이다. 카메라는 기존 팔 끝 카메라만 사용한다.

```bash
bash sim/real_maps/run_codex_validation.sh --start-stage cabin
```

`SIM_GUI=0`은 Gazebo/RViz 창을 끄며 팔 카메라 렌더링은 유지한다. 기본 `SIM_SPEED=max`는 물리 step 1 ms를 유지하고 Gazebo 시간 제한을 푼다. 실제 배속은 머신 부하에 따라 달라진다([Gazebo 물리 설정](https://classic.gazebosim.org/tutorials?tut=physics_params)). `SIM_SPEED=3`처럼 수치로 고정할 수도 있다.
