# 실측 지도 Gazebo 시뮬 (sim/real_maps)

모든 판정은 **시뮬레이션 검증**이다. 실물 검증으로 쓰지 않는다. 월드 치수 대부분이 아직 모름(임시값)이라 결과에는 "임시 치수"를 붙인다. 현장 기본 파일(`nav2_params.yaml`, `nav_safety.yaml`, `drive_calib.yaml`, `fieldctl`, `start_field_*.sh`, `map_pins.json`, `latest_map.txt`, `waypoints.json`)과 이성덕 미션 코드는 고치지 않는다.

## 안전: 실제 로봇과 섞이지 않게

같은 LAN에 Jetson이 있다. 모든 시뮬은 `sim_container.sh`가 만든 격리 컨테이너 안에서만 돈다.

- Docker `--internal` 네트워크 `packagu_sim_isolated` (LAN·인터넷 경로 없음)
- `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=77`, 장치 전달·privileged·host network 없음
- 위 조건이 하나라도 빠지면 셸(`sim_container.sh verify`)과 파이썬(`isolation.assert_isolated`) 양쪽에서 기동을 거부한다
- `scripts/fastdds_lan_peers.xml`, `scripts/run_sim_host.sh`(Jetson 분산 모드)는 쓰지 않는다
- 컨테이너는 호스트 uid로 돌아 root 소유 파일을 남기지 않는다. 필드용 `ros2_humble` 컨테이너는 건드리지 않는다

## 한 명령 실행

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
| `run_e2e.py` | 왕복 러너(goal당 300 s [제안값], 실패 시 정지·기록·중단, 지도 전환 뒤 새 AMCL 표본 판정) |
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
