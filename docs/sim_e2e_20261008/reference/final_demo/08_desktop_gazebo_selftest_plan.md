# 192 데스크톱 Gazebo 자체 시험 계획 (토요일 주행 시험 대비)

작성일: 2026-10-08. 상태: **계획만**. 목적: 토요일(2026-10-10) 현장 주행 시험 전에 같은 지도·설정·지점으로 Gazebo에서 미리 돌린다. 경로 문제, 명령 순서 오류, 운용 스크립트 결함을 현장 전에 찾는 것이 목표다. 결과는 모두 **시뮬레이션 검증**이다. 실물 검증으로 쓰지 않는다.

표기: `아직 모름` = 사실 미확인(⚠️미확인과 같은 뜻), `미정` = 결정 안 됨. 기준값은 `[제안값]`이다. 모르는 항목 전체는 [10_unknowns_register.md](10_unknowns_register.md)에 있다.

## 0. 이미 있는 것과 없는 것

| 자산 | 위치 | 상태 |
|---|---|---|
| 9/15 실측 지도 Gazebo 시험 | 데스크톱 로컬 checkout의 `sim/real_maps/`(`run_suite.sh`, `summarize.py`, `record_rehearsal_sim.py`, `f2_door_open.world` 등) | 보고서: [2026-09-15_real_map_gazebo_report.md](../../docs/bringup_guide/2026-09-15_real_map_gazebo_report.md). **GitHub `lee/sim-real-maps`(`44e9fc8`)에는 `sim/` 파일이 0개다**(2026-10-08 확인). 데스크톱에 아직 남아 있는지 아직 모름 |
| 9/15 시험 bag·CSV | 데스크톱 `logs/real_map_sim/`(gitignore) | 남아 있는지 아직 모름 |
| 9/15 환경 | Ubuntu 22.04.5, i7-12700, RAM 32 GB, GPU 드라이버 없음(software GL), ROS 2 Humble Docker, Gazebo Classic 11.10.2 | "192 데스크톱"이 이 기계인지 아직 모름 |
| 가상 KKU 월드 | `src/common_pkg/worlds/kku_f{1,2,3}.world`, `scripts/run_kku_sim.sh` | 5월 가상 건물. 실제 건물 기하가 아니다 |
| 층 이동 월드 교체 | `test_workspace/gazebo_world_swap/`(보행자·정적 장애물 포함) | 가상 KKU 기준으로 6/25·6/29 시뮬 검증 기록 |
| 이성덕 카메라 미션 | GitHub `2026-10-06`(`43872e8`) | 미션 launch가 `simulation_mode: False`를 고정하고 `use_sim_time` 인자가 없다(launch 파일 직접 확인). 시뮬에서 돌리려면 시뮬 전용 launch가 필요하다 |

9/15 시험은 정규 조건 33건 중 7건만 끝났다. 그 7건은 모두 SUCCEEDED였다. F2 entry 최소 벽 거리는 0.161~0.208 m, staging은 0.319~0.321 m였다. 소요 시간 538~811 s는 실시간 비율(RTF) 기록이 없어 실제 주행 시간과 비교할 수 없다.

## 1. 기준선 — 무엇으로 시뮬하나

토요일 현장과 같은 파일로 돌려야 의미가 있다. [07 점검](07_jetson_ssh_readonly_check_2026-10-08.md) 결과로 아래 표를 채운다.

| 항목 | Jetson 값(오늘 채움) | 시뮬에 쓸 값 |
|---|---|---|
| 코드 커밋 | `~/Code_Space` = `9c5ee7e` 코드(10/8 확인). 이성덕 코드는 별도 폴더 `43872e8` | 시뮬 브랜치 `lee/sim-e2e-20261008` = `43872e8`(= `9c5ee7e` + 이성덕 커밋) + `lee/sim-real-maps` 병합 |
| `nav2_params.yaml` | `80787840cd76` wall_push(10/8 확인, 철회 미적용) | P0 = `nav2_params_pre_wallpush_20260915.yaml`(`e8cf213b33fb`, 9/17 철회 결정). P1 = 현재 Jetson 기본(wall_push)도 같이 돌려 차이를 기록 |
| `nav_safety.yaml`·`drive_calib.yaml` | 브랜치와 같음(10/8 확인) | 브랜치 파일 |
| 지도 F1·F2 | F1 v3·F2 `f2_nav_unknown_v1`, 브랜치와 같음(10/8 확인) | 브랜치 파일 |
| `waypoints.json` | 브랜치와 같음(10/8 확인, 지점 13개) | 브랜치 파일 |
| 토요일 주행 범위 | 미정 | 05 T6 기준: F1 출발 → `f1_locker` 5회, F2 `f2_delivery_left_room4` → `f2_elevator_staging_v1` 5회 |

Jetson과 시뮬 파일이 다르면 그 차이를 결과 표 맨 위에 적는다. 다른 파일로 얻은 결과는 토요일 판단에 쓰지 않는다.

## 2. 데스크톱 준비 (오늘 오후~저녁)

### 준비 1. 기계 확인

```bash
lsb_release -ds; nproc; free -h | head -2; (nvidia-smi -L 2>/dev/null || echo "GPU 없음"); docker images --format '{{.Repository}}:{{.Tag}} {{.ID}}' | grep -i -E 'ros|humble'
```

컨테이너 안에서 `gazebo --version`과 `ros2 pkg list | grep -i gazebo`를 본다. 9/15 환경표와 다르면 10 문서 `U-D04`에 적는다.

### 준비 2. 9/15 시뮬 작업 찾기 — 가장 먼저

```bash
find ~ -maxdepth 5 -type d -path '*sim/real_maps' 2>/dev/null; find ~ -maxdepth 5 -type d -name real_map_sim 2>/dev/null
```

- 찾으면: 그 checkout에서 `git status -sb`와 `ls sim/real_maps`를 기록한다. GitHub에 올라가지 않은 유일한 사본이므로 **바로 새 폴더에 tar로 백업**한다(기존 백업 덮어쓰기 금지). 원래 checkout은 건드리지 않는다.
- 못 찾으면: 다시 만든다. 9/15 보고서 1~3절이 설계 문서 역할을 한다. 최소 범위는 지도 → 벽 생성기(행 run-length 병합, 정합 검사), 로봇 sim 설정, OpenCR shim, 지면 진실 p3d, 시나리오 러너다. 이 경우 토요일 전에는 G1·G2만 목표로 한다.

### 준비 3. 코드 받기

9/15 checkout과 섞지 않도록 따로 받는다. Code_Space는 private이라 인증이 필요하다. 데스크톱에서 GitHub 인증이 되는지 아직 모른다(`gh auth status`로 상태만 본다. 토큰 값을 출력하지 않는다).

```bash
git -C <9/15 checkout> fetch origin && git -C <9/15 checkout> worktree add ../cs_sim_20261008 origin/2026-10-06
```

`<9/15 checkout>`은 준비 2에서 찾은 경로다. 찾지 못했으면 새로 clone한다. Jetson patch가 있으면 이 worktree에만 적용한다.

### 준비 4. Jetson 파일 덮기

07 4절의 `_cfg.tgz`를 worktree 밖 시뮬 전용 폴더에 풀고, SHA256이 07 결과와 같은지 확인한다. 시뮬은 이 폴더의 절대경로로 params·지도를 넘긴다. colcon install share에는 빌드 뒤 추가한 설정 파일이 없을 수 있다(9/15 기록).

### 준비 5. 동작 확인

F1 v3 월드에서 다음을 확인한다: `/scan` 약 7.6 Hz, `/odom`, TF `map→odom→base_footprint`, AMCL 수렴, Nav2 lifecycle active, 짧은 goal 1개 SUCCEEDED. 9/15에는 빠른 자동 startup에서 lifecycle 응답이 유실됐다. 같은 증상이면 9/15 방식(discovery 대기 → localization → initial pose → navigation 순서 STARTUP)을 쓴다.

## 3. 시험 항목

모든 항목은 bag에 지면 진실 pose, `/amcl_pose`, `/plan`, costmap, `/cmd_vel`·`/cmd_vel_safe`, `/rosout`을 남긴다. RTF와 데스크톱 CPU도 같이 기록한다.

| ID | 토요일 연결 | 내용 | 반복 | 기록 | 기준 `[제안값]` |
|---|---|---|---|---|---|
| G1 | 05 T6 첫째 | F1 `f1_initial_test` → `f1_locker` (9/16 현장 실패 goal) | 5 | action 결과, 시간(시뮬·실제), footprint–벽 최소 거리, recovery 횟수, 도착 오차 | SUCCEEDED 4/5 이상, 최소 거리 0.15 m 이상, lethal-start 0 |
| G2 | 05 T6 둘째 | F2 `f2_delivery_left_room4` → `f2_elevator_staging_v1` (9/16 현장 2회 실패) | 5 | G1 항목 + 코너 `(-12.95,-5.25)` 반경 3 m 최소 거리, collision-ahead 수, 정차 위치 퍼짐 | G1과 같음 |
| G3 | 05 T6 시작 전 | 초기 위치 오차: 실제 스폰과 AMCL 초기값을 일부러 어긋나게 준 뒤 G1 | 3 | 초기·최종 위치 오차, 회복 여부 | 마지막 AMCL 10표본 모두 0.25 m 이하(9/15 기준) |
| G4 | 05 T6 운용 | 토요일 명령 순서 그대로: `record start/check` → `pose capture` → goal → 300 s 넘으면 `stop` → `cancel --all` → action 최종 상태 확인 → `resume` | 2 | 각 명령 exit code·출력, bag metadata 생성 여부 | 모든 단계 정상. 9/15에 본 결함(`record stop` 뒤 metadata 없음, `field_pose_capture.py` lethal 미검출)이 현재 코드에 남았는지 확인 |
| G5 | 미정 | F2 staging → entry, 문 열린 월드(`f2_door_open.world`) | 3 | G2 항목 | 토요일에 entry까지 갈지 미정. 미정이면 생략 |
| G6 | 토요일 범위 밖 | 이성덕 카메라 미션 논리: 가짜 층수 인식기(HTTP `/api/state`) + 팔 `simulation_mode` + 시뮬 전용 launch(`use_sim_time`) | 1 | CR-01(호출 직후 탑승 주행), CR-24(지도 전환 뒤 위치 확인 없음) 재현, `ElevatorArrivalGate` 동작 | 재현 여부 기록만. 고치지 않는다 |
| G7 | 미정 | 증속 후보 v011·v012 | — | — | 토요일 증속 시험 여부 미정. 정해지면 9/15 S5 방식으로 |

G1·G2·G4가 토요일 전 필수다. G3는 시간이 있으면 한다. G6은 [09 월드 계획](09_realistic_sim_world_plan.md)의 엘리베이터 모델(L3) 없이도 가짜 인식기로 일부 확인할 수 있지만, 우선순위는 토요일 뒤다.

### 실패했을 때

- 현장 기본 파일(`nav2_params.yaml`, `nav_safety.yaml`, `drive_calib.yaml`, `fieldctl`, `start_field_*.sh`, `map_pins.json`, `latest_map.txt`, `waypoints.json`)은 고치지 않는다. 시뮬 전용 사본으로만 후보를 만든다.
- 후보(경유점, 지점 조정 등)는 결과 표와 함께 사용자에게 보여 주고, 토요일 현장에서 쓸지는 사용자가 정한다.
- 시뮬에서만 실패하고 원인이 시뮬 모델(바퀴 마찰, 볼캐스터, software GL 지연)이면 그렇게 적고 현장 판단에 섞지 않는다.

## 4. 일정 `[제안값]`

| 때 | 할 일 |
|---|---|
| 10/8(목) 낮 | 07 Jetson 점검 → 결과로 1절 기준선 표 채우기 |
| 10/8(목) 오후 | 준비 1~4. 준비 2에서 9/15 작업을 못 찾으면 재구성 시작 |
| 10/8(목) 저녁 | 준비 5 → G1·G2 1회씩 |
| 10/9(금) | G1·G2 반복, G4, 시간이 있으면 G3. [09 월드 계획](09_realistic_sim_world_plan.md) 토요일 전 항목(SW1~SW3) |
| 10/9(금) 밤 | 결과 요약 → 05 T6 표에 "시뮬 사전 결과" 열 추가 여부 결정 |
| 10/10(토) | 현장 시험 |

## 5. 기록

- 데스크톱: bag·dense CSV는 `logs/real_map_sim/<실행 이름>/`(gitignore). 실행마다 새 이름을 쓴다.
- 이 PC로 옮길 것: 요약 표, 그림, 설정 해시, 실패 실행의 `/rosout` 발췌. 큰 bag은 옮기지 않는다.
- 요약 문서 위치·이름은 미정이다(제안: `docs/sim/2026-10-09_pre_saturday_sim.md`).

## 6. 시뮬로 확인할 수 없는 것

수동 지도의 절대 오차, 유리·반사 재질에 대한 실제 LiDAR 반응, 실제 AMCL 노이즈, 보조 바퀴·문턱, 모터 부하·제동·온도, 실제 OpenCR 펌웨어와 전원, Jetson CPU 부하(데스크톱이 훨씬 빠르다), Wi-Fi 끊김, 실제 엘리베이터 문·버튼·표시창, 팔·지게팔 동작. 시뮬 합격은 이 항목들에 대해 아무것도 말하지 않는다.

## 7. 멈추고 사용자에게 물을 조건

- sudo·패키지 설치가 필요할 때(설치 명령만 정리한다).
- 9/15 시뮬 작업을 찾지 못해 재구성 규모가 커질 때.
- 월드–지도 정합 오차가 0.05 m를 넘을 때(9/15 기준).
- Jetson 파일과 GitHub 파일이 달라 어느 쪽을 기준으로 할지 정해야 할 때.
