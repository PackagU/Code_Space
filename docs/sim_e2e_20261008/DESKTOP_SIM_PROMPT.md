# 192 데스크톱 시뮬레이션 세팅 프롬프트 (2026-10-08)

사용법:

- 192 데스크톱에서 **Claude Code**를 열고 아래 `---` 사이를 첫 메시지로 붙여 넣는다. 구현은 Claude Code가 맡는다.
- **Codex**는 같은 checkout에서 동시에 파일을 고치지 않는다. 각 단계가 끝날 때 [CODEX_REVIEW_PROMPT.md](CODEX_REVIEW_PROMPT.md)로 읽기 전용 검토를 맡긴다. 둘이 같은 파일을 동시에 고치면 충돌이 난다.
- Codex로 구현하려면 같은 본문을 Codex 첫 메시지로 넣어도 된다. 그때는 Claude Code를 검토 역할로 쓴다.

---

`PackagU/Code_Space`(private) 저장소의 `lee/sim-e2e-20261008` 브랜치를 받아, **우리가 만든 현장 지도(F1 `f1_manual_clean_v3`, F2 `f2_raw_20260914`/`f2_nav_unknown_v1`)로 End-to-End 시뮬레이션 환경**을 만들어라. 목표는 한 명령으로 실측 지도 월드 + 로봇 + 안전 게이트 + Nav2를 띄우고, 그 안에서 (1) 토요일(10/10) 현장 주행 시험의 사전 시험과 (2) F1 출발 → 보관함 → 엘리베이터 → F2 배달 위치 → 복귀 왕복을 자동으로 돌릴 수 있게 하는 것이다. 모든 결과의 판정은 **시뮬레이션 검증**이며 실물 검증으로 쓰지 마라. 막히면 비동기 질문을 남기고 영향 없는 다음 단계를 진행해라. 장시간 대기는 로컬 셸로 하고 모델을 반복해서 깨우지 마라.

## 0. 받기와 먼저 읽을 것

```bash
git clone --branch lee/sim-e2e-20261008 https://github.com/PackagU/Code_Space.git cs_sim_e2e
cd cs_sim_e2e && git log --oneline -5
```

- 이미 다른 Code_Space checkout이 있으면 **그 checkout을 건드리지 말고** 새 폴더에 받는다. 9/15 시뮬 작업이 그 안에 남아 있을 수 있다(1단계).
- 인증이 안 되면 `gh auth status`만 확인하고 질문한다. 토큰 값을 출력하거나 파일에 쓰지 마라.
- 읽는 순서: 저장소 루트 `AGENTS.md` → `docs/sim_e2e_20261008/README.md` → `docs/sim_e2e_20261008/reference/final_demo/08_desktop_gazebo_selftest_plan.md` → `09_realistic_sim_world_plan.md` → `10_unknowns_register.md` → `docs/sim_e2e_20261008/reference/2026-09-15_real_map_gazebo_report.md` → `docs/sim_e2e_20261008/reference/2026-09-15_linux_desktop_sim_prompt.md`(설정 사실 목록).
- 표기: `아직 모름` = 사실 미확인, `미정` = 결정 안 됨, `[측정값]`·`[제안값]`. 모르는 치수를 지어내지 말고 임시값은 `[제안값]`으로 표시해라.

## 안전 — 실제 로봇과 섞이지 않게

- **같은 LAN에 Jetson이 있다.** 데스크톱 ROS 그래프가 Jetson과 연결되면 시뮬 명령(`/cmd_vel` 등)이 실제 로봇으로 갈 수 있다. 시뮬 컨테이너는 반드시 격리한다: `ROS_LOCALHOST_ONLY=1`과 0이 아닌 `ROS_DOMAIN_ID`(`[제안값]` 77) 또는 9/15처럼 격리된 Docker 네트워크. `scripts/fastdds_lan_peers.xml`과 `scripts/run_sim_host.sh`(Jetson 분산 모드)는 **쓰지 마라**. 시작 스크립트는 격리 설정이 없으면 기동을 거부하게 만든다.
- Jetson에 SSH 접속하거나 Jetson 파일을 바꾸지 마라. 하드웨어 장치를 컨테이너에 넘기지 마라.

## 금지와 경계

- 현장 기본 파일(`src/slam_pkg/config/nav2_params.yaml`, `src/drive_pkg/config/nav_safety.yaml`, `src/drive_pkg/config/drive_calib.yaml`, `scripts/fieldctl`, `scripts/start_field_*.sh`, `src/slam_pkg/maps/field/{map_pins.json,waypoints.json,*/latest_map.txt}`)과 이성덕 미션 코드는 고치지 마라. 시뮬 전용 파일·프로필·launch·스크립트로 분리한다. 현장 코드 결함을 찾으면 고치지 말고 보고서에 적는다.
- `.env`, 키, 토큰, 인증·세션 파일은 읽거나 커밋하지 마라.
- sudo·패키지 설치·이미지 pull이 필요하면 명령만 정리하고 멈춘다.
- 커밋·push는 `lee/sim-e2e-20261008`에만 한다. push 전에 `git pull --ff-only`. force push·PR·다른 브랜치 merge 금지.

## 확정 사실 (2026-10-08 기준)

- Jetson 현장 코드 = 이 브랜치의 `9c5ee7e` 부분과 같다(10/8 blob 대조). 지도·지점·안전 게이트·구동 보정 파일 해시는 `docs/sim_e2e_20261008/README.md` 2절.
- Nav2: **P0 = `src/slam_pkg/config/nav2_params_pre_wallpush_20260915.yaml`(`e8cf213b…`)** — 사용자가 9/17 wall_push 철회를 결정했다. **P1 = `nav2_params.yaml`(`80787840…`, wall_push_v1)** — Jetson에 아직 남아 있는 현재 기본값. 둘 다 돌려 차이를 기록하되 P1을 권고하지 마라. colcon install share에는 빌드 뒤 추가한 설정이 없을 수 있으니 params는 `src/...` 절대경로로 넘긴다.
- 지도: F1 = `src/slam_pkg/maps/field/f1/f1_manual_clean_v3.yaml`(v2 외곽 직각화, 254/0/205, `free_thresh 0.19`). F2 = `f2/f2_nav_unknown_v1.yaml` + `f2_raw_20260914.pgm`. F3는 범위 밖.
- 지점(`waypoints.json`, 지도 좌표): `f1_initial_test (-4.518, 2.179)`, `f1_locker (-5.125, -8.625)`, `f1_elevator_entry (-8.725, -0.175)`, `f1_elevator_inside (-10.175, -2.075)`, `f1_elevator_exit`, `f2_elevator_entry (-11.375, -2.525)`, `f2_elevator_staging_v1 (-11.175, -3.075)`, `f2_elevator_inside (-12.475, -1.325)`, `f2_elevator_exit`, `f2_delivery_left_room4 (-24.725, -17.975)`, `f2_delivery_destination (-39.925, -32.625)`. yaw는 파일에서 읽어라. 엘리베이터 entry·inside·exit은 지도 이미지에서 계산한 값이며 로봇 실측이 아니다. F2 캐빈 쪽 여유 추정은 0.224 m로 footprint 외접 0.39 m보다 작다.
- 9/16 현장 Nav2 goal(`docs/presentation_20260916/goal_results.csv`): F1 시작 → `f1_locker` FAILED(97 s), F2 엘리베이터 앞 → `f2_delivery_left_room4` SUCCEEDED(244 s), room4 → `f2_elevator_staging_v1` FAILED 2회(collision-ahead 18회·0회), F1 엘리베이터 앞 → 시작 지점 SUCCEEDED(119 s, collision-ahead 113회). 시뮬 시간과 비교할 실제 시간 기준이다.
- 로봇: footprint `[[0.033,0.219],[0.033,-0.219],[-0.327,-0.219],[-0.327,0.219]]`(`base_footprint` 원점이 차체 앞 끝 근처, inscribed 0.033 m). `wheel_radius 0.033`, 실효 윤거 0.4323(`drive_calib.yaml`). LiDAR 프레임 `laser`, URDF `delivery_robot.urdf.xacro`의 `laser_z` 기본 0.667 m. 실물 LiDAR 높이는 아직 모름(9/10 Jetson TF는 옛 URDF로 0.770 m였다). RPLiDAR A1 약 7.6 Hz, 약 1,450 점/scan. 실차는 9/12 기록상 IMU를 발행하지 않았다.
- OpenCR shim(`src/drive_pkg/drive_pkg/opencr_bridge_node.py`의 `send_command_tick` 재현): `/drive/ready` 발행, 명령 0.5 s 끊기면 0, 좌우 목표 rpm 중 하나라도 48 초과면 비율 축소 없이 0 + ready false, 60 rpm/s slew.
- 안전 게이트 `nav_safety_gate`는 실제 코드와 `nav_safety.yaml`을 그대로 쓰고 `/cmd_vel` → `/cmd_vel_safe`만 diff drive에 연결한다.
- 9/15 시뮬에서 나온 운용 결함(현장 스크립트는 그대로 둠): `record stop` 뒤 bag metadata 없음, 원본 topic list가 hidden action status를 못 봄, `field_pose_capture.py`가 OccupancyGrid 99/100을 raw lethal 253/254와 비교해 lethal을 놓침. 빠른 자동 startup에서 lifecycle 응답 유실 → discovery 대기 후 localization → initial pose → navigation 순서 STARTUP으로 해결했다.
- 이성덕 카메라 미션 launch(`test_workspace/elevator_mission/src/elevator_mission_pkg/launch/elevator_camera_mission.launch.py`)는 `simulation_mode: False`를 고정하고 `use_sim_time` 인자가 없다. 미션 기본 지도·지점 설정(`test_workspace/elevator_auto_map_switch/config/floor_maps.yaml`, `test_workspace/elevator_mission/config/kku_nav_points.yaml`)은 가상 KKU 기준이다(검토 CR-03).

## 1. 환경 기록과 9/15 작업 복구

1. 환경: OS, CPU, RAM, GPU(`nvidia-smi` 없으면 software GL), Docker 이미지(`docker images | grep -i ros`), 컨테이너 안 Gazebo 버전(`gazebo --version` 또는 `gz sim --version`), `ros2 pkg list | grep -i gazebo`. 9/15 환경(Ubuntu 22.04.5, i7-12700, 32 GB, GPU 드라이버 없음, Gazebo Classic 11.10.2)과 같은 기계인지 적는다. `docker/compose/docker-compose.linux.yml`의 이미지 `ghcr.io/packagu/ros2-humble-slam:humble`이 없으면 멈추고 질문한다.
2. 9/15 시뮬 코드 찾기:

```bash
find ~ -maxdepth 5 -type d -path '*sim/real_maps' 2>/dev/null; find ~ -maxdepth 5 -type d -name real_map_sim 2>/dev/null
```

- 찾으면: 원래 checkout의 `git status -sb`·`git log -1`을 기록하고, 새 폴더 `~/packagu_sim_backup_$(date +%Y%m%d_%H%M)/`에 `sim/real_maps/`와 `logs/real_map_sim/` 목록(bag은 목록만, 복사는 선택)을 tar로 백업한다. 원래 checkout은 고치지 않는다. 코드(`sim/real_maps/`)만 이 브랜치로 복사해 **별도 커밋**한다("sim: 9/15 실측 지도 시뮬 코드 복구"). 그다음 현재 브랜치 경로·파일명에 맞추는 수정은 다음 커밋으로 나눈다.
- 못 찾으면: 9/15 보고서 1~3절을 설계서로 삼아 다시 만든다. 이때 보고에 "재구성"이라고 적는다.

## 2. 한 층 실측 지도 시뮬 (A층)

`sim/real_maps/` 아래에 둔다(9/15 구조가 있으면 따른다).

1. **월드 생성기**: PGM/YAML의 occupied 셀을 행 단위 run-length로 사각형 병합해 벽을 세운다(셀당 상자 금지, 높이 1.0 m — 9/15와 같게). origin·resolution에 정확히 맞추고, raster 복원 불일치와 모서리 오차를 검사한다. 0.05 m를 넘으면 멈추고 질문한다.
   - F1: v3 기준이되 보관함 유리 고정문만 뺀 `noglass` 월드(9/15: 원본과 44셀 차이). 생성기는 `docs/field_review_20260915/f1_manual_clean_20260915_v3/build_v3.py`·`geometry_v3.json`. Nav2는 원본 v3(유리문 포함)를 로드한다.
   - F2: `f2_raw_20260914.pgm` occupied 셀 그대로(잡음 유지). 문 열린 리허설용 `f2_door_open` 변형(9/15: 통로 중심 `(-11.925,-1.925)`, 축 0.741948 rad, 폭 1.0 m·수직 범위 0.8 m 마스크 안 78셀 제거)도 유지한다.
2. **치수 파일**: `sim/real_maps/world_params.yaml`(이름 바꿔도 됨)에 월드 치수를 모은다. 값마다 출처(`[측정값] 날짜·방법` / `[제안값] 근거` / `아직 모름(임시값)`)를 단다. 엘리베이터·보관함·유리·문턱 치수는 지금 모두 아직 모름이다. 목록은 `reference/final_demo/10_unknowns_register.md`의 `U-E`·`U-H`. 토요일 측정 뒤 값만 바꾸고 다시 생성할 수 있어야 한다.
3. **로봇**: URDF 기준. 차체 충돌 형상 = footprint, 볼캐스터·보조 바퀴는 마찰만 단순화. Gazebo diff drive(`odom→base_footprint`), ray LiDAR `/scan`(frame `laser`, 7.6 Hz, 약 1,450 점, 가우시안 잡음은 `[제안값]`으로 넣고 끄고 켤 수 있게). 지면 진실은 p3d 20 Hz 별도 토픽(Nav2에 넣지 않는다). URDF와 다른 점은 표로 남긴다.
4. **구동 대체**: OpenCR shim + 실제 `nav_safety_gate`.
5. **Nav2**: `src/slam_pkg/launch/kku_navigation.launch.py`를 `use_sim_time:=true`, 지도 yaml·`params_file` 절대경로로 띄운다. `start_field_navigation.sh`가 노드 이름 전제(`/rplidar`, `/packagu_opencr_bridge`) 때문에 막히면 시뮬 전용 시작 스크립트를 만든다. 그 스크립트도 `scripts/field_map_guard.py --stage pre-nav`를 호출한다.
6. **한 명령 시작**: 예) `bash sim/real_maps/start_sim.sh --floor F1 --params P0 --spawn f1_initial_test [--gui]`. 스폰·초기 pose는 `waypoints.json` 이름으로 받는다. 격리 설정(위 안전 절) 검사, lifecycle 순서 STARTUP, 준비 완료 판정(`/scan` 주기, TF `map→odom→base_footprint`, AMCL, Nav2 active)을 포함한다. 종료 시 gzserver·노드를 정리한다.
7. **확인**: 짧은 goal 1개 SUCCEEDED와 RTF를 기록한다.

## 3. 토요일 사전 시험 1차 (A층 위)

`reference/final_demo/08_desktop_gazebo_selftest_plan.md` 3절 G1·G2를 각 1회 먼저 돌린다. P0 기준.

- G1: F1 `f1_initial_test` → `f1_locker`.
- G2: F2 `f2_delivery_left_room4` → `f2_elevator_staging_v1`. 코너 `(-12.95,-5.25)` 반경 3 m 최소 벽 거리.
- 기록: action 결과, 시뮬 시간·실제 시간·RTF, 지면 진실 footprint 다각형–벽 최소 거리, collision-ahead 수, recovery 횟수, 도착 오차, 48 rpm 차단·ready 이탈 수. 자연스러움 지표(제자리 회전 횟수·누적 각, 각속도 부호 전환, 실제 거리/직선 거리, 정지·재출발)도 같이 남긴다.

## 4. 층 이동과 엘리베이터 (B층)

`reference/final_demo/09_realistic_sim_world_plan.md` L3을 따른다. 치수는 모두 `world_params.yaml` 임시값이다.

1. **층 이동 방식은 미정(U-X04)**이다. 기본 구현은 권고안 (b): F1·F2 월드를 한 Gazebo 월드에 서로 보이지 않을 만큼 떨어뜨려 두고, 층 이동 때 로봇을 **캐빈 안 상대 자세 그대로** 다른 층 캐빈으로 옮긴다(Gazebo set_entity_state). 지면 진실 평가 때 층별 오프셋을 빼라. 기존 (a) 월드 교체(`test_workspace/gazebo_world_swap/`)를 쓰는 편이 훨씬 쉬우면 그 이유를 적고 (a)로 해도 된다.
2. **엘리베이터 제어 노드(시뮬 전용)**: 상태 닫힘 → 호출 → 도착 대기 → 열림 → 열림 유지 → 닫힘 → 이동 → 목적층 열림. 시간은 `[제안값]` 파라미터. 문은 승강장 문·캐빈 문 미닫이 모델 또는 충돌체 on/off로 시작한다. 현재 층을 토픽으로 내고, 가짜 층수 인식기(HTTP `/api/state`, `tools/floor_reader/app.py` 응답 형식과 같게)가 그 값을 내보낼 수 있게 한다.
3. **지도 전환**: `test_workspace/elevator_auto_map_switch/`의 orchestrator를 쓰되, 가상 KKU용 `floor_maps.yaml`을 고치지 말고 시뮬 전용 `sim/real_maps/floor_maps_real.yaml`(F1 v3·F2 nav_unknown, 캐빈 초기 pose)을 만든다. 전환 뒤 `/initialpose` 반영과 AMCL 수렴(공분산·지면 진실 오차)을 기록한다(검토 CR-24).
4. 문턱은 넣더라도 "속도 낮추는 구간이 동작하는지"만 본다. Gazebo 볼캐스터 물리로 넘었는지는 참고값이다.

## 5. End-to-End 왕복 (C층)

시뮬 전용 러너 `sim/real_maps/run_e2e.py`(이름 자유)가 Nav2 goal과 엘리베이터 제어 노드 명령을 순서대로 보낸다. 팔·지게팔은 시뮬하지 않고 적재 상태 표시만 바꾼다(9/16 PC 시뮬과 같은 방식). 이성덕 미션 노드는 이 단계에서 쓰지 않는다.

```
F1 f1_initial_test → f1_locker(적재 표시) → f1_elevator_entry → 호출·문 열림 대기 → f1_elevator_inside
→ 층 이동 F1→F2 + 지도 전환 → f2_elevator_exit → f2_delivery_left_room4(하역 표시)
→ f2_elevator_staging_v1 → f2_elevator_entry → 호출·문 열림 대기 → f2_elevator_inside
→ 층 이동 F2→F1 + 지도 전환 → f1_elevator_exit → f1_initial_test
```

- 각 단계에 제한 시간(`[제안값]` goal당 300 s)과 실패 처리(정지·기록·중단)를 둔다. 성공으로 넘어가기 전에 action 최종 상태가 SUCCEEDED인지 본다.
- 단계별 결과·시간·최소 벽 거리·지도 전환 뒤 위치 오차를 CSV로 남긴다. 1회 완주를 먼저 목표로 하고, 되면 3회 반복한다.
- 캐빈 진입·회전이 footprint 때문에 막히면(F2 여유 0.224 m 추정) 그것도 결과다. 지점 좌표를 몰래 바꾸지 말고 시뮬 전용 후보로 따로 적는다.

## 6. 남은 사전 시험 (A·B층 위)

08 문서의 G1·G2 반복(P0 5회, P1 3회), G3(초기 pose 오차 주입, AMCL 회복: 마지막 10표본 모두 0.25 m 이하), G4(토요일 명령 순서 리허설: `record start/check` → `pose capture` → goal → 300 s 초과 시 `stop` → `cancel --all` → 최종 상태 확인 → `resume`, 9/15 결함이 현재 코드에 남았는지 확인). 기준(`[제안값]`): SUCCEEDED 4/5 이상, 최소 거리 0.15 m 이상, lethal-start 0.

## 7. (선택) 이성덕 카메라 미션 논리 (D층)

시간이 남을 때만 한다. 시뮬 전용 launch(복사본)로 `use_sim_time:=true`, 팔 `simulation_mode:=true`(serial_port 비움), `floor_reader_bridge`는 가짜 층수 인식기 URL, 미션 지도·지점은 시뮬 전용 실측 설정을 쓴다. 검토 문서(`reference/final_demo/02_code_review_2026-10-06.md`)의 CR-01(호출 직후 탑승 주행)·CR-24(지도 전환 뒤 위치 확인 없음)가 재현되는지 기록만 한다. 미션 코드는 고치지 마라.

## 8. 산출물과 커밋

- 코드: `sim/real_maps/`(생성기·정합 검사·`world_params.yaml`·로봇 sim 설정·shim·시작 스크립트·엘리베이터 제어 노드·가짜 층수 인식기·E2E 러너·시나리오 러너·요약 스크립트)와 그 사용법 `sim/real_maps/README.md`. 처음 보는 사람이 `start_sim.sh`와 `run_e2e.py`만으로 다시 돌릴 수 있어야 한다.
- 보고서: `docs/sim_e2e_20261008/REPORT.md` — 환경, 9/15 작업 복구/재구성 여부, 모델·지도 정합 오차, 단계별 결과 표(반복 수, action 결과, 최소 벽 거리, 시간·RTF, 48 rpm 차단), P0 대 P1, E2E 완주 여부와 실패 단계, 임시 치수 목록, **시뮬로 검증할 수 없는 항목**(수동 지도 절대 오차, 유리·반사, 실제 AMCL 노이즈, 보조 바퀴·문턱, 모터 부하·제동·온도, 실제 펌웨어·전원, Jetson CPU, 실제 엘리베이터·버튼·표시창, 팔·지게팔).
- `reference/final_demo/10_unknowns_register.md`의 `U-D` 항목을 확인한 값으로 갱신한다(이 브랜치 사본만).
- bag·dense CSV는 gitignore된 `logs/real_map_sim/<실행 이름>/`에 둔다. 실행마다 새 이름. 커밋하지 않는다.
- 새 테스트는 `scripts/run_offline_tests.sh` 규칙(ROS 없으면 SKIP)을 따른다.
- 커밋 전 검사: 텍스트 파일 CR 0, 100 MB 초과 파일 없음, `git diff --stat`가 예상 범위, 비밀 패턴 grep(`ghp_`, `github_pat_`, `BEGIN .* PRIVATE KEY`, `password`). 단계마다 작게 커밋한다.

## 9. 멈추고 질문할 조건

- sudo·패키지 설치·이미지 pull이 필요할 때
- 시뮬 컨테이너를 LAN에서 격리할 수 없을 때
- 9/15 작업을 못 찾아 재구성 규모가 커질 때(그래도 2·3단계는 진행)
- 월드–지도 정합 오차가 0.05 m를 넘을 때
- 지면 진실 pose를 얻을 수 없을 때
- 현장 기본 파일을 고쳐야만 진행할 수 있을 때

## 10. 완료 보고

바꾼 파일, 한 명령 실행법, 단계별 핵심 수치(G1·G2 P0 대 P1, E2E 완주 여부·실패 단계, 지도 전환 뒤 위치 오차), 9/15 작업 복구 여부, 임시 치수 목록, 커밋 SHA를 간결하게 보고한다. 토요일 현장 시험(05 T6)에 바로 쓸 수 있는 결론과 쓸 수 없는 것(시뮬 한계)을 나눠 적는다.

---
