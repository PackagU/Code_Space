# 리눅스 데스크톱 Gazebo 실측 지도 시뮬 프롬프트 (2026-09-15 확정본)

아래 `---` 사이를 리눅스 데스크톱 세션의 첫 메시지로 붙여 넣는다.

---

`PackagU/Code_Space`(private) 저장소의 `lee/sim-real-maps` 브랜치를 받아, 9/14~15 현장 실패를 Gazebo에서 재현하고 9/15에 현장 기본값으로 반영한 개선(F1 지도 v3, Nav2 `wall_push_v1`)이 효과가 있는지 시뮬로 확인해라. 모든 결과 판정은 **시뮬레이션 검증**이며 실물 검증으로 쓰지 마라. 막히면 비동기 질문을 남기고 영향 없는 다음 단계를 진행해라. 장시간 대기는 로컬 셸로 하고 모델을 반복해서 깨우지 마라.

## 받기와 먼저 읽을 것

```bash
git clone --branch lee/sim-real-maps https://github.com/PackagU/Code_Space.git   # 이미 있으면 git fetch origin && git switch lee/sim-real-maps && git pull --ff-only
git log --oneline -5
```

- 브랜치 tip은 젯슨 미러 `lee/jetson-live` 커밋 `e223b7532d98231291bdb4ffd015c990cab2b716`을 merge한 이후여야 한다. `git merge-base --is-ancestor e223b7532d98 HEAD`가 성공하지 않으면 멈추고 질문한다.
- 읽는 순서: 루트 `AGENTS.md` → `docs/field_review_20260915/README.md`(PC 경로→브랜치 경로 대응표) → `session_autonomy_improvement_20260915_175043_KST.md` → `2026-09-15_autonomy_test_review.md` → `2026-09-15_autonomy_field_procedure.md`(특히 5.1절).
- `git clone` 인증이 안 되면 `gh auth status`만 확인하고 질문한다. 토큰 값을 출력하거나 파일에 쓰지 마라.

## 현재 확정 사실 (젯슨 현장 기본값, 2026-09-15 20:45 KST)

- **F1 기본 지도 = `src/slam_pkg/maps/field/f1/f1_manual_clean_v3.yaml`/`.pgm`** (yaml `d701d2b6…`, pgm `d525759b…`). v2 외곽선을 raw SLAM 주 벽 방향 21.75°에 맞춰 직각화했고 엘리베이터는 네모다. 값은 254/0/205, `free_thresh 0.19`. `latest_map.txt`와 `map_pins.json` F1 항목이 v3다. 생성기와 기하는 `docs/field_review_20260915/f1_manual_clean_20260915_v3/{build_v3.py,geometry_v3.json}`.
- F2 기본 지도 = `f2/f2_nav_unknown_v1.yaml` + `f2_raw_20260914.pgm`. F3는 이번 범위 밖이다.
- **Nav2 기본 `src/slam_pkg/config/nav2_params.yaml` = wall_push_v1 적용본** (`80787840…`): global costmap `inflation_radius 1.0`, `cost_scaling_factor 2.0`, planner `cost_travel_multiplier 3.0`. local costmap(0.55/3.0)과 RPP는 이전과 같다.
- **이전 기본값** = `nav2_params_pre_wallpush_20260915.yaml` (`e8cf213b…`). A/B의 기준선이다.
- 증속 후보 `nav2_params_speed_v011.yaml`(0.11 m/s·0.22 rad/s), `nav2_params_speed_v012.yaml`(0.12·0.20)은 wall_push 값을 포함한다. v012는 gate 상한 0.13이 필요하다(`field_base.launch.py`의 `gate_max_linear_speed:=0.13`, 셸에서는 `GATE_MAX_LINEAR_SPEED=0.13`).
- footprint `[[0.033,0.219],[0.033,-0.219],[-0.327,-0.219],[-0.327,0.219]]`. `base_footprint` 원점이 차체 앞 끝 근처라 inscribed가 0.033 m다. Humble SmacPlanner2D는 중심 셀 cost ≥ INSCRIBED만 충돌로 본다(humble 소스 `collision_checker.cpp`, `smac_planner_2d.cpp`).
- F2 벽 근접 지점(사용자가 그림으로 지정) = 복도에서 로비로 꺾기 직전 **왼쪽 벽의 움푹 들어간 곳 `(-12.95, -5.25)`**. 문서 앞부분의 `(-11.05, -1.85)` 추정은 틀렸다. 오프라인 근사에서 이 코너의 경로 중심 여유는 이전 기본값 0.552 m, wall_push_v1 0.886 m였다(`docs/field_review_20260915/autonomy_improvement_20260915/wall_push_evaluation.json`).
- 주요 웨이포인트(`waypoints.json`): `f2_delivery_destination (-39.925,-32.625,-2.33)`, `f2_elevator_entry (-11.375,-2.525,2.312744)`, `f2_elevator_staging_v1 (-11.175,-3.075,2.312744)`, `f1_locker (-5.125,-8.625,-1.69)`, `f1_elevator_entry (-8.725,-0.175,-2.222665)`, 옛 `f1_idle (-3.725,2.075,-1.701)`, 사용자 확인 idle 시드 `(-4.518,2.179,-1.701)`(v3 footprint–벽 0.453 m).
- 젯슨 오프라인 시험 기준선: `scripts/run_offline_tests.sh` 호스트 PASS 37 / SKIP 7 / FAIL 4. FAIL 4건(`test_opencr_firmware_contract`, `test_map_cleanup_review`, `test_kku_navigation_launch`, `test_nav2_orthogonal_tuning`)은 이번 작업 전부터 있던 옛 기대값·Python 3.8 문제다. 새 실패만 보고한다.
- colcon `--symlink-install` 환경에서도 빌드 뒤에 추가한 설정 파일은 install share에 없다. params는 `src/slam_pkg/config/...` 절대경로로 넘긴다.

## 금지와 경계

- 현장 기본 파일(`nav2_params.yaml`, `nav_safety.yaml`, `drive_calib.yaml`, `fieldctl`, `start_field_*.sh`, `map_pins.json`, `latest_map.txt`, `waypoints.json`)을 수정하지 마라. 시뮬 전용 파일·프로필·스크립트로 분리한다.
- 권고 설정을 젯슨에 반영하지 마라. 반영은 사용자 검토 뒤 Windows 세션에서 따로 한다.
- `.env`, 키, 토큰, 인증·세션 파일은 읽거나 커밋하지 마라. force push와 PR 생성은 금지한다.
- sudo·패키지 설치가 필요하면 설치 명령만 정리하고 멈춘다.

## 0. 환경 확인과 기록

- OS, CPU/GPU, 네이티브 ROS Humble인지 Docker인지, Gazebo 종류·버전(`ros2 pkg list | grep -i gazebo`, `gazebo --version` 또는 `gz sim --version`).
- 5월 시뮬 자산을 먼저 읽고 재사용 여부를 판단한다: `scripts/run_sim_host.sh`, `scripts/run_kku_sim.sh`, `src/slam_pkg/launch/kku_simulation.launch.py`, `scripts/generate_kku_worlds.py`, `test_workspace/gazebo_world_swap/`.
- GPU가 없으면 gzserver headless와 RViz로 진행한다.

## 1. 월드 생성 (`sim/real_maps/`)

- PGM/YAML의 occupied 셀을 행 단위 run-length로 사각형 병합해 높이 1.0 m 벽으로 올린다. 셀당 박스는 금지한다.
- 좌표를 map origin·resolution에 정확히 맞추고, 지도 셀과 월드 벽의 정합 오차를 수치로 검사한다. 0.05 m를 넘으면 멈추고 질문한다.
- **F1 월드는 v3 기준이되 유리 고정문을 뺀다.** 라이다가 유리를 못 보는 현실을 흉내 내기 위해서다. `build_v3.py`를 시뮬 폴더로 복사해 `INNER_WALLS_UV`의 유리문 선분(`(357.3,204.5)-(383.6,204.5)`)만 빼고 그린 `f1_manual_clean_v3_noglass.pgm`을 만든다. 원본 v3와 달라진 셀 수를 기록하고, 다른 셀은 모두 같아야 한다. Nav2가 로드하는 지도는 원본 v3(유리문 포함)다.
- F2 월드는 `f2_raw_20260914.pgm`의 occupied 셀 그대로 만든다. 흩어진 점 잡음은 현장 스캔에 있던 것이므로 지우지 않는다.

## 2. 로봇 모델

- `src/common_pkg/urdf/delivery_robot.urdf.xacro`를 기준으로 하고, 위 footprint, `wheel_radius=0.033`, 실효 윤거 0.4323(`drive_calib.yaml`), LiDAR 프레임 `laser`(URDF 값), RPLiDAR A1 약 7.6 Hz·약 1,450 pt/scan과 대조해 차이를 표로 남긴다.
- 시뮬 차체 충돌 형상은 footprint와 같게 둔다. 볼캐스터·보조 바퀴는 마찰만 단순화한다.

## 3. 실차 base 대체

- Gazebo diff drive(`odom→base_footprint` TF)와 ray LiDAR `/scan`을 쓴다.
- `nav_safety_gate`는 실제 코드와 `nav_safety.yaml`을 그대로 쓰고, 출력 `/cmd_vel_safe`만 diff drive에 연결한다. v012 시험 때만 `max_linear_speed` 0.13 override를 준다.
- OpenCR 브리지 흉내 shim: `src/drive_pkg/drive_pkg/opencr_bridge_node.py`의 `send_command_tick`을 읽고 똑같이 구현한다. `/drive/ready` 발행, 명령 0.5 s timeout이면 0, 좌우 목표 rpm 중 하나라도 48 초과면 비율 축소 없이 0 출력 + ready false, 60 rpm/s slew.
- 지면 진실 pose(Gazebo model state 또는 p3d)를 토픽으로 낸다. 얻을 수 없으면 멈추고 질문한다.
- 가능하면 `/ros2_ws` 경로 구조를 유지해 `scripts/field_map_guard.py`, `scripts/record_field_bag.sh`, `scripts/analyze_nav_bag.py`, `scripts/field_pose_capture.py`를 그대로 쓴다. 노드 이름 전제(`/rplidar`, `/packagu_opencr_bridge`) 때문에 `start_field_navigation.sh`가 막히면 시뮬 전용 시작 스크립트를 따로 만든다. 그 스크립트도 `field_map_guard.py --stage pre-nav`를 호출한다.

## 4. 시나리오

모든 시나리오는 bag에 지면 진실 pose, `/amcl_pose`, `/plan`, costmap, `/cmd_vel`·`/cmd_vel_safe`, `/rosout`을 기록한다. 지면 진실 footprint 다각형과 월드 벽의 최소 거리를 시계열로 계산하고, 각 조건을 3회 반복한다. 파라미터 조건은 `P0 = nav2_params_pre_wallpush_20260915.yaml`, `P1 = nav2_params.yaml`(현 기본, wall_push_v1)이다.

- **S1 F2 벽 근접:** 스폰 `f2_delivery_destination`. goal (a) `f2_elevator_entry`, (b) `f2_elevator_staging_v1`. 각각 P0·P1. 지정 코너 `(-12.95,-5.25)` 반경 3 m 안의 지면 진실 최소 벽 거리, collision-ahead 수, 경로 형상, 소요 시간을 비교한다. 오프라인 근사(0.552→0.886 m)와 방향이 같은지도 적는다.
- **S2 F1 v3 정상 경로:** 스폰·initial pose 모두 idle 시드 `(-4.518,2.179,-1.701)` → `f1_locker` → `f1_elevator_entry` → idle 시드. P0·P1. 유리문 열린 절반으로 지나는지 본다.
- **S3 F1 초기 pose 오차 주입:** 스폰은 idle 시드, AMCL initial pose는 옛 `f1_idle (-3.725,2.075,-1.701)` → `f1_locker`. P0·P1. 9/14처럼 잘못된 문 접근이나 `Starting point in lethal space`가 재현되는지, AMCL이 스스로 복구하는지 본다. `field_pose_capture.py`로 goal 전 footprint lethal 검사가 이 상황을 잡아내는지도 확인한다.
- **S4 추가 후보 (최대 2개, 시뮬 전용 params 파일):** P1로도 S1·S3에서 벽 거리 0.20 m 미만이나 lethal-start가 남을 때만 만든다. 예: footprint 전체를 검사하는 계획기, inflation 추가 조정. `base_footprint` 원점 이동은 URDF·odom·TF 전체에 영향이 있으니 제안만 하고 구현하지 않는다.
- **S5 증속:** `nav2_params.yaml`(0.10·0.25), `nav2_params_speed_v011.yaml`, `nav2_params_speed_v012.yaml`(+gate 0.13)로 S2 경로를 반복한다. shim이 48 rpm 초과로 0을 낸 횟수, ready 이탈, 도착 여부, 시간을 비교한다.
- **S6 운용 리허설:** map guard(pre-nav/goal), `record start/check/stop` 흐름, `analyze_nav_bag.py`, `field_pose_capture.py`를 시뮬 bag과 시뮬 스택에서 끝까지 실행한다. 스크립트 결함이 나오면 수정안과 테스트를 시뮬 쪽 또는 별도 파일로 만들고, 현장 스크립트는 고치지 말고 보고한다.

## 5. 산출물과 커밋

- `sim/real_maps/`(또는 기존 구조에 맞는 위치): 월드 생성기와 정합 검사, noglass 지도 생성, 로봇 sim 설정, shim, 시뮬 시작 스크립트, 시나리오 러너(스폰·initial pose·goal·반복·결과 CSV 일괄 처리).
- `docs/sim/2026-09-xx_real_map_gazebo_report.md`:
  - 환경
  - 모델·지도 정합 오차
  - 시나리오별 표(3회 반복, 지면 진실 최소 벽 거리, action 결과, collision-ahead, 48 rpm 차단, 소요)
  - P0 대 P1 결론과 추가 후보 권고
  - **시뮬로 검증 불가한 항목:** 수동 지도 절대 오차, 유리·반사 재질, 보조 바퀴·문턱, 모터 부하·제동·온도, 실제 펌웨어, 실제 AMCL 노이즈
- 새 테스트는 `scripts/run_offline_tests.sh` 규칙(ROS 없으면 SKIP)에 맞추고, 러너에 등록했다면 기존 기준선과 비교해 보고한다.
- 커밋 전 검사: 텍스트 파일 CR 0, `git diff --stat`이 파일 전체 규모가 아닌지, 100 MB 초과 파일 없음, 비밀 패턴 grep(`token`, `password`, `BEGIN .* PRIVATE KEY`). 큰 bag은 커밋하지 말고 요약 CSV·그림만 올린다.
- `lee/sim-real-maps`에만 커밋·push한다. push 전에 `git pull --ff-only`로 원격 변경을 받는다. force push·PR 금지.

## 멈추고 질문할 조건

- sudo·패키지 설치가 필요할 때
- 5월 시뮬 자산이 Humble/현재 URDF와 맞지 않아 대규모 재작성이 필요할 때
- 월드–지도 정합 오차가 0.05 m를 넘을 때
- 지면 진실 pose를 얻을 수 없을 때
- 브랜치에 `e223b7532d98` 미러 커밋이 없을 때

## 완료 보고

바꾼 파일, 시나리오별 핵심 수치(특히 S1 코너 최소 벽 거리 P0 대 P1, S3 lethal-start 재현 여부), 추가 후보 권고안, 실물로만 확인할 수 있는 항목, 커밋 SHA를 간결하게 보고한다.

---
