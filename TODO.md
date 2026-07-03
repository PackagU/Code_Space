# TODO — 종설_6조

> Claude Code와 Codex CLI가 공유하는 **세션 단위 휘발성 메모/lock 보드**.
> 영속 작업은 GitHub Issues 로, 큰 그림은 `Roadmap/` 으로.
> 세션 시작 시 읽고, 세션 종료 시 업데이트 후 커밋.

---

## 🔒 현재 편집 중
(없음)

---

## 🗳 다음 회의 안건 (혼자 결정 불가)

## 메모
- 2026-07-03: **GitHub 이관 + 진단 P1~P10 실행 세션.** (1) `PackagU/Code_Space` main/dev push — 새 공개 히스토리(일지·webm·legacy 제외 190파일), 이전 31커밋은 로컬 `archive/pre-github-main`+`lee/l3-cleanup`에 보존, CI(check.yml) green. (2) `legacy/` 로컬 아카이브 신설(수동 floor_orchestrator_pkg, phase.md, 루트 nav2_params.yaml, FeedBack/, 루트 docker 사본 등 — `legacy/README.md` 목록 참조, GitHub 업로드 금지). (3) `docker/` 폴더가 컨테이너 SSOT — 개발 amd64 + `Dockerfile.jetson`(aarch64) + compose 3종, `run_kku_sim.sh` 경로 전환, publish-ghcr 워크플로우 docker/ 기준 보정. (4) 맵 기대값 단일소스화(`map_expectations.py`), verifier `--floor/--from-floor` 인자화. (5) **F2→F3 전환 검증 완료**: `WITH_F3=1` smoke — F1→F2→F3 전체 체인 PASS (F3는 실측 전까지 F2 복제 레이아웃). (6) 신규 원커맨드: `scripts/bootstrap_workspace.sh`(fresh clone 초기화), `scripts/run_offline_tests.sh`(19종 통합), `scripts/check_portability.py`(이식성 가드 R1~R4). (7) AGENTS.md 전면 정합화 + `docs/hardware_spec.md`(HW SSOT) + `docs/deployment/01_portability_policy.md` 신설, Roadmap 상태 실제 진행 반영. (8) 발견 버그 수정: 루트 colcon 빌드가 test_workspace 패키지 흡수(stale ament index로 Gazebo 기동 실패) → `--base-paths src` 한정+회귀 가드, sparse-checkout 잔재 해제. **다음 세션**: `git pull origin dev` 후 시작. Notion 동기화(`sync_docs_to_notion.py`)로 hardware_spec/deployment 신규 문서 업로드 확인 필요. 참고: `.gitignore`가 `docs/session_wiki/`를 로컬 전용화(추적 목록엔 남아있어 이 브랜치 로컬 커밋에는 무해).
- 2026-06-29 (3차, 후속 goal): world-swap sim-to-real 후속 항목을 "복도 폭 2.0m 축소"만 제외하고 구현·검증·문서화함. 추가: `run_l3_world_swap_smoke.sh`에 gated 옵션 3종(`WITH_RECOVERY`/`WITH_LOC_FAULT`/`WITH_PROFILE`, 기본 0)과 타임스탬프 아티팩트 수집(`verification/run_<ts>/` + `scenario_summary.md`); 신규 `scripts/profile_resources.sh`(host/container/Jetson 공용 CPU/mem 샘플러); `test_smoke_scripts_contract.py` stale assert 수정(`f2_corridor 2.0`→`2.5`) + 회귀 가드(CORRIDOR_HALF=2.5/F2맵 498x348/옵션 기본값). 문서: `docs/session_wiki/.../2026-06-29_followup_goal_results.md`(검증표·시나리오 매트릭스·failure mode catalog·엘베 실측 분리·팔/리프트 계약·Jetson 명령·인계), `improvement_report.md` §1.2 갱신 + §1.4~1.8 신규. **다음 세션 3 명령(컨테이너 /ros2_ws)**: (1) `python3 test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py` (2) `bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh` (3) `WITH_RECOVERY=1 WITH_PROFILE=1 bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh`. **건드리면 안 됨**: `generate_kku_worlds.py` `CORRIDOR_HALF = 2.5`(5m 테스트 복도), F2 맵 498x348/origin -2.46,-2.96, smoke 옵션 기본값 0. **needs measurement**: 복도 실측 2.0m, 엘베 캡 깊이/문턱 높이, 적재물 질량/COM.
- 2026-06-29 (2차): Gazebo world swap full smoke를 실제 시연 흐름에 맞게 확장함. `run_l3_world_swap_smoke.sh`가 F1 `charge_station`에서 시작해 `/initialpose` 발행, F1 `parcel_storage`/`parcel_pickup` 방문, F1 `elevator_entry`/`elevator_inside` 복귀 후 F2 world swap과 F2 corridor goal을 수행함. `WITH_RVIZ=true`로 RViz 관찰 가능, `KEEP_RUNNING=1`로 종료 후 프로세스 유지 가능. Docker full smoke PASS, 오프라인 테스트 6종 PASS. AGENTS.md에 E2E 원커맨드 정책 추가. Notion 업로드는 수행하지 않음.
- 2026-06-29: Gazebo World Swap Claude Code 재검증 결과를 `docs/session_wiki/2026-06-25_gazebo_world_swap/2026-06-29_world_swap_test/`에 정리함. 포함 문서: `README.md`, `claude_review_result.md`, `test_procedure_atoz.md`. Claude 검토 판정은 PoC 기준 merge 가능, 요구사항 충족. 후속 하드닝: Gazebo delete/spawn 응답 대기 단계 timeout 추가, swap 실패 후 같은 floor 재시도 가능 구조로 개선. Notion 업로드는 수행하지 않음.
- 2026-06-25: Gazebo World Swap PoC 구현 및 검증 완료. 새 workspace `test_workspace/gazebo_world_swap/` 추가, `gazebo_world_swap_pkg`가 `/floor_orchestrator/status`를 관찰해 F1→F2 ready 후 `/delete_entity` + `/spawn_entity`로 `kku_f1_building`을 `kku_f2_building`으로 교체함. `src/common_pkg/launch/gazebo.launch.py`는 `spawn_point:=elevator_exit|elevator_inside` 인자 추가(기본 exit 유지). 원커맨드 검증: `docker exec -it ros2_humble bash` → `cd /ros2_ws` → `bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh`. 최종 PASS: F2 map(477x299, origin -1.99,-1.0), Gazebo `kku_f2_building` present / `kku_f1_building` absent, `/scan` finite, F2 corridor goal `(2.0,12.0)` SUCCEEDED. 한계: named waypoint `208` `(2.35,12.0)` 직접 goal은 planner ABORTED — `docs/improvement_report.md`에 waypoint reachability 개선 항목 추가.
- 2026-06-12 (2차): 세션 일지 + 장기 로드맵 수립. (a) `docs/session_wiki/2026-06-12_auto_map_switch/` 6개 문서 작성 후 `sync_docs_to_notion.py`로 Notion 업로드. (b) repo 루트 `Roadmap/` 폴더에 8단계 로드맵 작성: 01 PoC 마무리(L3/L5+좌표 캡처) → 02 URDF 실스펙 → 03 실측 맵 → 04 동적 장애물 → 05 엘베 내부 시뮬+멀티층 자동 전환 → 06 통합 반복 안정성 → 07 Jetson 배포(aarch64 이미지 — 1주차부터 병렬 시작 권장) + 08 안전/보안 게이트(횡단). 각 단계에 작업 체크박스/완료 기준/리스크 포함. Han(HW 스펙, 물리 E-stop)·Kim(엘베 버튼 로봇팔) 협의 항목은 각 문서에 표시 — 회의 안건으로 올릴 것.
- 2026-06-12: 자동 맵 전환 PoC 구현 완료. `test_workspace/elevator_auto_map_switch/`에 `auto_floor_orchestrator_pkg`(노드명 `floor_orchestrator_node` — legacy SwitchFloor의 `/floor_orchestrator_node/set_parameters` 하드코딩 호환), `config/floor_maps.yaml`, 오프라인/ROS 테스트 4종, `probe_nav2_services.sh` 추가. 검증: L0/L1/L2/L4 host+container PASS, 컨테이너 colcon build PASS, legacy 회귀 5종 PASS, legacy 무수정. 미수행: L3(실제 Nav2 load)/L5(E2E)는 Gazebo 세션 필요 — 절차는 completion_report.md §5. 설계 판단 기록: 워크스페이스 루트 `FeedBack/` 8개 문서 (initialpose 주체=orchestrator, 실패 시 pending=true 유지, map 경로 3단계 resolve 등).
- 2026-06-02: 자동 맵 전환 PoC 계획을 Notion `session_wiki`에도 업로드함. 페이지: `2026-06-02 Elevator Auto Map Switch Planning` (`3739c69b-0345-815b-ab28-d5fb6ff207fe`). 시뮬레이션 성공 이미지 첨부 칸을 페이지 상단에 만들어 두었고, 추후 Gazebo/RViz 또는 mission 성공 화면을 첨부하면 됨.
- 2026-06-02: 자동 맵 전환 PoC용 신규 테스트 폴더 생성 완료. 위치: `test_workspace/elevator_auto_map_switch/`. 포함 문서: README, 현재 분석, 시행 계획, 구현 계획, 검증 계획, 2026-06-01/2026-06-02 일지. 기존 `test_workspace/elevator_mission/`은 수정하지 않고 legacy 수동 2-Phase 폴더로 보존함. 검증: legacy `test_point_registry.py`, `test_behaviors_dryrun.py` 통과.
- 2026-06-02: 엘리베이터 층 이동 확인 이후 다음 단계로, 엘베 도착 시 자동으로 목표층 맵을 전환하는 PoC는 오늘 구현하지 않고 별도 테스트 폴더에서 계획/분석부터 시작하기로 함. 기존 `test_workspace/elevator_mission/` 수동 2-Phase 폴더는 legacy로 보존하고 수정하지 않는다. 신규 작업 공간은 `test_workspace/elevator_auto_map_switch/`로 분리한다.
- 2026-06-01: waypoint 기반 직각 라우터 추가. `test_workspace/elevator_mission/scripts/orthogonal_router.py`가 층별 복도 중심선(`main_corridor_x`/`main_corridor_y`)을 기준으로 ㄱ/ㄴ자 route waypoint를 만들고, `delivery_mission_node`는 단일 `NavigateToPose` 대신 `NavigateRoute`로 경유점을 순차 전송함. F2/F3 방 이동은 `x=2.0` 복도 중심선을 따라 올라간 뒤 방 앞에서 90도 접근하고, F1 택배존은 `y=0.0` 주 복도축을 따라 이동 후 진입함. Nav2 코너 통과가 뭉개지지 않도록 `src/slam_pkg/config/nav2_params.yaml`의 goal tolerance도 조정함.
- 2026-05-28: workspace 내부에 `SLAM/` 폴더를 생성하고 `https://github.com/PackagU/Road_MAP.git`을 clone해 SLAM 정리용 독립 repo로 연결함. `main` 브랜치는 README 삭제 상태라 파일이 없고, 실제 운영 브랜치는 `setup/linux-slam-workspace`로 설정함. 앞으로 문서(`docs/`)와 test/검증용 코드를 제외한 SLAM 구현 코드, launch/config/yaml/rviz/world, 민감정보가 없는 map 백업은 `SLAM/` 안에 저장 후 `origin/setup/linux-slam-workspace`에 commit/push한다. 부모 repo에는 nested checkout이 섞이지 않도록 `.gitignore`에 `/SLAM/` 제외 규칙을 추가함. 세션 시작 규칙의 `git pull origin dev`는 원격 `dev` 브랜치가 없어 실패함.
- 2026-05-27: `docs/` Notion 위키 단방향 동기화 인프라 구축 + 첫 업로드 완료. 추가: `scripts/sync_docs_to_notion.py` (manifest 기반, idempotent), `docs/_style/notion_markdown_style.md` (Notion + Obsidian 호환 작성 규칙 SSOT — AGENTS.md #10에서 라우팅), `docs/.notion_sync.json` (path → page_id 매핑, git 추적), `scripts/requirements_notion_sync.txt`, `.env.example`. 노션 `📚 문서 위키` 페이지 (`36d9c69b-0345-8141-be1b-e2baff251bda`) 아래 28개 페이지 생성됨. 사용법: `python scripts/sync_docs_to_notion.py [--dry-run]`. 알려진 함정 두 개 (모두 스크립트에서 해결됨): (a) update 시 자식 sub-page도 children block으로 잡혀 archive 되는 문제 → `child_page` block은 삭제 대상에서 제외. (b) http(s)/mailto 아닌 link href는 노션이 거부 → plain text로 변환.
- 2026-05-27: 어제 저녁부터 현재까지의 대화/수정 파일/디버깅/진행상황 위키 정리 생성. 위치: `docs/session_wiki/2026-05-27_slam_debug/`. 포함: 자료 수집 범위와 정리 계획, KST 타임라인, 수정 파일 목록, 로봇 미이동/WASD/Nav2 디버깅 로그, SLAM/Nav2 기본 명령어, 현재 상태와 다음 작업. 민감정보/OAuth 값은 제외함.
- 2026-05-27: `src/slam_pkg/launch/kku_navigation.launch.py` 추가. `floor:=F1|F2|F3`로 저장 맵 YAML을 선택하고 Nav2 `bringup_launch.py`를 include하도록 구성. `src/slam_pkg/package.xml`에 `nav2_bringup` exec_depend 추가. 검증: `python3 scripts/test_kku_navigation_launch.py`, `python3 -m py_compile src/slam_pkg/launch/kku_navigation.launch.py scripts/test_kku_navigation_launch.py`.
- 2026-05-27: `docs/simulation_test/03_nav2_one_floor/02_create_kku_navigation_launch.md`의 `nav2_params.yaml` 확인 부분 보강. `use_sim_time: true` 같은 YAML 예시는 터미널 명령이 아니라 파일 내부 값임을 명시하고, 확인용 `grep` 및 `nano` 명령을 추가함.
- 2026-05-27: 시뮬레이션/SLAM/Nav2 관련 문서를 `docs/simulation_test/` 아래로 재구성. `01_environment`, `02_gazebo_slam_mapping`, `03_nav2_one_floor`, `04_point_goal_delivery`, `99_reference`로 단계별 분리하고, 저장 맵 Nav2 실행에서 빠졌던 `kku_navigation.launch.py`/`nav2_params.yaml` 생성 절차를 `docs/simulation_test/03_nav2_one_floor/02_create_kku_navigation_launch.md`에 추가함. `.gitignore`는 `docs/simulation_test/**`를 추적 예외로 갱신.
- 2026-05-27: F1/F2/F3 저장 맵 이후 단계 문서 추가. `docs/simulation_test/04_point_goal_delivery/01_kku_nav2_point_goal_workflow.md`에 point goal 기반 자율주행 구조, Nav2 bringup 필요 파일, `kku_nav_points.yaml` 설계, RViz goal 테스트, 장애물 회피 검증, 엘리베이터 상태머신 확장 순서를 정리함. 핵심 주의사항: Gazebo 설계 좌표와 SLAM 저장 맵 `map` 좌표는 첫 Nav2 테스트에서 RViz로 확인 후 확정해야 함. `.gitignore`에 해당 문서 예외도 추가.
- 2026-05-27: KKU WASD 방향 반전/정지키 재점검. 원인: teleop 매핑이 아니라 `src/common_pkg/urdf/robot.urdf.xacro`의 `right_wheel_joint` rpy가 왼쪽과 반대로 되어 diff_drive에서 `linear.x`가 회전처럼, `angular.z`가 직진처럼 동작함. 오른쪽 바퀴 rpy를 `-1.5708 0 0`으로 맞추고, `k`/Space 정지 입력 시 `stop: cmd_vel=(0.00, 0.00)` 로그를 출력하도록 `keyboard_teleop.py` 개선. 검증: `python3 scripts/test_wasd_teleop.py`, `bash scripts/test_kku_sim_scripts.sh`, `python3 -m py_compile ...`, 컨테이너 `colcon build --symlink-install --packages-select common_pkg drive_pkg`, 컨테이너 xacro 변환.
- 2026-05-27: KKU WASD teleop 속도 체감 혼동 재점검. `process_key()`를 추가해 현재 키 상태별 실제 `/cmd_vel` 계산을 테스트 가능하게 분리하고, 속도 변경 로그에 `linear/angular` 변경 종류와 현재 `cmd_vel=(linear_x, angular_z)`를 출력하도록 개선. 컨테이너 설치 파일 기준 확인: `w -> q`는 linear cmd 증가, `a -> q`는 angular cmd 유지, `a -> e`는 angular cmd 증가.
- 2026-05-27: KKU WASD teleop 속도 조절 키 분리. `q/z`는 선속도만 증가/감소, `e/c`는 각속도만 증가/감소하도록 `src/drive_pkg/drive_pkg/keyboard_teleop.py` 수정. 도움말 및 KKU 실행 문서 반영. 검증: `python3 scripts/test_wasd_teleop.py`, `python3 -m py_compile src/drive_pkg/drive_pkg/keyboard_teleop.py scripts/test_wasd_teleop.py`, `bash scripts/test_kku_sim_scripts.sh`, 컨테이너 `colcon build --symlink-install --packages-select drive_pkg`.
- 2026-05-27: KKU teleop을 WASD 방식으로 교체. `src/drive_pkg/drive_pkg/keyboard_teleop.py` 추가, `scripts/teleop.sh`는 `ros2 run drive_pkg keyboard_teleop` 실행으로 변경. 키: `w/s/a/d` 전진/후진/좌회전/우회전, `k` 또는 Space 정지, `q/z` 선속도 증가/감소, `e/c` 각속도 증가/감소. `run_kku_sim.sh` 빌드 대상에 `drive_pkg` 포함. 검증: `python3 scripts/test_wasd_teleop.py`, `bash scripts/test_kku_sim_scripts.sh`, `python3 -m py_compile ...`, 컨테이너 `colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg`.
- 2026-05-27: KKU 시뮬레이션 로봇 미이동 문제 조사. `/cmd_vel`은 `diff_drive`가 구독 중이었지만 `/clock`, `/scan`, `/odom`이 timeout이었고 오래된 `gzserver kku_f1.world`와 새 `F2` spawn 프로세스가 섞여 있었음. 현재 stale Gazebo/RViz/SLAM 프로세스 정리 완료. `scripts/run_kku_sim.sh`에 실행 전 stale 프로세스 정리 단계 추가, `docs/simulation_test/02_gazebo_slam_mapping/03_kku_simulation_quickstart.md`에 `/clock`/`/odom` 진단 및 재실행 안내 추가. 로그: `logs/2026-05-27_13-53/summary.md`.
- 2026-05-27: `docs/simulation_test/02_gazebo_slam_mapping/03_kku_simulation_quickstart.md`에 완전 종료 상태에서 시작하는 VSCode host 터미널 흐름 추가. 포함: `xhost`, `docker compose ... up -d`, `docker ps`, `Ctrl+Shift+P -> Dev Containers: Attach to Running Container... -> ros2_humble`, `/ros2_ws` 열기, 컨테이너 내부 `colcon build`, `ros2 launch`, teleop, `/scan`/`/map` 확인.
- 2026-05-27: KKU 시뮬레이션 초간단 실행 스크립트 추가. `./scripts/run_kku_sim.sh F1`로 X11 권한, Docker compose up, colcon build, 통합 launch를 한 번에 수행. `./scripts/teleop.sh`로 조종, `./scripts/save_kku_map.sh F1`로 표준 경로에 map 저장. 사용 문서: `docs/simulation_test/02_gazebo_slam_mapping/03_kku_simulation_quickstart.md`. 검증: `bash scripts/test_kku_sim_scripts.sh`, `bash -n scripts/run_kku_sim.sh scripts/teleop.sh scripts/save_kku_map.sh scripts/test_kku_sim_scripts.sh`.
- 2026-05-27: Linux desktop VSCode에서 Docker `ros2_humble` 컨테이너 attach 후 KKU 시뮬레이션을 실행하는 A-to-Z 문서 추가. 저장 위치: `docs/simulation_test/01_environment/03_vscode_docker_kku_simulation_atoz.md`. 포함: X11 권한, compose up, VSCode attach, `/ros2_ws` 열기, colcon build, `slam_pkg kku_simulation.launch.py floor:=F1|F2|F3`, teleop, `/scan`/`/odom`/`/map` 확인, 층별 map 저장, GUI/ament 오류 대응.
- 2026-05-27: 시뮬레이션 바로 돌릴 수 있는 상태까지 완성. 완료 보고서 = `docs/simulation_test/02_gazebo_slam_mapping/02_kku_pre_simulation_completion_report.md`. 추가된 것: `common_pkg/launch/gazebo.launch.py` (floor 인자), `slam_pkg/launch/kku_simulation.launch.py` (Gazebo + SLAM Toolbox + RViz 통합), `slam_pkg/config/slam_view.rviz`. 검증 = python syntax + xacro + xml well-formed (호스트), 실제 Gazebo 실행은 컨테이너에서. 다음 = 보고서 §2.1 컨테이너 첫 실행 → §2.2 상태머신/waypoint.
- 2026-05-27: Gazebo world 3개 생성 완료. `scripts/generate_kku_worlds.py`가 YAML(`src/common_pkg/config/kku_pre_simulation_map.yaml`)을 읽어 `src/common_pkg/worlds/kku_f{1,2,3}.world`를 출력. 구조: 엘베(7 walls, 동측 1.0m 문 + corridor shoulder 2개) + right corridor(F1=15m, F2/F3=22m, dead-end 끝벽) + back corridor(F1=8m form-only, F2/F3=14m with 8개 방) + F1 alcove 택배존(0.9m 문, 외곽 3벽). 벽 두께 0.12m, 높이 2.4m. YAML 변경 시 `python3 scripts/generate_kku_worlds.py` 재실행. 다음 = 층별 Gazebo launch.
- 2026-05-26: `docs/simulation_test/02_gazebo_slam_mapping/01_kku_pre_simulation_plan.md` 평가 후 수정 완료. 결정사항: 택배존을 1F 오른쪽 복도 남쪽 alcove로 모델링, 복도 끝은 모두 dead-end 끝벽, 엘베 문 폭 0.9→1.0m, 방 크기 2.5×3.0m, 층별 frame_id `map_f1/f2/f3` 분리, F1 뒤쪽 복도는 형태만 유지(방 없음). 완료 기준은 "Codex 작업 6항목이 명령대로 수행됨"으로 단순화.
- 2026-05-26: 다음 작업 = (1) YAML(`src/common_pkg/config/kku_pre_simulation_map.yaml`)을 수정된 plan에 맞춰 정렬: 엘베 문 0.9→1.0m, 복도 끝벽 세그먼트 추가, frame_id 필드 추가, 방 크기 명시. 그 다음 (2) Codex로 `kku_f1/f2/f3.world` 생성.
- 2026-05-27: YAML 정합화 완료. 변경: `coordinate_system.per_floor_frame_ids`, 각 floor `frame_id`, 엘베 문 3곳 1.0m, 6개 corridor `closed_ends:[end]`, `global_defaults.room_size{2.5,3.0}` + 문 폭 3종 분리(`room_door_width`/`elevator_door_width`/`parcel_zone_door_width`), `parcel_zone` shape=alcove + `enclosure{door:north 0.9m, outer_walls:[s,e,w]}`, F1 back_corridor `form_only_no_rooms_for_slam_consistency` notes, F2/F3 right_corridor `no_rooms_on_this_corridor` notes. 다음 단계 = Codex로 `src/common_pkg/worlds/kku_f{1,2,3}.world` 생성.
- 2026-05-27: 건국대 신공학관 레퍼런스 기반 pre-simulation 맵 스펙 재수정. 구조는 엘베 하차 기준 오른쪽 복도는 유지하되 방을 두지 않고, 방은 엘베 왼쪽 코너를 돌아 들어가는 뒤쪽 복도 양쪽 벽에만 마주보게 배치. 배송층 호수는 `201~208`, `301~308`. 저장 위치: `src/common_pkg/config/kku_pre_simulation_map.yaml`, 계획서: `docs/simulation_test/02_gazebo_slam_mapping/01_kku_pre_simulation_plan.md`.
- 2026-05-26: 이전 해석 기록. 오른쪽 복도와 뒤쪽 복도 모두에 방을 두는 안은 2026-05-27 수정으로 폐기.
- 2026-05-04: `AGENTS.md`에 외부 업로드 금지 및 토큰/secret 접근 금지 공통 지침 추가
- 2026-05-13: 외부 반출 금지 범위를 민감정보(Notion API 값, token, secret, credential, `.env`, SSH key, auth/session 값, 개인정보 등)로 축소. 민감정보가 아닌 코드/문서/로그/설정은 사용자 허가 또는 팀 정책에 따라 GitHub/DockerHub/Notion 등에 공유 가능.
- 2026-05-13: `PackagU/ros2-humble-slam-docker` Docker 공유 repo 생성 및 GHCR publish workflow 성공. 팀원 전용 운영을 위해 repo/GHCR package는 private 유지가 맞음. 팀원은 GitHub 권한 + GHCR 로그인 후 pull.
- 2026-05-13: `docs/simulation_test/01_environment/01_linux_docker_slam_setup.md`를 private GHCR + `docker_env` 기준으로 업데이트. Linux desktop 세팅은 이 문서의 `gh auth` → `docker login ghcr.io` → `docker compose pull/up` 순서로 진행.

### 자동 코드 리뷰 도구 도입 검토
PR 마다 자동으로 코드 리뷰 코멘트를 다는 도구가 있음. 도입 여부 / 어느 도구 / 비용 부담 주체를 회의에서 결정 필요.

**옵션**
| 도구 | 비용 | 비고 |
|------|------|------|
| Claude Code Action (anthropics/claude-code-action) | Anthropic API 종량제, 월 $5-40 | AGENTS.md 인식, 품질 최상 |
| GitHub Copilot Code Review | Copilot Pro 구독 (학생 무료 가능) | 가입 후 체크박스 한번 |
| CodeRabbit | Public repo 무료 / private 유료 | repo 가시성에 따라 결정 |
| 도입 안 함 (린터만) | 무료 | ament_lint + ruff + clang-format |

**결정 항목**
- [ ] 도입 여부
- [ ] 도입 시 도구 선택
- [ ] 비용 부담자 (개인 키 vs 팀 공용 계정)
- [ ] 모델/요금제 (Sonnet vs Haiku 등)
- [ ] 트리거 — 모든 PR vs 라벨 붙은 PR 만

**관련 자료**: 회의 후 결정되면 `docs/code_review_policy.md` 작성 + 워크플로우 추가.

### plan/ + GitHub Issues 운영 시작
신규 도입한 4-layer 체계(roadmap / plan / Issues / TODO) 의 운영 시작 일자와 첫 Issue 작성 책임자를 정해야 함.

**결정 항목**
- [ ] 운영 시작일 (당장 vs 다음 주부터)
- [ ] M1 Milestone 생성 + 5/31 마감 등록 책임자
- [ ] ROAD MAP Project 보드 초기 세팅 (`docs/project_board.md` §6 체크리스트) 책임자
- [ ] 라벨 동기화 (`.github/labels.yml` → repo) 책임자
- [ ] `plan/*.md` 의 각 영역 파일을 본인이 직접 검토 + 보강

---

## 🔴 우선순위 높음

- [ ] **[NEXT]** 자동 맵 전환 PoC L3/L5 시뮬 검증 (Gazebo + Nav2 실세션 필요)
      {절차는 `test_workspace/elevator_auto_map_switch/docs/completion_report.md` §5. 사전에 `scripts/probe_nav2_services.sh`로 서비스명 확인 후 `dry_run_map_load:=false`. L5는 좌표 캡처(legacy 보고서 §4) 선행 권장}
- [x] 엘베 도착 시 자동 맵 전환 PoC 계획 검토 후 구현 (2026-06-12)
      {`auto_floor_orchestrator_pkg` 구현 완료. L0/L1/L2/L4(오프라인) host+container PASS, colcon build PASS, legacy 회귀 5종 PASS. 판단 기록은 `FeedBack/` 폴더. 핵심: 노드명은 legacy SwitchFloor가 하드코딩한 `floor_orchestrator_node` 유지 — legacy 수동 orchestrator와 동시 실행 금지}
- [ ] **[NEXT]** 엘리베이터 층 이동 상태머신 초안 + Nav2 waypoint 변환 — 보고서 §2.2
      {py_trees_ros 기반 BT, World swap + robot respawn, 신규 `test_workspace/elevator_mission/` 에 PoC}
- [x] GitHub 레포 초기 push (2026-07-03) — `PackagU/Code_Space` main+dev, CI green. 브랜치 전략은 AGENTS.md §GitHub 워크플로우.
- [ ] GitHub 개선 — 팀원 초대 확인, main 브랜치 보호 설정, Issue 라벨 세팅 (회의 안건과 연동)
- [ ] VcXsrv + Docker GUI 연결 확인 (RViz2 또는 Gazebo 실행)
      {컨테이너 안에서 rviz2 실행 → GUI 창이 Windows에 뜨면 성공}

## 🟡 진행 예정

- [ ] SLAM Toolbox 파라미터 초기 튜닝 (RPLiDAR A1m8 기준)
- [ ] URDF 로봇 모델 초안 (common_pkg) — 1차 작성됨, Fusion 결과 반영 필요
- [ ] drive_pkg 기본 구조 (Han)
- [ ] robot_arm_pkg 기본 구조 (Kim)
- [ ] Gazebo 시뮬레이션 환경 구성

## 🟢 완료

- [x] 컨테이너에서 시뮬 첫 실행 검증 (2026-05-27) — F1/F2/F3 SLAM map 저장 완료, WASD teleop 동작 확인. 보고서 §2.1.
- [x] `docs/` → Notion 위키 단방향 동기화 인프라 구축 + 첫 업로드 (2026-05-27) — `scripts/sync_docs_to_notion.py`, `docs/_style/`, AGENTS.md #10. 28페이지 노션 생성.
- [x] `src/common_pkg/config/kku_pre_simulation_map.yaml` 을 수정된 plan과 정합화 (2026-05-27)
- [x] Gazebo world 3개 생성 (2026-05-27) — `src/common_pkg/worlds/kku_f{1,2,3}.world` (F1 18 walls, F2/F3 각 46 walls)
- [x] 층별 Gazebo launch + 통합 SLAM launch + RViz config (2026-05-27) — 완료 보고서 `docs/simulation_test/02_gazebo_slam_mapping/02_kku_pre_simulation_completion_report.md`
- [x] `slam_pkg` 통합 launch (Gazebo + SLAM Toolbox + RViz) (2026-05-27)
- [x] SLAM 시뮬레이션 + 자체 제작 테스트 지도 구성
- [x] Docker 컨테이너 빌드 테스트 (`docker compose up --build`)
- [x] Linux desktop VSCode에 Codex CLI, Claude Code, Gemini CLI 설치
- [x] OpenCR + Dynamixel 2개 바퀴 구동 테스트 A to Z 문서 작성 — `docs/opencr_dynamixel_wheel_test.md`
- [x] Linux desktop Docker + ROS2 SLAM A to Z 가이드 작성 — `docs/simulation_test/01_environment/01_linux_docker_slam_setup.md`
- [x] 워크스페이스 폴더 구조 생성
- [x] CLAUDE.md → AGENTS.md 분리 (Claude Code + Codex CLI 공통화)
- [x] Docker 파일 템플릿 작성
- [x] `plan/` 폴더 + 6개 영역별 계획 파일 작성
- [x] `.github/ISSUE_TEMPLATE/` task/bug/spike 템플릿 + `labels.yml`
- [x] `docs/project_board.md` ROAD MAP 자동화 가이드
- [x] `docs/improvement_report.md` 운영 지침 (AGENTS.md 에 등재)

---

## 메모

- ROS2 Domain ID: 0 (기본값 유지)
- DISPLAY: `host.docker.internal:0.0` (VcXsrv)
- Docker base image: `my_ros2_humble` (DockerHub, Kim 관리)
- M1 마감: **2026-05-31** — 하드웨어 스펙 확정 + Fusion 모델링 + SLAM/팔 시뮬 동작
# 2026-05-13 세션 업데이트

- Linux desktop Docker + ROS2 Humble SLAM 환경 구성 완료.
- `docker_env` 기준 compose 실행과 `ros2_humble` 컨테이너 실행 완료.
- Docker 권한 문제 해결: `hsm` 사용자를 `docker` group에 추가하고 재로그인.
- X11 GUI 권한 문제 해결: `xhost +local:docker`, `xhost +local:root` 적용.
- VSCode Dev Containers로 `ros2_humble` attach 성공.
- 컨테이너 VSCode에서 `/ros2_ws`를 열고 ROS2 작업 흐름 확인.
- `colcon build --symlink-install` 실패 원인 확인: GitHub가 빈 폴더를 저장하지 않아 `launch/`, `maps/` 폴더 누락.
- 빈 폴더 누락에도 빌드가 깨지지 않도록 `drive_pkg`, `robot_arm_pkg`, `slam_pkg` CMake install 조건 수정.
- Gazebo + SLAM Toolbox + RViz2 실행 흐름 확인.
- map 생성, 저장, RViz2 표시까지 완료.
- VSCode 기반 Docker SLAM 작업법 문서 추가: `docs/simulation_test/01_environment/02_vscode_docker_slam_workflow.md`.
- 상세 시행착오 기록 추가: `obsidian_vault/SLAM/trials/2026-05-13.md`.

## 다음 작업

- 생성한 map 파일명과 저장 위치를 팀 기준으로 정리.
- `/scan`, `/odom`, `/tf` 확인 결과를 다음 실험 로그에 남김.
- SLAM Toolbox 파라미터 비교 실험 시작.
- `resolution`, `max_laser_range`, `loop_search_maximum_distance`별 map 품질 비교.
