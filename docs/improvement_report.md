# Improvement Report

## Active Improvements

### 1.1 🟡 [20%] F2 Room Waypoint Reachability

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-06-25
> 진척: Gazebo world swap 후 Nav2가 F2 map/world 정합 상태에서 `(2.0, 12.0)` corridor goal은 `SUCCEEDED` 했지만, 기존 named waypoint `208` `(2.35, 12.0)` 직접 goal은 planner가 valid path를 만들지 못해 `ABORTED` 됨. 방 앞 목적지 좌표를 free-space doorway/corridor 중심으로 재검토해야 함.

영향:

- `parcel_to_208` 같은 room waypoint 기반 L5 mission이 목적지 좌표에서 실패할 수 있다.
- world swap 자체는 성공해도, 목표점이 occupied cell 또는 inflation 영역에 있으면 planner가 실패한다.

다음 조치:

- F2/F3 room waypoint가 occupancy grid free cell에 있는지 자동 검사한다.
- `kku_nav_points.yaml`의 room 목표를 doorway 접근점과 room 식별점으로 분리한다.
- mission은 Nav2 goal로 free-space 접근점을 사용하고, 도착 후 배송 동작은 별도 상태로 처리한다.

### 1.2 🟡 [40%] Gazebo World Swap Failure-Path Hardening

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-06-29
> 진척: (a) delete/spawn 응답 대기 timeout 구현됨 — `world_swap_node.py`가 `service_wait_timeout_sec`(기본 10s) deadline 초과 시 `failed` status를 publish. (b) 남은 갭: swap 실패(`state=failed`) 후 같은 floor 재시도 가능하게 confirmed-floor rollback 처리, 실패 경로 단위 테스트.

영향:

- Gazebo 서비스가 요청을 받은 뒤 응답을 반환하지 않으면 world swap effect가 무한 대기할 수 있다.
- delete 또는 spawn 실패 후에도 target floor가 이미 처리된 것으로 기록되어 다음 ready status에서 재시도하지 못할 수 있다.
- 정상 smoke에는 영향이 없지만, 장시간 운용이나 반복 테스트에서는 map/world 불일치가 고착될 수 있다.

다음 조치:

- `delete_wait_response`와 `spawn_wait_response` 단계에 deadline timeout 처리를 추가한다.
- timeout 또는 service exception 발생 시 failed status를 publish하고 effect를 정리한다.
- swap 성공 시점에 confirmed floor를 갱신하거나, 실패 시 last floor 상태를 rollback해 같은 floor 재시도가 가능하게 만든다.
- 실패 경로 단위 테스트와 contract test를 추가한다.

### 1.3 🟡 [10%] Charge Station Idle Return Behavior

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-06-29
> 진척: Gazebo full smoke에서 F1 엘리베이터 앞 `(1.6, 0.0)`을 `charge_station` spawn point로 정의하고, 시연 시작 위치로 검증함. 유휴 상태에서 자동 복귀하는 mission/orchestrator 상태머신은 아직 미구현.

영향:

- 현재는 smoke 시작점만 충전 스테이션으로 고정되어 있고, 임무 종료 또는 대기 상태에서 자동으로 복귀하지 않는다.
- 실제 시연에서는 로봇이 임무 후 복도나 배송 위치에 남아 다음 시나리오 시작 pose가 흔들릴 수 있다.

다음 조치:

- mission 상태에 `idle_return` 또는 `return_to_charge_station` 단계를 추가한다.
- 충전 스테이션 좌표를 waypoint registry와 Gazebo spawn point 양쪽에서 같은 의미로 관리한다.
- 임무 완료, 취소, timeout 후 복귀 동작을 각각 테스트한다.

### 1.4 🟡 [50%] Recovery 동작 실전화

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-06-29
> 진척: (a) Nav2 기본 recovery 스택(navigate_to_pose_w_replanning_and_recovery BT + behavior_server spin/backup/drive_on_heading/wait + RecoveryNode 한정 재시도) 확인. (b) `WITH_RECOVERY=1` smoke 옵션 추가 — 도달 불가능 goal로 "무한 회전/무한 재시도 없이 한정 ABORTED" 자동 검증. (c) 남은 갭: 실제 "복도 완전 차단"(물리 장애물) 케이스, planner/controller 서버 crash 후 자동 재기동.

영향:

- 막힘 시 무한 hang/회전을 방지해야 장시간 무인 운용이 가능하다.
- 현재 검증은 planner-failure 경로(도달 불가 goal)만 자동화됨. 물리 차단/노드 crash는 미자동화.

다음 조치:

- 복도 완전 차단용 임시 장애물 spawn 후 backup+재계획 거동을 자동 검증한다.
- lifecycle 노드 crash 시 respawn 또는 명시적 실패 보고 경로를 추가한다.

### 1.5 🟡 [20%] Localization 강건성 (odom noise / aliasing / initialpose 오염)

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-06-29
> 진척: (a) `WITH_LOC_FAULT=1` smoke 옵션 추가 — 0.5m 어긋난 initialpose 주입 후 AMCL+scan 매칭 보정으로 goal 도달하는지 자동 검증. (b) 무특징 직선 복도 aliasing(종방향 특징 부족, 대칭 오인)은 현재 해결 대상이 아니라 문서화 대상으로 분리.

영향:

- 소오차(0.5m)는 복도 양벽 특징으로 보정 가능하나, 대형 오차/무특징 구간은 발산 위험.
- odom 노이즈(실기 IMU/엔코더)는 아직 시뮬에 미반영.

다음 조치:

- 실측 SLAM 맵을 ground truth로 aliasing 위험 구간을 식별한다.
- 대형 오차 시 AMCL global relocalization 트리거 조건을 추가한다.
- diff_drive odom에 노이즈 모델을 주입하는 옵션을 검토한다.

### 1.6 🟢 [75%] Jetson Xavier NX 부하 프로파일

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-07-03
> 진척: (a) host/container/Jetson 공용 `profile_resources.sh` 추가(CPU%/mem/load/런타임 CSV + summary, tegrastats 자동 감지). (b) `WITH_PROFILE=1` smoke 옵션으로 미션 동안 백그라운드 샘플링. (c) 2026-07-03: `docker/Dockerfile.jetson`(aarch64, GUI 제외) + `docker-compose.jetson.yml` + 배포/검증 절차(`docs/deployment/01_portability_policy.md`) 준비 완료. (d) 남은 갭: buildx 크로스 빌드/GHCR push + Jetson 실기 수치 수집.

다음 조치:

- aarch64 이미지 배포 후 동일 미션을 Jetson에서 돌려 cpu_pct_peak/mem_used_peak를 기록한다.
- sim(host) 대비 실기 포화 여부로 노드 분산/주기 조정을 판단한다.

### 1.7 🟡 [15%] 로봇팔/리프트 통합 (적재 후 질량/COM)

> 상태: 🔄 in-progress · 담당: Han/Kim/Lee · 업데이트: 2026-06-29
> 진척: (a) 적재 footprint는 smoke가 런타임 costmap param으로 모사(확장->하역 시 원복). (b) 택배 질량/COM, 리프트 위치별 COM_z 변화는 URDF에 미모델링. (c) 실제 팔·리프트 제어 미구현 — 계약(footprint 토글, arm-base 상호배제, 리프트 COM 반영) 수준으로만 정리.

영향:

- 적재물(최대 5kg) + 리프트 상승 시 COM_z 상승으로 고속 회전/급정지 전복 마진이 줄어든다.
- footprint만 반영되고 동역학(질량/관성)은 미반영이라 sim 통과가 실기 안정성을 보장하지 않는다.

다음 조치:

- URDF inertial에 적재물/리프트 위치를 반영하고 Gazebo link mass를 갱신한다.
- 팔 동작 중 base 정지(상호배제)를 BT/mission lock으로 보장한다.

### 1.8 🔴 ⏸ [범위 외] 복도 실측 2.0m 재검증

> 상태: ⏸ 보류(실측 대기) · 담당: Lee · 업데이트: 2026-06-29
> 진척: 현재 테스트 복도는 5m(`CORRIDOR_HALF = 2.5`). sim-to-real의 가장 큰 갭이지만 이번 goal에서 의도적으로 제외함. 회귀 가드(`test_smoke_scripts_contract.py`)가 2.5 고정을 assert해 실수 축소를 차단.

다음 조치(실측 후):

- 신공학관 실측 -> `kku_pre_simulation_map.yaml` 갱신 -> world/맵 재생성 -> smoke 재검증.
- 좁은 복도 기준 inflation/lookahead/회전반경 재튜닝.

### 1.9 🔴 ✅ 저장소 거버넌스 — 원격 부재로 로컬 유일본 위험

> 상태: ✅ done · 완료: 2026-07-03 · 담당: Lee
> 조치: 31커밋/2만여 줄이 원격 없이 로컬에만 존재하던 상태를 해소. `PackagU/Code_Space` main(새 공개 히스토리, 일지/대용량/민감정보 제외 190파일)+dev push, CI(`check.yml`) 4/4 green, 이전 이력은 로컬 `archive/pre-github-main`·`lee/l3-cleanup` 브랜치에 보존. push 전 토큰/개인 이메일/대용량 스캔 0건 확인.

### 1.10 🟡 ✅ Fresh-clone 재현성 — 새 환경에서 빌드/테스트 불가

> 상태: ✅ done · 완료: 2026-07-03 · 담당: Lee
> 조치: (a) `robot_arm_pkg` 빈 launch/ install 가드, (b) `scripts/bootstrap_workspace.sh`(worlds+maps 생성, 멱등 확인), (c) smoke `ensure_maps` 자동 부트스트랩, (d) 검증 — GitHub fresh clone에서 bootstrap→오프라인 19/19 PASS→컨테이너 colcon build 4패키지 성공.

### 1.11 🟡 ✅ 맵 기대값 하드코딩 3중화 (stale assert 위험)

> 상태: ✅ done · 완료: 2026-07-03 · 담당: Lee
> 조치: `map_expectations.py`(맵 yaml+pgm 헤더) 단일 소스 신설, verifier `--floor/--from-floor` 인자화, 계약 테스트가 하드코딩 재유입을 차단. stale 문서값(477x299) 4곳 정정. §1.8 실측 반영 시 맵 재생성만으로 기대값 자동 추종.

### 1.12 🟡 ✅ 오케스트레이터 노드명 충돌 (legacy/auto 동시 실행 위험)

> 상태: ✅ done · 완료: 2026-07-03 · 담당: Lee
> 조치: legacy 수동 `floor_orchestrator_pkg`를 `legacy/`(로컬 아카이브)로 이동해 빌드 대상에서 제거 — 동일 노드명 동시 실행 자체가 불가능해짐. 부수 발견: 루트 colcon 빌드가 test_workspace 패키지를 흡수하던 중첩 결함을 `--base-paths src`로 수정(삭제된 패키지의 stale ament index가 Gazebo 기동을 죽이던 문제 — 회귀 가드 추가).

### 1.13 🟢 ✅ F3 층 전환 미검증 (자산만 존재)

> 상태: ✅ done · 완료: 2026-07-03 · 담당: Lee
> 조치: smoke `WITH_F3=1` 옵션 신설 — F2 검증 후 엘베 복귀 → target_floor=F3 param → request_switch → F2→F3 world swap 검증(`kku_f3_building` present/`kku_f2` absent, map ready) → F3 복도 goal SUCCEEDED. 컨테이너 전체 체인(F1→F2→F3) exit 0 확인. 주의: F3 world/맵은 실측 전까지 F2 레이아웃 복제본(§1.8 연계).

### 1.14 🟡 ✅ Smoke footprint 원복 무음 실패 — F2 엘베 갇힘

> 상태: ✅ done · 완료: 2026-07-03 · 담당: Lee
> 조치: 원인 — F2 도착 후 footprint 원복 `ros2 param set ... footprint "[]"` 가 `[]` 를 bool_array 로 파싱해 type error 로 실패했는데 `|| true` 가 마스킹(초기 커밋부터 존재). 확장 footprint(전방 0.40m)가 유지된 채 엘베(문 1.0m)를 나오다 collision ahead → spin 회복 실패 → no valid path 로 갇힘(`WITH_PEDESTRIAN=1 WITH_F3=1` 런에서 발현 — 보행자 7명 set_entity_state 부하로 planner 20Hz→1Hz 저하가 방아쇠). 수정 — `set_costmap_footprint` 헬퍼 신설: 원복을 몸통 polygon 문자열로 교체하고 local/global 모두 "Set parameter successful" 확인, 실패 시 즉시 exit 1(무음 실패 차단). 후속 발견(같은 세션): (a) 보행자 7명×10Hz set_entity_state 부하로 엘베 정차가 벽에 붙어 탈출 불능 → pedestrians.py 5Hz 로 완화, (b) initialpose 유실 시 nav2 activation 데드락 → /amcl_pose 수신 확인 + 재발행, (c) bt_navigator active 前 goal 거부(시작 레이스) → lifecycle active 게이트(initialpose 뒤 배치 필수), (d) 좁은 문 앞 보행자 통과 대기 불가(failure_tolerance 1s) → 45s/움직임 판정 60s + 보행자 런 goal 재시도 2회. 검증 — `WITH_PEDESTRIAN=1 WITH_F3=1` 전체 체인(F1 픽업 7 goal + F1→F2→F3) exit 0, 전 goal SUCCEEDED(재시도 0회 소요), control_loop_missed_rate 20→4.

### 1.15 🔴 실물 주행 불능 위험 — 접지 클리어런스 0 + 전방 지지 부재 (Fusion 반영 미확인)

> 상태: 신규 (HW 제작 착수 전 확인 필수) · 담당: Han(설계 반영)/Lee(검증) · 업데이트: 2026-07-07
> 근거: `delivery_robot.urdf.xacro` 원본(HW팀 Fusion 실측 기반) 주석 — 원본은 섀시 바닥이 바퀴 접지면과 같은 평면(클리어런스 0)이고 후방 캐스터 1개뿐.

영향:

- 클리어런스 0이면 섀시가 바닥에 깔려 바퀴가 헛돌아 **로봇이 아예 주행 불가**. 시뮬은 ★SIM 보정(+0.02m)으로만 돌고 있음.
- 지지 다각형이 구동축(x=0)~후방 캐스터(x=-0.13) 사이인데 COM_x는 로봇 단독 +12mm(축 앞), **적재 5kg 시 +72mm** — 전방 지지 없이는 정적으로도 앞으로 전복.
- 두 결함 모두 2026-06-29에 "HW팀 전달됨"으로만 기록 — **Fusion 설계에 실제 반영됐는지 미확인 상태로 제작 착수는 불가**.

다음 조치:

- Han에게 Fusion 최신본에서 (a) 접지 클리어런스 ≥20mm (b) 전방 캐스터/지지 확인 요청 — 반영 확인 후 이 항목 ✅.
- 배터리(미정, §hardware_spec) 장착 위치를 **구동축 뒤쪽**으로 잡으면 COM_x를 축 뒤로 되돌릴 수 있음 — 배터리 베이 위치를 COM 보정 수단으로 설계에 포함할 것.

### 1.16 🔴 바퀴 66mm·볼캐스터 30mm vs 엘베 문턱/틈새 — 계산상 통과 불가 위험

> 상태: 신규 (부품 발주 전 결정 필요) · 담당: Han/Lee · 업데이트: 2026-07-07
> 근거: 바퀴 r=33mm, 캐스터 볼 r=15mm(URDF/Fusion 실측), 엘베 문턱 높이·틈새 폭은 미실측(hardware_spec §3).

영향:

- 강체 바퀴 단차 등반 필요 견인비 F/W=√(h(2r−h))/(r−h): r=33mm에서 **h=5mm 문턱도 μ≈0.62 필요(고무 한계 근접), h=10mm는 μ≈1.03로 정적 통과 불가능** — 관성 돌파에 의존하게 됨.
- 엘베 승강로 틈새는 통상 20~30mm — **볼 지름 30mm 캐스터는 30mm 틈새에 완전히 빠지고**, 20mm 틈새에서도 3.8mm 낙하 후 탈출해야 함. 엘베 탑승 로봇의 고전적 실패 지점.
- 모터 토크 산정(§hardware_spec 미정 항목)도 문턱 높이에 종속 — 바퀴/캐스터/모터가 한 묶음 결정 사항.

다음 조치:

- **신공학관 엘베 문턱 높이·틈새 폭 실측을 부품 발주보다 먼저** 수행 (맵 실측 방문과 동일 일정에 처리 가능).
- 실측 전 보수적 권고: 구동 바퀴 ≥100mm, 캐스터 ≥50mm(또는 스키드 플레이트 병용) 검토.
- 실측 후 등반 견인비 재계산 → 바퀴 지름 확정 → 모터 토크 산정 순서로 진행 (지름 변경 시 URDF/odom 파라미터 연동 갱신).

### 1.17 🟡 로봇팔 장착 위치 미정의 — 버튼 리치·COM·URDF 통합 공백

> 상태: 신규 · 담당: Kim(팔)/Han(섀시 인터페이스)/Lee(URDF) · 업데이트: 2026-07-07

영향:

- 팔(4DOF, 리치 60cm)이 URDF/hardware_spec 어디에도 장착 위치·질량 없음. 엘베 버튼 높이는 통상 0.9~1.2m — **섀시 상판(z≈0.27m) 장착이면 최대 리치 0.87m로 버튼에 닿지 않을 수 있음**. 마스트 상단(z≈0.72m) 기준이면 여유.
- 장착 위치가 섀시 구조·COM·footprint(수납 시 외곽)를 모두 바꾸므로 **섀시 제작 전에 확정해야 하는 값**.

다음 조치:

- 회의 안건: 팔 장착 위치/높이 + 버튼 실측 높이 확인 → hardware_spec §1 승격.
- 확정 후 URDF에 팔 질량/수납 외곽 반영 (improvement_report §1.7과 연동).

### 1.18 🟡 리프트·LiDAR 사양 불일치 3건 (제작 전 정리)

> 상태: 신규 · 담당: Han/Lee · 업데이트: 2026-07-07

- (a) **리프트 행정 표기 불일치**: hardware_spec "행정 0.5m(URDF)/설계 0.7m" — 마스트 총높이 0.7m에 행정 0.7m은 물리적으로 불가(캐리어 겹침 필요). 0.7m는 마스트 높이, 행정은 0.5m로 해석되나 **T스크류 발주 길이가 갈리므로 제작 전 확정 필요**.
- (b) **리프트 구동력 여유 없음**: URDF `lift_joint` effort=50N < 캐리어 0.5kg+적재 5kg=54N — 시뮬에서도 정격 하중을 못 들어올리는 값. 실물 모터도 54N+마찰 기준으로 산정할 것 (웜기어 자기유지로 정지 유지력은 무관).
- (c) **적재물 상승 시 LiDAR 간섭**: 스캔면 z≈0.77m, 캐리어 최상단 z≈0.62m — **높이 15cm 넘는 적재물을 최고점까지 올리면 스캔면을 가려 SLAM/costmap 오염**. 적재물 높이 상한 규정 또는 "리프트 상승 중 주행 금지" 인터락(§1.7 상호배제와 동일 메커니즘)으로 처리.

### 1.19 🟡 [80%] Code_Space GHCR publish 403 — 패키지가 옛 repo에만 연결

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-08-10
> 진척: (a) 원인 확정 ✅ — GHCR 패키지 `ros2-humble-slam`이 `ros2-humble-slam-docker` repo에만 연결되어 Code_Space 워크플로우 GITHUB_TOKEN push가 403 (repo 이관 후 Code_Space publish-ghcr는 한 번도 성공한 적 없음, run 31289125731에서 최초 발현) / (b) 인터림 우회 ✅ — 옛 publish repo에 jetson 잡 동기화 후 그쪽 CI로 publish (Dockerfile 헤더에 명시된 기존 동기화 절차) / (c) 정식 해결 미착수 — **패키지 설정 Manage Actions access에 `PackagU/Code_Space`(Write) 추가** 후 Code_Space publish-ghcr 재검증, 이후 publish 경로 일원화 결정(회의).

### 1.20 🔴 OpenCR 쇼트 사망 — 오도메트리/구동 계통 블로커

> 상태: 신규 · 담당: Lee(보고)/Han(구동부) · 업데이트: 2026-08-17

- (a) **하드웨어**: OpenCR 1.0 쇼트로 사용 불가 (2026-08-17 Jetson 세팅 중 확인). 재구매 vs 수리 vs 대체 보드(예: OpenCR 재구매, 또는 IMU+모터 드라이버 분리 구성) — 회의 안건.
- (b) **실측 매핑 영향**: 2026-08-09 구축한 OpenCR 브리지→오도메트리 체인 사용 불가. LiDAR 단독(스캔매칭 only) 매핑은 가능하나 품질 저하 가능 — `run_field_mapping.sh` 오도메트리 없는 경로 검증 필요.
- (c) **Jetson compose 영향**: `/dev/opencr` devices 매핑은 장치 부재 시 컨테이너 기동 실패 → OpenCR 복구 전까지 해당 줄 주석 운용 (커밋 금지, 로컬 수정만).

### 1.21 🟡 ✅ Gazebo Classic arm64 바이너리 부재 — Jetson 단독 시뮬 불가

> 상태: ✅ done (분산 구성으로 결정) · 담당: Lee · 완료: 2026-08-17

- (a) **실측 근거**: Ubuntu jammy arm64 저장소에 `gazebo`/`libgazebo-dev` 없음(설치 후보 없음), packages.ros.org arm64 는 `gazebo-dev`/`gazebo-msgs` 만 존재(런타임 래퍼 `gazebo-ros`/`gazebo-plugins` 는 amd64 전용, 그마저 arm64 deb 은 의존 미충족으로 설치 불가), OSRF ubuntu-stable jammy arm64 인덱스(711 패키지)는 Ignition/신형 Gazebo 뿐.
- (b) **결정**: gazebo11 소스 빌드(수 시간·고위험)는 손절. Jetson 포함 시뮬 검증은 **분산 구성** — 데스크톱 Gazebo(`scripts/run_sim_host.sh`) + Jetson 실전 스택(smoke `GAZEBO_REMOTE=1`). 절차: portability policy §4.5. 부수 효과: Jetson 부하 측정에서 Gazebo 오버헤드 제거(더 정확).
- (c) **장기**: 신형 Gazebo(gz-sim)는 arm64 지원 — 시뮬 스택 이관은 별도 대형 과제로 회의 안건 (world/plugin/launch 전면 포팅 필요, 당장 불필요).

아직 없음 (완료 항목은 분기말에 이 절로 이동).
