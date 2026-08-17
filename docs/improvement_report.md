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

### 1.22 🟡 ✅ 시뮬 로봇 무명령 드리프트 — 대기 중 0.3~0.7cm/s 전진 미끄러짐

> 상태: ✅ done (URDF 물리 수정 + 실측 검증) · 담당: Lee · 완료: 2026-08-17

- (a) **실측**: cmd_vel 발행자 0, RTF 0.99 상태에서 0.3~0.7cm/s 지속 활주 (분산 구성과 무관 — 명령 없이 접촉 물리만으로 미끄러짐).
- (b) **영향 (최초 판단 정정)**: "미션 중엔 무해"는 **오판**이었다. goal 사이 정지 구간마다 몸체가 미끄러지는데 **바퀴가 안 구르는 활주라 odom이 정지로 인식** → AMCL 갱신 트리거(odom 이동량) 자체가 없어 belief 동결 → 실위치·belief 간극이 누적 → 스캔의 실제 벽이 belief 좌표계에서 로봇 발밑에 그려짐 → `Starting point in lethal space` + 회복행동 전부 `Collision Ahead` → **미션 ABORT** (F1 pickup에서 2회 재현, 1.2m 간극 실측).
- (c) **수정 (검증 완료)**: `delivery_robot.urdf.xacro` — 바퀴 조인트 `dynamics damping 0.05/friction 0.3` (수동 활주 제동, max_wheel_torque 40 대비 무시 가능) + 바퀴/캐스터 접촉 `maxVel 1.0→0.0` (솔버 보정속도 주입 차단) + 바퀴 `mu 1.0→100` (슬립 방지). **A/B 실측: 30초당 8.6cm → 90초당 2.4mm (약 1/100).**
- (d) **실기 시사점**: 좀비 노드 잔류 속도로 로봇이 계속 주행한 사례도 재현됨 — 시뮬 diff_drive엔 cmd_vel timeout이 없음. 실기 OpenCR 브리지의 0.5s watchdog 정책이 옳았다는 방증, Gazebo 쪽도 diff_drive `cmd_vel_timeout` 설정 검토(잔여 과제). 실물 바퀴/접지 검토는 §1.15에서 Han과 계속.

### 1.23 🟡 ✅ Fast DDS unicast peers 함정 — 같은 호스트 late-participant 상호 발견 불가

> 상태: ✅ done (멀티캐스트 locator 복원) · 담당: Lee · 완료: 2026-08-17

- (a) **증상**: 분산 smoke에서 F1 미션 완주 후 `request_switch` CLI 무한 대기. Nav2 맵은 F2로 전환됐지만(orchestrator↔map_server 매칭 정상) world_swap·로봇팔 노드가 status를 못 받아 월드 교체/팔 시퀀스 미발화.
- (b) **원인**: `scripts/fastdds_lan_peers.xml`의 `initialPeersList`가 **기본 멀티캐스트 announce를 대체**해버림. unicast peer는 참가자 ID 0~3 포트만 탐색하므로 같은 호스트에서 늦게 뜬 참가자끼리(ID>=4: orchestrator↔world_swap/arm/신규 CLI)는 서로 발견할 경로가 없음. 참가자가 적은 데스크톱(낮은 ID)과의 교차 매칭만 성립 — 관측 전부(맵만 전환, 데스크톱 호출 즉시 성공)와 일치.
- (c) **수정**: XML `initialPeersList`에 기본 멀티캐스트 locator `239.255.0.1` 복원(로컬/유선 직결 discovery 담당) + unicast 항목은 Wi-Fi 예비로 유지. smoke의 `ros2 service call` 2곳에 `timeout 45` + 실패 즉시 종료 추가. A/B 실측: 수정 전 Jetson 로컬 echo/call 블록 → 수정 후 즉시 수신.
- (d) **교훈**: `wait_for_service`(daemon 그래프 조회)와 실제 call(신규 DDS participant 직접 discovery)은 경로가 달라 전자가 통과해도 후자가 무한 대기할 수 있음 — 스크립트의 CLI 서비스 호출엔 항상 timeout을 건다.
- (e) **잔존 (멀티캐스트 복원 후에도)**: Jetson 로컬 신규 CLI가 산발적으로 half-hang — param set 이 서버엔 적용되고 응답만 유실된 사례 실측 (F1 미션 중 footprint 확장에서 1회). 대응: smoke 의 모든 CLI 경계(`service call`/`param set`)에 timeout+재시도 적용, nav goal 은 기존 `NAV_GOAL_RETRIES` 사용. 2026-08-17 후속: `set_orchestrator_target_floor` 헬퍼로 F1/F3 `target_floor` param set 에도 timeout 30s ×3 적용(기존 F3 경로는 timeout 부재였음). **데스크톱에서도 재현 확정(반복 런 run_02)**: 서버는 `armed: target=F1` 완료 후 rmw `failed to send response` — Jetson 한정이 아닌 로컬 rmw_fastrtps 일반 현상. 대응: `request_floor_switch` 헬퍼 — 응답 유실 시 orchestrator 로그의 armed 증거로 성공 판정(효과 검증), 증거 없을 때만 재호출. 근본 대책 후보: Fast DDS Discovery Server 로 전환(분산 구성 전반의 discovery 를 단일 서버로 일원화) — 회의 안건.
- (f) **ros2 daemon 오염 (2026-08-17 야간 실측)**: 강제 프로세스 정리 후 daemon 의 rclpy 컨텍스트가 죽은 채(XML-RPC `!rclpy.ok()` fault) 살아남아 `ros2 topic list` 전부 실패 → smoke 가 `/clock` 대기에서 멈춤. 수정: smoke 시작부에 `ros2 daemon stop` 리셋 추가(다음 CLI 호출이 새로 띄움) — 반복 런 강건성 확보.

### 1.24 🟡 ✅ 층 전환 시 로봇 위치 연속성 — tolerance 주차가 F2 출구를 봉쇄

> 상태: ✅ done (스폰 정렬 + costmap 클리어) · 담당: Lee · 완료: 2026-08-17

- (a) **증상**: F2 전환(맵+월드+팔) 성공 직후 f2_corridor 가 `no valid path found` ABORT — 로봇이 엘리베이터에서 못 나옴.
- (b) **원인**: `elevator_inside` goal 이 xy tolerance 한계(실측 0.5m 오프셋, (-0.2,-0.49))로 SUCCEEDED 한 채 전환되면 orchestrator 가 initialpose 를 스폰 좌표 (0,0) 으로 시딩 → belief-실위치 오프셋 상태에서 스캔의 엘베 벽이 belief 좌표계의 문 통로 위에 마킹 → costmap 출구 봉쇄.
- (c) **수정 (완주로 검증)**: smoke 에 `align_robot_to_spawn` — 전환 검증 후 `set_entity_state` 로 로봇을 스폰 (0,0) 에 정렬 + 양쪽 costmap 클리어. 실물 엘리베이터는 물리적 연속이라 없는 문제 — 시뮬 world-swap 전용 부기.
- (d) **후속 후보**: 정렬을 smoke 가 아닌 `gazebo_world_swap_pkg`(world_swap 노드) 책임으로 이동하면 smoke 외 사용처에서도 안전 — 회의 안건.

### 1.25 🟡 ✅ 산발 주행 봉쇄 4종 — 문폭 wedging·전복·belief 드리프트·footprint 간섭

> 상태: ✅ done (원인 4종 전부 참값 스냅샷으로 확정·수정·재검증) · 담당: Lee · 완료: 2026-08-18
> 최종 실증: 데스크톱 왕복 10/10 연속 PASS + 분산(Jetson) 왕복 3/3 완주 — 회복 스택(REALIGN 0.974m 흡수 포함) 실전 검증

- (a) **증상**: 왕복 반복 런 run_05(1/7 빈도) — F2 전환 직후 f2_corridor 3회 시도 전부 실패. nav2 로그에 collision-ahead 약 450초 연속 + spin/backup 회복 전부 봉쇄(사방 lethal). align 텔레포트 success=True·costmap 클리어 정상이라 §1.24(belief 오프셋)와 다른 결함.
- (b) **가설 2개 (사후 로그로 판별 불가)**: ① 보행자 `set_entity_state` call_async 결과 미확인 → 조용한 실패 누적 시 보행자가 통로에 프리즈(450s+ 정지 장애물과 정합) ② 떠난 장애물의 stale lethal 마크 잔존.
- (c) **완화 (적용)**: send_nav_goal 재시도 전 양쪽 costmap 클리어(② 대응) + pedestrians.py에 set_entity_state 실패 누적 감지 ERROR 로그(① 재현 시 즉시 판별).
- (d) **후속**: 반복 런에서 재발 시 pedestrians 로그의 실패 누적 여부로 가설 확정 → ①이면 동기 재시도/큐 제한, ②면 costmap 파라미터(observation persistence) 튜닝.
- (e) **2차 재현 (3차 반복 런 run_08, F1 alcove 출구)**: f1_parcel_pickup 재시도 후 f1_parcel_exit 376s 연속 collision-ahead. 프리즈 감지 로그 침묵 + costmap 클리어 무효 → 유력 가설 재편: **좁은 alcove 재시도/회복기동으로 AMCL belief 오프셋 → 잘못된 좌표계에 스캔 벽이 문 위에 재마킹되는 자기강화 봉쇄**(클리어해도 즉시 재마킹 — 관측 정합). 단, 기존 감지기는 무응답(콜백 미발화) 모드를 못 잡는 맹점 확인.
- (f) **완화 2차 (적용)**: ① 픽업 도킹 재정위 — pickup 성공 직후 알려진 도킹 좌표로 initialpose 재발행(실기 도킹 재정위와 동일 패턴) ② 실패 시 `dump_world_state` 스냅샷(전 모델 참값 pose + AMCL belief) — 다음 재현에서 가설 확정 가능 ③ 프리즈 감지 v2(무응답 timeout 기반).
- (g) **원인 확정 2건 (스냅샷 판독, 2026-08-18)**: ① F1 alcove 출구 — 로봇이 0.9m 문 동측 jamb 에 물리적으로 낌(2회 스냅샷 좌표 소수 4자리 동일). 수정: 택배존 문폭 0.9→1.0m(실측 전 가정치 조정, 엘베 문으로 통과성 검증된 폭) ② 분산 run2 — **로봇 참값 z=0.23m 공중 부양**: 보행자 set_entity_state 텔레포트가 로봇과 겹치는 순간 Gazebo 관통 해소 충격량으로 로봇이 올라탐/전복. 수정: 보행자 근접 일시정지(로봇 belief 0.7m 이내 위치 갱신 보류 — 사람이 로봇을 뚫고 걷지 않게).
- (h) **드리프트 재정위 실증**: 5차 반복 run_01 에서 belief 드리프트 0.716m 실측 → REALIGN 발화 → 런 PASS. 데스크톱 10/10 연속 PASS 달성(repeat_20260817_133709).
- (i) **4번째 원인 + 최종 완결 (2026-08-18)**: 분산 run3 — 택배 확장 footprint 노즈 0.40m 가 1.6m 엘베 포켓의 벽 inflation 과 겹쳐 collision-ahead patience 초과(성공 21회/실패 1회 산발). 노즈 0.35m 로 조정(택배 크기 실측 전 가정치). 이후 분산 3/3 완주 — 마지막 런에서 REALIGN 이 0.974m 드리프트를 흡수하고 재시도 1회로 완주(회복 스택 실전 검증). 잔여 과제: mu2=1.0 방향 분리의 실기 파라미터 무관성 확인(시뮬 전용 값), Fast DDS Discovery Server 전환(§1.23).

아직 없음 (완료 항목은 분기말에 이 절로 이동).
