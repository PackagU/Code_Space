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

### 1.6 🟢 [60%] Jetson Xavier NX 부하 프로파일

> 상태: 🔄 in-progress · 담당: Lee · 업데이트: 2026-06-29
> 진척: (a) host/container/Jetson 공용 `profile_resources.sh` 추가(CPU%/mem/load/런타임 CSV + summary, tegrastats 자동 감지). (b) `WITH_PROFILE=1` smoke 옵션으로 미션 동안 백그라운드 샘플링. (c) 남은 갭: Jetson 실기 수치 수집 — aarch64 배포 컨테이너 필요.

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

## Completed Improvements

아직 없음.
