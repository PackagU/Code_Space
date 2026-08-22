# Portability Policy and Jetson Deployment

> 목적: 지금 데스크톱 시뮬에서 도는 코드를 **수정 없이** Jetson Xavier NX 컨테이너에 올리기 위한 규칙과 절차.
> 검사기: `python3 scripts/check_portability.py` (CI에서도 실행). AGENTS.md 필수 규칙 13이 이 문서를 가리킨다.

## 1. 왜 필요한가

- Jetson(JetPack)은 Ubuntu 20.04 기반이라 ROS2 Humble을 직접 설치할 수 없다 → 모든 실행은 컨테이너.
- 데스크톱은 amd64, Jetson은 aarch64 → 이미지가 다르다. 코드가 컨테이너 내부 레이아웃이나 호스트 경로를 가정하면 이전할 때마다 깨진다.
- 2026-06-30 부하 검증에서 확인했듯 실기 병목은 compute 부하다. 배포 전 이식성 문제로 시간을 잃으면 부하 튜닝 시간이 사라진다.

## 2. 코드 이식성 규칙 (R1~R4)

| 규칙 | 내용 | 적용 범위 |
|------|------|----------|
| R1 | `/home/...` 사용자 홈 절대경로 금지 | `src/`, `scripts/`, `test_workspace/` 전체 |
| R2 | `/ros2_ws` 등 컨테이너 절대경로의 기능적 사용 금지 (주석은 허용) | `src/` (배포 대상 패키지) |
| R3 | `/dev/tty*` 시리얼 포트는 launch 인자(`DeclareLaunchArgument`)로만 | `src/` launch 파일 |
| R4 | `map_file_name` 등 yaml 내 절대경로 금지 — launch 파라미터로 override | `src/` yaml |

계층별 성격:

- `src/` = 배포 대상. 경로는 `ament_index`(share 디렉토리) 또는 launch 인자로만 해석한다.
- `test_workspace/` = 컨테이너 검증 도구. `${ROOT:-/ros2_ws}` env-override 패턴은 허용.
- `scripts/` = 호스트 래퍼. `docker exec` 안의 컨테이너 경로는 허용, 호스트 경로는 스크립트 위치 기준 상대 경로로.

## 3. 이미지 전략

| 태그 | 용도 | 베이스 | 포함 |
|------|------|--------|------|
| `ghcr.io/packagu/ros2-humble-slam:humble` | 개발 (amd64) | `osrf/ros:humble-desktop-full` | Gazebo, RViz, Nav2, SLAM Toolbox |
| `ghcr.io/packagu/ros2-humble-slam:humble-jetson` | 실기 (aarch64) | `ros:humble-ros-base` | Nav2, SLAM Toolbox, rplidar (GUI 제외) |

원칙:

- 정의는 `docker/`가 SSOT. 두 Dockerfile의 공통 패키지 목록이 어긋나지 않게 함께 수정한다.
- SLAM/Nav2는 CPU 스택이므로 CUDA/L4T 베이스가 필요 없다. GPU가 필요해지는 시점(예: 카메라 추론)에 별도 태그로 분리한다.
- 시뮬 전용 의존(Gazebo)은 Jetson 이미지에 절대 넣지 않는다. **Gazebo Classic 은 arm64 공식
  바이너리 자체가 없어** (improvement_report §1.21) Jetson 위 시뮬은 불가능하며, Jetson 을
  포함한 시뮬 검증은 분산 구성(§4.5: 데스크톱 Gazebo + Jetson 실전 스택)으로 수행한다.
  예외: `ros-humble-gazebo-msgs` 는 메시지 정의만이라 (Gazebo 런타임 아님) Jetson 이미지에
  포함한다 — 분산 검증 스크립트가 원격 Gazebo 서비스 호출에 사용 (없으면 verify 단계 즉사 실측).

## 4. Jetson 배포 절차 (원커맨드 지향)

### 4.1 데스크톱에서 이미지 준비 (1회)

```bash
docker buildx create --use 2>/dev/null || true
docker buildx build --platform linux/arm64 \
  -f docker/Dockerfile.jetson \
  -t ghcr.io/packagu/ros2-humble-slam:humble-jetson \
  --push docker/
```

### 4.2 Jetson에서 기동

```bash
git clone https://github.com/PackagU/Code_Space.git && cd Code_Space
python3 scripts/generate_kku_worlds.py && python3 scripts/generate_kku_maps.py  # 맵/월드 생성(.pgm은 git 미포함)
docker login ghcr.io   # GitHub PAT (read:packages)
docker compose -f docker/compose/docker-compose.jetson.yml up -d
docker exec -it ros2_humble bash
# 컨테이너 안:
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### 4.3 부하 검증 (통과 기준 포함)

시뮬 없이 headless로 Nav2 스택을 띄우고 리소스를 샘플링한다:

```bash
# 컨테이너 안 — 프로파일러는 test_workspace 마운트에 있음
bash /ros2_ws/test_workspace/gazebo_world_swap/scripts/profile_resources.sh --duration 120 --out /tmp/jetson_profile &
ros2 launch slam_pkg kku_navigation.launch.py floor:=F1 use_sim_time:=false rviz:=false
```

통과 기준(2026-06-30 부하 강건성 세션에서 정의, 상세 로그는 Notion 세션 일지):

- `cpu_pct_peak` 포화(600%/6코어 기준) 미만에서 control loop `missed its desired rate` 없음
- `WITH_STRESS` 대응 시나리오: 부하 주입 상태에서도 RT 우선순위(`chrt -r 10`, SCHED_RR) 적용 시 미션 완주
- RT 영구 적용은 systemd unit 또는 컨테이너 기동 스크립트에서 `chrt`/`renice` 수행

### 4.4 사전 방지 게이트

배포 전 데스크톱에서 아래가 전부 통과해야 한다:

```bash
python3 scripts/check_portability.py
python3 test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py
bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh   # 컨테이너
```

CI(`.github/workflows/check.yml`)는 이 중 오프라인 검사를 PR마다 자동 실행한다.

### 4.5 분산 시뮬 부하테스트 — 데스크톱 Gazebo + Jetson 실전 스택

Gazebo Classic(11)은 arm64 공식 바이너리가 없어(§1.21) Jetson 위에서 시뮬을 직접 돌릴 수 없다.
대신 같은 LAN 에서 역할을 나눈다 — 이 구성이 부하 측정도 더 정확하다(Jetson 에 실전 부하만 실림):

- **데스크톱**: Gazebo 물리/센서 (기존 amd64 dev 컨테이너) — 실전에서 실센서가 대체할 부분
- **Jetson**: Nav2 + 층전환 오케스트레이터 + 미션 + 로봇팔 (프로덕션 `humble-jetson` 이미지 그대로)

전제: 두 머신 같은 서브넷, `ROS_DOMAIN_ID` 동일(기본 0), 두 compose 모두 `network_mode: host` (기본값).

**원커맨드 (2026-08-21 신설, 권장)**: 데스크톱에서 `bash scripts/run_distributed_e2e.sh` 한 줄이 아래 [0]~[3] 을
자동으로 수행한다 — Jetson 잔존 노드 정리 → 데스크톱 Gazebo 리셋·기동 → `Successfully spawned entity` 로그 확인(사람 눈 의존 제거) →
Jetson smoke(nohup) 기동 → 종료까지 폴링 → Jetson 로그·`run_<ts>/` 아티팩트를 `verification/dist_<ts>/` 로 수집.
옵션은 `SMOKE_ENV`(기본: 왕복+보행자+정적 장애물+리프트+팔+프로파일, retries=2, MAX_MISSED_RATE=30), `GAZEBO_GUI`, `JETSON_SSH`.
실측 2026-08-21 3/3 PASS: 각 11 goal 1차 SUCCEEDED·재시도 0, Jetson cpu avg 48%/peak 89~94%(6코어 시스템 전체), mem 2.5GB/6.8GB,
missed 23~29(데스크톱 0~3 대비 높음 → 원커맨드 기본 `MAX_MISSED_RATE=60`, 실기 임계는 실센서 주행에서 수립).

수동 절차(원커맨드가 안 될 때). 실행 순서는 반드시 지킨다 (2026-08-17 완주 실증 절차 — 순서 위반이 실패 2회를 유발했다):

```bash
# [0] Jetson 정리 — 이전 런의 노드/스크립트 잔류 제거
docker restart ros2_humble

# [1] 데스크톱 Gazebo 리셋 + 기동 — 매 런 전 필수 (월드 상태 오염 방지).
#     "Successfully spawned entity [elevator_robot_f1]" 확인 후 다음 단계로.
docker restart ros2_humble
GAZEBO_GUI=true bash scripts/run_sim_host.sh F1

# [2] Jetson smoke 원커맨드 (호스트에서 실행 — nohup 이라 SSH 끊겨도 계속 돈다)
#     왕복 배달 체인(현행 표준): WITH_RETURN=1. 실물 팔 장착 시 ARM_SERIAL_PORT=/dev/arm_servo 추가.
docker exec ros2_humble bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/scripts/fastdds_lan_peers.xml && ros2 daemon stop >/dev/null 2>&1; cd /ros2_ws && GAZEBO_REMOTE=1 WITH_RETURN=1 WITH_PEDESTRIAN=1 WITH_ARM=1 WITH_PROFILE=1 NAV_GOAL_RETRIES=2 nohup bash test_workspace/gazebo_world_swap/scripts/run_l3_world_swap_smoke.sh > /ros2_ws/logs/smoke_run.log 2>&1 & sleep 3; tail -3 /ros2_ws/logs/smoke_run.log"

# [3] 진행 관찰 (Jetson)
docker exec ros2_humble tail -f /ros2_ws/logs/smoke_run.log
```

`NAV_GOAL_RETRIES=2` 는 분산+보행자 구성 권장값 — 데스크톱 10회 연속 검증과 동일 조건
(2026-08-18 확정: retries=1 은 보행자 조우 2연속이면 즉사). 재시도 시 스모크가
costmap 클리어 + belief 드리프트 재정위(>0.3m, 시뮬 오라클)를 자동 수행한다.
데스크톱 단독 결정적 회귀 검증에서는 기본 0 유지.

성공 판정 (2026-08-18 왕복 체인 실측 기대값):

- F1 7 goal SUCCEEDED → `ARM PASS: F2` → f2_corridor SUCCEEDED → f2_elevator_inside →
  F2→F1 역전환 `PASS world swap smoke` → `ARM PASS: F1`(완료 누계 2) → f1_charge_station SUCCEEDED
- 아티팩트: `verification/run_<ts>/` — `profile/resource_summary.txt`(기준: cpu_pct_peak < 600%),
  `control_metrics.txt`(missed/TF 외삽 횟수 기록 — 임계는 실기 기준 수립 후 `MAX_MISSED_RATE` 로 게이트)
- 기준 실측치(왕복): cpu_peak 67~78% / mem 2.4~2.5GB (여유 큼)
- 데스크톱 단독 반복 기준: `REPEAT_N=10 bash .../run_roundtrip_repeat.sh` — 10/10 연속 PASS
  (2026-08-17 실증, repeat_summary.md 자동 집계)

주의사항 (실측으로 확정):

- **Jetson 이미지에 `ros-humble-gazebo-msgs` 필요** — Dockerfile.jetson 에 반영됨. 이미지 재빌드
  전의 기존 컨테이너에는 `apt-get update && apt-get install -y ros-humble-gazebo-msgs` 1회 주입
  (docker restart 에는 유지되고, 컨테이너 recreate 시에만 재설치).
- 두 머신 모두 컨테이너 이름이 `ros2_humble` — 반드시 자기 머신 터미널에서 실행할 것.
  (참고: Jetson hostname 이 `hsm-desktop` 로 되어 있어 프롬프트로 머신을 구분하면 안 된다.)
- Jetson 저장소는 bundle 기반 — 데스크톱에서 bundle 갱신 후
  `scp ~/code_space.bundle hsm:/home/hsm/code_space.bundle`, Jetson 에서 로컬 수정 파일
  `git checkout -- <파일>` 후 `git pull origin <브랜치>`, compose 장치 sed 재적용.

**DDS discovery (실측 확정)**: 이 Wi-Fi 환경은 멀티캐스트 discovery 가 막혀 있어
`scripts/fastdds_lan_peers.xml` (unicast peers, 양쪽 IP 명시) 를 **양쪽 모두** 적용해야 한다.
XML 의 `initialPeersList` 에는 기본 멀티캐스트 locator `239.255.0.1` 이 반드시 포함되어야 한다 —
빠지면 같은 호스트에서 늦게 뜬 노드끼리 상호 발견이 안 된다 (improvement_report §1.23).
데스크톱은 `run_sim_host.sh` 가 자동 적용하고, Jetson 은 smoke 실행 전 같은 셸에서:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/scripts/fastdds_lan_peers.xml
ros2 daemon stop   # 이전 설정으로 캐시된 daemon 제거
```

IP 가 바뀌면 XML 의 두 항목을 갱신한다. `/clock` 만 보이고 `/scan` 이 늦게 오는 것은
엔드포인트 교환 지연 — 수 초 뒤 재확인하면 된다.

## 5. 새 코드 작성 시 체크리스트

- [ ] 파일 경로는 `get_package_share_directory()` 또는 launch 인자로 해석했는가
- [ ] 시리얼/장치 경로가 인자화되어 있는가 (Jetson에서 장치명이 다를 수 있음)
- [ ] GUI 의존(RViz/Gazebo)이 launch 옵션으로 꺼지는가 (`rviz:=false` 등)
- [ ] 새 apt 의존을 추가했다면 `docker/Dockerfile`(개발)과 `docker/Dockerfile.jetson`(실기, GUI 제외) 양쪽에 반영했는가
- [ ] `python3 scripts/check_portability.py` 통과했는가
