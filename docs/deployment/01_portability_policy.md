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
- 시뮬 전용 의존(Gazebo)은 Jetson 이미지에 절대 넣지 않는다.

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

## 5. 새 코드 작성 시 체크리스트

- [ ] 파일 경로는 `get_package_share_directory()` 또는 launch 인자로 해석했는가
- [ ] 시리얼/장치 경로가 인자화되어 있는가 (Jetson에서 장치명이 다를 수 있음)
- [ ] GUI 의존(RViz/Gazebo)이 launch 옵션으로 꺼지는가 (`rviz:=false` 등)
- [ ] 새 apt 의존을 추가했다면 `docker/Dockerfile`(개발)과 `docker/Dockerfile.jetson`(실기, GUI 제외) 양쪽에 반영했는가
- [ ] `python3 scripts/check_portability.py` 통과했는가
