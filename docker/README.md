# docker/ — PackagU 컨테이너 환경 (SSOT)

프로젝트가 사용하는 모든 Docker 정의의 단일 관리 지점.
ROS2 Humble은 Jetson(JetPack, Ubuntu 20.04)에 직접 설치할 수 없어 **모든 실행은 컨테이너 기준**이다.

## 구성

| 파일 | 용도 | 아키텍처 |
|------|------|----------|
| `Dockerfile` | 개발용 (Gazebo + RViz + Nav2 + SLAM) | amd64 데스크톱 |
| `Dockerfile.jetson` | 실기 배포용 (RViz2·floor reader 런타임, Gazebo 제외) | aarch64 (Xavier NX) |
| `compose/docker-compose.linux.yml` | Linux 데스크톱 개발 | amd64 |
| `compose/docker-compose.windows.yml` | Windows + VcXsrv 개발 | amd64 |
| `compose/docker-compose.jetson.yml` | Jetson 실기 | aarch64 |
| `scripts/entrypoint.sh` | 공용 진입점 (ROS source + domain id) | 공용 |

## 데스크톱에서 실행 (Linux)

```bash
xhost +local:docker
docker compose -f docker/compose/docker-compose.linux.yml up -d
docker exec -it ros2_humble bash
# 컨테이너 안:
cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
```

또는 원커맨드: `./scripts/run_kku_sim.sh F1`

## Jetson 배포 절차 (요약)

상세 정책과 검증 절차는 [docs/deployment/01_portability_policy.md](../docs/deployment/01_portability_policy.md) 참조.

```bash
# 1) x86 데스크톱에서 aarch64 이미지 크로스 빌드 + push (GHCR 로그인 필요)
docker buildx build --platform linux/arm64 \
  -f docker/Dockerfile.jetson \
  -t ghcr.io/packagu/ros2-humble-slam:humble-jetson --push docker/

# 2) Jetson 에서
git clone https://github.com/PackagU/Code_Space.git && cd Code_Space
python3 scripts/generate_kku_worlds.py && python3 scripts/generate_kku_maps.py
docker compose -f docker/compose/docker-compose.jetson.yml up -d
```

Jetson에서 같은 Dockerfile을 직접 검증할 때는 운영 태그를 덮어쓰지 않고 새 태그를 쓴다.

```bash
docker build --pull=false \
  -f docker/Dockerfile.jetson \
  -t packagu/ros2-humble-slam:humble-jetson-p02 docker/
```

시리얼 장치 매핑은 환경변수로 제어한다 — 기본 `/dev/null` 이라 HW 미장착 상태에서도 기동된다.
장착한 장치만 `docker/compose/.env` 에 실경로를 지정한다 (예: `RPLIDAR_DEVICE=/dev/rplidar`,
`ARM_SERVO_DEVICE=/dev/arm_servo`). 항목별 절차는 [docs/deployment/03_hw_update_checklist.md](../docs/deployment/03_hw_update_checklist.md) 참조.

## 이미지 전략

- 개발: `ghcr.io/packagu/ros2-humble-slam:humble` (amd64)
- 실기: `ghcr.io/packagu/ros2-humble-slam:humble-jetson` (aarch64)
- GHCR publish CI: `.github/workflows/publish-ghcr.yml` — `docker/**` 변경이 main에 push되면
  이 폴더의 Dockerfile 기준으로 amd64 이미지를 빌드/publish 한다.
  구 publish repo(`PackagU/ros2-humble-slam-docker`, 로컬 체크아웃 = `docker_env/`)는
  정리 대상 — GHCR 패키지 연결 repo 전환은 회의에서 확정.
  (첫 실행 시 GHCR 패키지가 구 repo에 연결되어 있으면 org 패키지 설정에서
  Code_Space에 write 권한을 부여해야 한다.)

## 마운트 규약

컨테이너 `/ros2_ws` 아래에 `src`(코드), `maps`(맵), `test_workspace`(PoC/smoke), `scripts`(생성기, 읽기전용)가 마운트된다.
경로 커스터마이즈는 env로: `PACKAGU_SRC`, `PACKAGU_MAPS`, `PACKAGU_TEST_WORKSPACE`, `PACKAGU_SCRIPTS`.
`logs`는 `PACKAGU_LOGS`로 호스트에 영속화한다. entrypoint는 compose가 지정한 필수 읽기/쓰기 마운트가 실제로 없으면 exit 78로 중단한다.

층 인식기는 기본 ROS 서비스와 분리된 선택형 profile이다. P02에서는 코드와 data 영속 경로만 고정하며 카메라 성공을 주장하지 않는다.

```bash
# 카메라 없이 배포 경로/API 실패 상태만 확인
PACKAGU_JETSON_IMAGE=packagu/ros2-humble-slam:humble-jetson-p02 \
FLOOR_READER_SOURCE=disabled \
docker compose -f docker/compose/docker-compose.jetson.yml \
  --profile camera run --rm --no-deps floor_reader
```

실제 카메라 backend와 장치/runtime 전달은 C02 검증 후 `FLOOR_READER_SOURCE`와 compose에 반영한다. 기본 bind 주소는 `127.0.0.1`이며 신뢰된 LAN에서 명시적으로 사용할 때만 `0.0.0.0`으로 바꾼다. 비밀값이 없는 예시는 `compose/jetson.env.example`에 있다.
