# docker/ — PackagU 컨테이너 환경 (SSOT)

프로젝트가 사용하는 모든 Docker 정의의 단일 관리 지점.
ROS2 Humble은 Jetson(JetPack, Ubuntu 20.04)에 직접 설치할 수 없어 **모든 실행은 컨테이너 기준**이다.

## 구성

| 파일 | 용도 | 아키텍처 |
|------|------|----------|
| `Dockerfile` | 개발용 (Gazebo + RViz + Nav2 + SLAM) | amd64 데스크톱 |
| `Dockerfile.jetson` | 실기 배포용 (GUI 제외 런타임) | aarch64 (Xavier NX) |
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
