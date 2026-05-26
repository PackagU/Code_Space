# Linux Desktop Docker + ROS2 SLAM Setup Guide

작성일: 2026-05-13
대상: Ubuntu 22.04 LTS X11 desktop, ROS2 Humble, SLAM Toolbox, Gazebo, RViz2

## 1. 결론

Linux desktop에는 ROS2를 로컬에 직접 설치하지 말고 Docker로 실행한다. 이 프로젝트의 기준은 다음과 같다.

- 호스트 Linux에는 Docker, Git, X11 GUI 권한만 설치한다.
- ROS2 Humble, SLAM Toolbox, Gazebo, RViz2, RPLiDAR 드라이버는 Docker 이미지 안에 둔다.
- 소스 코드는 호스트의 `src/`를 컨테이너의 `/ros2_ws/src`로 mount해서 즉시 반영한다.
- 노트북과 Linux desktop은 하나의 컨테이너를 실시간 공유하지 않는다. 각 PC가 같은 Dockerfile로 같은 환경을 재현한다.
- Docker 실행환경은 `PackagU/ros2-humble-slam-docker` repo와 private GHCR 이미지 `ghcr.io/packagu/ros2-humble-slam:humble`을 기준으로 맞춘다.
- PC 사이 소스 동기화는 Git 또는 승인된 로컬 복사 방식으로 한다.
- `AGENTS.md` 보안 정책상 Notion API 값, token, secret, credential, `.env`, SSH key, auth/session 값, 개인정보 등 민감정보는 외부 서비스로 올리지 않는다. 민감정보가 아닌 코드/문서/이미지는 사용자 허가 또는 팀 정책에 따라 GitHub, DockerHub, Notion 등에 업로드할 수 있다.

Docker를 쓰는 이유는 명확하다. ROS2/Gazebo 계열은 의존성 차이에 민감하다. 팀원이 각자 로컬 Ubuntu에 패키지를 설치하면 버전 차이, 누락 패키지, GUI 설정 차이 때문에 재현성이 깨진다. Dockerfile을 기준으로 잡으면 팀 전체가 같은 ROS 배포판과 패키지 목록을 쓴다.

## 2. 동기화 방식

### 같은 컨테이너를 실시간 공유하는 방식이 아님

컨테이너는 PC마다 따로 뜬다.

```text
노트북              Linux desktop
------              -------------
Docker Engine       Docker Engine
같은 Dockerfile      같은 Dockerfile
같은 repo 내용       같은 repo 내용
별도 컨테이너        별도 컨테이너
```

같은 컨테이너를 네트워크로 붙여서 두 PC가 동시에 쓰는 구조는 추천하지 않는다. GUI, USB 장치, ROS_DOMAIN_ID, 파일 권한, 네트워크가 엉키기 쉽다.

### 소스와 실행 환경을 분리한다

```text
소스 코드 동기화: Git 또는 승인된 로컬 복사
실행 환경 동기화: PackagU/ros2-humble-slam-docker
이미지 공유: GHCR private image
```

현재 권장 방식은 GitHub private repo + GitHub Container Registry(GHCR) private image다. 팀원은 PackagU GitHub 권한을 받은 뒤 각 PC에서 GHCR에 한 번 로그인한다. 브라우저에서 GitHub에 로그인되어 있어도 Docker CLI는 그 로그인 상태를 모르므로 `docker login ghcr.io`가 필요하다.

민감정보가 포함될 가능성이 있거나 계정 설정이 끝나지 않았다면 임시로 로컬 파일로 이미지를 옮길 수 있다.

```bash
# 기존 PC에서 이미지 파일로 저장
docker save ghcr.io/packagu/ros2-humble-slam:humble -o ros2_humble_slam_2026-05-13.tar

# USB/로컬 저장장치로 이동 후 Linux desktop에서 로드
docker load -i ros2_humble_slam_2026-05-13.tar
```

이미지 tar는 크다. 일반적으로는 GHCR에서 pull하는 편이 낫다.

### 팀원 전체가 같은 환경을 써야 하는가

공통 베이스는 같아야 한다.

- ROS 배포판: ROS2 Humble
- 기본 이미지: `osrf/ros:humble-desktop-full`
- 공유 이미지: `ghcr.io/packagu/ros2-humble-slam:humble`
- SLAM/Gazebo/RViz/Nav2/colcon 의존성: `PackagU/ros2-humble-slam-docker`의 Dockerfile에 기록
- 팀원별 패키지 실행: 각자 담당 패키지만 실행해도 됨

SLAM 담당자가 `slam_pkg`만 실행하더라도, 팀 전체는 같은 기본 Dockerfile을 써야 한다. 담당자마다 다른 로컬 설치법을 쓰면 통합 단계에서 문제가 생긴다.

## 3. 오늘 우선순위

오늘 목표는 실제 건물 맵 완성이 아니다. Linux desktop에서 같은 SLAM 실행 환경이 재현되는지 확인하는 것이다.

1. Ubuntu 22.04 X11 상태 확인
2. Docker Engine + Compose plugin 설치
3. Linux용 X11 Docker GUI 설정
4. 컨테이너 pull/up
5. `/ros2_ws` colcon build
6. Gazebo world 실행
7. SLAM Toolbox + RViz2 실행
8. `/scan`, `/odom`, `/tf` 확인
9. 로봇 teleop 이동
10. 테스트 맵 저장

`map swap`은 기본 SLAM이 된 뒤에 한다. 실제 건물 측량은 map swap POC 이후에 시작한다.

## 4. Host 준비

Linux desktop 터미널에서 실행한다.

```bash
lsb_release -a
echo $XDG_SESSION_TYPE
uname -m
```

기준값:

- Ubuntu: 22.04 LTS
- 그래픽 세션: `x11`
- 아키텍처: `x86_64`

`echo $XDG_SESSION_TYPE`이 `wayland`면 X11 세션으로 로그인하는 편이 편하다. 로그인 화면에서 톱니바퀴 메뉴를 눌러 `Ubuntu on Xorg`를 선택한다.

## 5. Docker 설치

공식 Docker apt repository를 사용한다. Ubuntu 기본 저장소의 `docker.io` 패키지는 쓰지 않는다.

### 5.1 충돌 패키지 제거

```bash
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do
  sudo apt-get remove -y "$pkg"
done
```

이미 설치된 패키지가 없다는 메시지는 정상이다.

### 5.2 Docker apt repository 등록

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl

sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt-get update
```

### 5.3 Docker Engine과 Compose plugin 설치

```bash
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

설치 확인:

```bash
sudo systemctl status docker --no-pager
sudo docker run hello-world
docker compose version
```

### 5.4 sudo 없이 docker 쓰기

개인 실험용 PC라면 docker 그룹에 현재 사용자를 추가한다.

```bash
sudo usermod -aG docker "$USER"
newgrp docker
docker run hello-world
```

주의: `docker` 그룹은 root 수준 권한을 준다. 공용 PC에서는 담당자와 상의한다.

### 5.5 GitHub CLI 설치

private GitHub repo와 GHCR private image를 쓰려면 GitHub CLI(`gh`)가 있으면 가장 쉽다.

```bash
sudo apt-get update
sudo apt-get install -y curl ca-certificates

sudo mkdir -p -m 755 /etc/apt/keyrings
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
  | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
  | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null

sudo apt-get update
sudo apt-get install -y gh
gh --version
```

## 6. Repo 준비

이 단계의 목표는 Linux desktop에 두 가지를 준비하는 것이다.

1. 실제 ROS2 workspace: 이 프로젝트 repo 또는 로컬 복사본
2. Docker 실행환경 repo: `PackagU/ros2-humble-slam-docker`

권장 폴더 구조:

```text
~/2026_graduation_project/
├── src/
│   ├── slam_pkg/
│   └── common_pkg/
└── docker_env/          # PackagU/ros2-humble-slam-docker
    ├── Dockerfile
    └── compose/
```

이미 Linux desktop에 프로젝트 workspace를 받아둔 상태라면 그 경로로 이동한다.

```bash
cd ~/2026_graduation_project
```

새 Linux desktop에 처음 가져오는 상황이라면 팀 정책에 맞는 방법을 선택한다.

- GitHub 사용이 허용된 상태: 프로젝트 repo를 `git clone` 또는 `git pull`
- GitHub를 아직 쓰지 않는 상태: USB, 외장 SSD, 로컬 네트워크 복사 등 로컬 방식
- 복사 전 제외 권장: `.env`, API key, SSH key, auth/session 파일, 개인정보가 포함된 로그

GitHub에서 private repo를 받을 경우 먼저 인증한다. 이미 로그인되어 있다면 `gh auth status`로 확인한다.

```bash
gh auth login -h github.com --web --git-protocol https
gh auth refresh -h github.com -s read:packages
gh auth setup-git
```

프로젝트 repo 주소가 확정되어 있으면 아래처럼 받는다.

```bash
cd ~
git clone <PROJECT_REPO_URL> 2026_graduation_project
cd ~/2026_graduation_project
```

Docker 실행환경 repo를 `docker_env` 이름으로 받는다.

```bash
git clone https://github.com/PackagU/ros2-humble-slam-docker.git docker_env
```

이 repo는 private이므로 PackagU 권한이 없는 계정에서는 clone이 실패한다.

GHCR private image pull을 위해 Docker에도 한 번 로그인한다. `<GITHUB_ID>`는 본인 GitHub ID로 바꾼다.

```bash
gh auth token | docker login ghcr.io -u <GITHUB_ID> --password-stdin
```

토큰 값은 화면, 문서, 채팅, Issue에 붙여넣지 않는다.

작업 전 확인:

```bash
git status --short --branch
ls docker_env/compose
ls src/slam_pkg
ls src/common_pkg
```

## 7. Linux용 Docker Compose 전략

현재 Linux desktop 기준 compose는 Docker 전용 repo에 들어 있다.

```text
docker_env/compose/docker-compose.linux.yml
```

Linux에서는 호스트의 X11 socket을 컨테이너에 mount하고 `DISPLAY=$DISPLAY`를 넘긴다. compose 파일은 기본적으로 아래 이미지를 사용한다.

```text
ghcr.io/packagu/ros2-humble-slam:humble
```

기본 mount 기준은 `docker_env/compose/../../src`이므로, `docker_env`를 프로젝트 repo 바로 아래에 clone해야 `src/`가 자동으로 `/ros2_ws/src`에 연결된다.

폴더 구조가 다르면 compose 실행 전에 절대경로를 지정한다.

```bash
export PACKAGU_SRC=/absolute/path/to/2026_graduation_project/src
export PACKAGU_MAPS=/absolute/path/to/2026_graduation_project/src/slam_pkg/maps
```

## 8. X11 GUI 권한 열기

Linux host에서 실행한다.

```bash
xhost +local:docker
```

작업이 끝나면 닫는다.

```bash
xhost -local:docker
```

`xhost +`처럼 전체 허용은 쓰지 않는다.

## 9. 컨테이너 pull/up

Linux host에서 Docker 실행환경 repo 기준으로 실행한다.

```bash
cd ~/2026_graduation_project
cd docker_env
git pull
docker compose -f compose/docker-compose.linux.yml pull
docker compose -f compose/docker-compose.linux.yml up -d
```

상태 확인:

```bash
docker ps
docker logs ros2_humble
```

컨테이너 접속:

```bash
docker exec -it ros2_humble bash
```

컨테이너 안에서 확인:

```bash
echo $ROS_DISTRO
echo $DISPLAY
echo $ROS_DOMAIN_ID
which ros2
```

기대값:

- `ROS_DISTRO`: `humble`
- `DISPLAY`: Linux host의 DISPLAY 값, 보통 `:0` 또는 `:1`
- `ROS_DOMAIN_ID`: `0`

기본 데스크탑 세팅에서는 `--build`를 쓰지 않는다. Dockerfile 의존성을 바꾸는 사람만 Docker repo에 커밋하고, GitHub Actions가 새 GHCR 이미지를 publish한 뒤 팀원들은 `pull`로 받는다.

## 10. GUI smoke test

컨테이너 안에서 실행한다.

```bash
xeyes
```

작은 눈 모양 창이 뜨면 X11 연결이 된다. `xeyes`가 없으면 아래 명령을 쓴다.

```bash
rviz2
```

RViz2 창이 뜨면 성공이다. 창이 뜨지 않으면 18장을 먼저 본다.

## 11. ROS2 workspace build

컨테이너 안에서 실행한다.

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

패키지 확인:

```bash
ros2 pkg list | grep -E "slam_pkg|common_pkg"
```

기대값:

```text
common_pkg
slam_pkg
```

새 터미널을 열면 자동으로 `/opt/ros/humble/setup.bash`와 `/ros2_ws/install/setup.bash`가 source된다. 같은 터미널에서 방금 build한 직후에는 `source install/setup.bash`를 직접 실행한다.

## 12. Gazebo 시뮬레이션 실행

터미널 1, 컨테이너 안:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch common_pkg gazebo.launch.py
```

성공 기준:

- Gazebo 창이 뜬다.
- `walls.world`가 로드된다.
- 로봇 모델 `elevator_robot`이 보인다.

## 13. SLAM Toolbox + RViz2 실행

터미널 2, 컨테이너 안:

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true
```

중요: Gazebo 시뮬레이션에서는 `use_sim_time:=true`를 반드시 쓴다.

성공 기준:

- RViz2 창이 뜬다.
- `slam_toolbox` 노드가 실행된다.
- `/scan` topic이 들어온다.

확인 명령:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

## 14. 로봇 이동

터미널 3, 컨테이너 안:

```bash
cd /ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

키:

- `i`: 전진
- `,`: 후진
- `j`: 좌회전
- `l`: 우회전
- `k`: 정지

SLAM 맵을 만들려면 같은 자리에서 회전만 하지 말고 벽과 기둥이 보이도록 천천히 이동한다.

## 15. 맵 저장

터미널 4, 컨테이너 안:

```bash
mkdir -p /ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/floor_test_1
```

호스트에서 확인:

```bash
ls src/slam_pkg/maps
```

기대 파일:

```text
floor_test_1.yaml
floor_test_1.pgm
```

이 파일이 생기면 M1 SLAM 기본 DOD에 가까워진다.

## 16. 오늘의 완료 기준

오늘은 아래 항목이 되면 성공이다.

- Docker Engine 설치 완료
- `docker compose version` 확인
- Linux용 compose로 `ros2_humble` 컨테이너 실행
- 컨테이너 안에서 RViz2 또는 Gazebo GUI 표시
- `colcon build --symlink-install` 성공
- `/scan`, `/odom`, `/tf` 확인
- teleop으로 로봇 이동
- `floor_test_1.yaml`, `floor_test_1.pgm` 저장

실패해도 어디까지 됐는지 기록하면 된다. Docker 설치, GUI, build, SLAM 중 어느 단계에서 막혔는지를 분리해야 다음 디버깅이 빠르다.

## 17. Map swap과 실제 측량 순서

### 지금 하지 않을 것

실제 신공학관 치수를 먼저 재서 Gazebo world를 만드는 작업은 뒤로 둔다. 환경 검증 전에 실측 world를 만들면 문제가 생겼을 때 원인이 Docker, Gazebo, SLAM, world 모델 중 어디인지 갈라내기 어렵다.

### 다음 순서

1. 단순 `walls.world`로 SLAM 맵 저장
2. 단순 테스트 맵 2개 생성: `floor_test_1`, `floor_test_2`
3. Nav2 `map_server`로 저장된 map load 확인
4. `/elevator_arrived` mock 신호를 기준으로 map 교체 POC
5. POC 성공 후 신공학관 데모 동선 측량
6. `building_v1.world`, `building_v2.world` 작성
7. 층별 SLAM 맵 저장: `floor1.yaml`, `floor2.yaml`
8. localization 재시작까지 확인

M1의 핵심은 SLAM 기본 동작이다. map swap은 M2 성격이 강하므로 오늘은 설계와 작은 POC까지만 잡는다.

## 18. Troubleshooting

### Docker 권한 오류

증상:

```text
permission denied while trying to connect to the Docker daemon socket
```

해결:

```bash
sudo usermod -aG docker "$USER"
newgrp docker
docker ps
```

그래도 안 되면 로그아웃 후 다시 로그인한다.

### RViz2/Gazebo 창이 안 뜸

확인:

```bash
echo $DISPLAY
xhost
docker exec -it ros2_humble bash
echo $DISPLAY
```

해결:

```bash
xhost +local:docker
cd ~/2026_graduation_project/docker_env
docker compose -f compose/docker-compose.linux.yml down
docker compose -f compose/docker-compose.linux.yml up -d
```

Wayland 세션이면 X11 세션으로 다시 로그인한다.

### GHCR pull이 unauthorized로 실패함

증상:

```text
unauthorized
denied
pull access denied
```

원인 후보:

- PackagU 조직 또는 `PackagU/ros2-humble-slam-docker` repo 권한이 없음
- GHCR login을 하지 않음
- `gh auth refresh -s read:packages`를 하지 않음
- Docker가 다른 GitHub 계정으로 로그인되어 있음

해결:

```bash
gh auth status
gh auth refresh -h github.com -s read:packages
gh auth token | docker login ghcr.io -u <GITHUB_ID> --password-stdin
docker pull ghcr.io/packagu/ros2-humble-slam:humble
```

그래도 실패하면 PackagU repo 권한과 GHCR package 권한을 확인한다.

### `Package 'common_pkg' not found`

해결:

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch common_pkg gazebo.launch.py
```

### `/scan`이 없음

확인:

```bash
ros2 topic list | grep scan
ros2 node list
```

원인 후보:

- Gazebo가 실행되지 않음
- `common_pkg` launch가 실패함
- LiDAR Gazebo plugin이 로드되지 않음
- workspace build 후 source를 하지 않음

먼저 Gazebo 터미널 로그에서 plugin 오류를 확인한다.

### SLAM 맵이 안 늘어남

확인:

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
```

원인 후보:

- `use_sim_time:=true` 누락
- `/scan` frame과 TF 연결 문제
- 로봇을 거의 움직이지 않음
- RViz fixed frame 설정 문제

Gazebo를 쓰는 동안은 `ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true`를 쓴다.

### 맵 저장 파일이 안 보임

컨테이너 안:

```bash
ls -la /ros2_ws/maps
```

호스트:

```bash
ls src/slam_pkg/maps
```

`docker_env/compose/docker-compose.linux.yml`의 volume에 아래 줄이 있어야 한다.

```yaml
- ${PACKAGU_MAPS:-../../src/slam_pkg/maps}:/ros2_ws/maps:rw
```

## 19. 세션 종료 기록

SLAM 세션을 마치면 로컬 로그를 남긴다.

```bash
python scripts/log_session.py \
  --summary "Linux desktop Docker에서 Gazebo/RViz2/SLAM Toolbox 실행 확인" \
  --result partial \
  --next "/scan, /odom, /tf와 map_saver까지 재검증"
```

실제 성공했다면 `--result success`로 기록한다.

`TODO.md`에는 다음 정보를 남긴다.

- Docker 설치 여부
- GUI 성공 여부
- colcon build 성공 여부
- `/scan` 확인 여부
- 맵 저장 여부
- 다음 막힌 지점

## 20. 최종 추천 운영 방식

Linux desktop 작업 방식:

```text
1. host에는 Docker만 설치
2. repo를 받음
3. Linux용 compose로 컨테이너 실행
4. 컨테이너 안에서 colcon build
5. Gazebo + SLAM + RViz2 실행
6. 맵 저장
7. 시행착오 로그 작성
```

팀 운영 방식:

```text
1. 의존성 추가는 Dockerfile에만 기록
2. 개인 PC에 직접 apt install한 ROS 패키지는 공식 환경으로 인정하지 않음
3. Docker 환경 변경은 PackagU/ros2-humble-slam-docker에 커밋
4. GHCR publish 전에는 이미지와 compose 설정에 민감정보가 들어가지 않았는지 확인
5. 팀원은 GitHub 권한 + GHCR login 후 같은 이미지를 pull
6. 소스 sync는 Git 정책이 정리된 뒤 시작
7. SLAM map swap은 기본 SLAM 저장 성공 후 진행
```

## 21. 참고 링크

- Docker Engine Ubuntu 설치: https://docs.docker.com/engine/install/ubuntu/
- Docker Linux post-install: https://docs.docker.com/engine/install/linux-postinstall/
- Docker Compose plugin 설치: https://docs.docker.com/compose/install/linux/
- GitHub CLI Linux 설치: https://github.com/cli/cli/blob/trunk/docs/install_linux.md
- GitHub Container Registry: https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry
