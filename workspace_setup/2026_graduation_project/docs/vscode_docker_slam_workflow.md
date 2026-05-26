# VSCode + Docker ROS2 SLAM 작업 가이드

작성일: 2026-05-13

대상 환경:

- Ubuntu 22.04 Linux desktop
- Docker Engine + Docker Compose plugin
- VSCode
- Dev Containers 확장
- `ros2_humble` 컨테이너
- ROS2 Humble workspace: `/ros2_ws`

## 1. 목표

Linux host에 ROS2를 직접 설치하지 않고, VSCode가 Docker 컨테이너에 붙어서 ROS2 작업을 한다.

```text
Linux host
├── ~/2026_graduation_project/
│   ├── src/              # 실제 ROS2 소스 코드
│   └── docker_env/       # Docker 실행환경 repo
│
└── VSCode
    └── ros2_humble 컨테이너에 attach
        └── /ros2_ws/src  # host의 src/와 연결됨
```

VSCode에서 `/ros2_ws/src`를 수정하면 host의 `~/2026_graduation_project/src`가 같이 수정된다.

## 2. VSCode 설치

Ubuntu Software의 Snap 버전보다 Microsoft 공식 apt 버전을 권장한다. Snap/Flatpak 버전은 Docker socket 접근이나 Dev Containers attach에서 문제가 날 수 있다.

```bash
sudo apt-get update
sudo apt-get install -y wget gpg apt-transport-https

wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/packages.microsoft.gpg > /dev/null

echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
  | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

sudo apt-get update
sudo apt-get install -y code
```

실행:

```bash
code
```

## 3. VSCode 확장 설치

VSCode Extensions에서 설치한다.

필수:

- Dev Containers
- Docker

권장:

- ROS
- Python
- CMake Tools

터미널에서 설치하려면:

```bash
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-azuretools.vscode-docker
code --install-extension ms-python.python
code --install-extension ms-vscode.cmake-tools
```

## 4. Docker 권한 확인

host 터미널에서 확인한다.

```bash
docker --version
docker compose version
sudo docker ps
docker ps
```

`sudo docker ps`는 되는데 `docker ps`가 permission denied면 현재 사용자가 Docker 권한을 못 받은 상태다.

해결:

```bash
sudo groupadd docker
sudo usermod -aG docker "$USER"
```

`groupadd`에서 이미 존재한다고 나오면 무시해도 된다.

그 다음 로그아웃 후 다시 로그인한다. VSCode도 완전히 종료하고 다시 켠다.

확인:

```bash
groups
docker ps
```

`groups` 출력에 `docker`가 있어야 한다.

## 5. 컨테이너 실행

host 터미널에서 실행한다.

```bash
cd ~/2026_graduation_project/docker_env
xhost +local:docker
xhost +local:root
docker compose -f compose/docker-compose.linux.yml up -d
docker ps
```

`docker ps`에서 `ros2_humble`이 보이면 준비 완료다.

## 6. VSCode에서 컨테이너 attach

VSCode에서 실행한다.

```text
Ctrl + Shift + P
Dev Containers: Attach to Running Container...
ros2_humble 선택
```

새 VSCode 창이 열리면 그 창은 컨테이너 안에 붙은 상태다.

## 7. 컨테이너 안에서 workspace 열기

컨테이너 VSCode 창에서:

```text
File > Open Folder
```

경로:

```text
/ros2_ws
```

열리면 Explorer에 아래 구조가 보인다.

```text
/ros2_ws
├── src/
├── build/
├── install/
└── log/
```

## 8. VSCode 터미널 사용

컨테이너 VSCode 창에서 터미널을 연다.

```text
Ctrl + `
```

이 터미널은 이미 컨테이너 안이다. 이제 매번 아래 명령을 칠 필요가 없다.

```bash
docker exec -it ros2_humble bash
```

바로 ROS2 명령을 실행한다.

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

패키지 확인:

```bash
ros2 pkg list | grep -E "common_pkg|slam_pkg|drive_pkg|robot_arm_pkg"
```

## 9. Gazebo 실행

컨테이너 VSCode 터미널 1에서 실행한다.

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch common_pkg gazebo.launch.py
```

성공 기준:

- Gazebo 창이 뜬다.
- `walls.world`가 로드된다.
- 로봇 모델이 보인다.

## 10. SLAM Toolbox + RViz2 실행

컨테이너 VSCode 터미널 2에서 실행한다.

```bash
cd /ros2_ws
source install/setup.bash
ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true
```

Gazebo 시뮬레이션에서는 `use_sim_time:=true`를 사용한다.

확인:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

## 11. Teleop 실행

컨테이너 VSCode 터미널 3에서 실행한다.

```bash
cd /ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

기본 조작:

- `i`: 전진
- `,`: 후진
- `j`: 좌회전
- `l`: 우회전
- `k`: 정지

## 12. Map 저장

컨테이너 VSCode 터미널에서 실행한다.

```bash
mkdir -p /ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/floor_test_1
```

host에서 확인:

```bash
ls ~/2026_graduation_project/src/slam_pkg/maps
```

기대 파일:

```text
floor_test_1.yaml
floor_test_1.pgm
```

## 13. 평소 작업 루틴

작업 시작 때 host 터미널에서:

```bash
cd ~/2026_graduation_project/docker_env
xhost +local:docker
xhost +local:root
docker compose -f compose/docker-compose.linux.yml up -d
```

VSCode에서:

```text
Dev Containers: Attach to Running Container...
ros2_humble 선택
/ros2_ws 열기
```

컨테이너 VSCode 터미널에서:

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 14. VSCode 창 사용 기준

VSCode 창을 두 개로 나누면 헷갈리지 않는다.

```text
창 1: host VSCode
경로: ~/2026_graduation_project
용도:
- git pull / git status / git push
- docs 수정
- docker_env 확인
- 전체 repo 관리

창 2: container VSCode
경로: /ros2_ws
용도:
- colcon build
- ros2 launch
- rviz2
- gazebo
- ROS2 package 코드 수정
```

## 15. Attach 실패 시 확인

### Docker 권한 문제

증상:

```text
permission denied while trying to connect to the docker API at unix:///var/run/docker.sock
```

확인:

```bash
groups
docker ps
```

해결:

```bash
sudo usermod -aG docker "$USER"
```

로그아웃 후 다시 로그인한다. VSCode도 종료 후 다시 실행한다.

### 컨테이너가 떠 있지 않음

확인:

```bash
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
```

실행:

```bash
cd ~/2026_graduation_project/docker_env
docker compose -f compose/docker-compose.linux.yml up -d
```

### VSCode Snap/Flatpak 설치 문제

확인:

```bash
which code
snap list | grep code
flatpak list | grep -i code
```

Snap/Flatpak이면 Microsoft 공식 apt 버전으로 다시 설치하는 편이 낫다.

### Dev Containers 로그 확인

VSCode에서:

```text
View > Output
오른쪽 드롭다운: Dev Containers
```

마지막 에러를 확인한다.

## 16. 작업 종료

컨테이너를 계속 켜둬도 된다. 끄려면 host 터미널에서:

```bash
cd ~/2026_graduation_project/docker_env
docker compose -f compose/docker-compose.linux.yml down
```

X11 권한을 닫으려면:

```bash
xhost -local:docker
xhost -local:root
```

## 17. 결론

Linux host에는 Docker만 둔다. ROS2, Gazebo, RViz2, SLAM Toolbox는 컨테이너 안에서 실행한다. VSCode는 `ros2_humble` 컨테이너에 attach해서 `/ros2_ws`를 작업 폴더로 사용한다.
