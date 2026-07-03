# KKU 시뮬레이션 초간단 실행법

작성일: 2026-05-27

이 문서는 매번 VSCode/Docker/Gazebo 절차를 길게 반복하지 않기 위한 빠른 실행법이다. 처음 세팅이 끝난 Linux desktop에서, **완전히 종료된 상태에서 다시 시작하는 흐름**을 기준으로 한다.

## 1. 완전 종료 상태에서 처음 시작

상황:

- PC는 켜져 있다.
- VSCode는 host 프로젝트 `~/2026_graduation_project`로 열었다.
- Docker 컨테이너는 아직 안 켰다고 생각한다.
- Gazebo/RViz2/SLAM도 아직 안 켰다.

### 1.1 Host 터미널에서 Docker 컨테이너 열기

VSCode host 터미널에서 아래만 입력한다.

```bash
cd ~/2026_graduation_project
xhost +local:docker
xhost +local:root
docker compose -f docker_env/compose/docker-compose.linux.yml up -d
```

컨테이너가 켜졌는지 확인한다.

```bash
docker ps
```

`ros2_humble`이 보이면 Docker 준비 완료다.

### 1.2 VSCode에서 컨테이너 attach

VSCode에서:

1. `Ctrl + Shift + P`
2. `Dev Containers: Attach to Running Container...` 선택
3. `ros2_humble` 선택
4. 새 VSCode 창이 뜨면 컨테이너 내부에 들어온 것이다.
5. 컨테이너 VSCode에서 `File -> Open Folder...`
6. `/ros2_ws` 열기

이제 VSCode 터미널을 새로 열면 컨테이너 내부 터미널이다.

### 1.3 컨테이너 내부에서 빌드

컨테이너 VSCode 터미널에서:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg
source install/setup.bash
```

### 1.4 컨테이너 내부에서 시뮬레이션 실행

F1:

```bash
ros2 launch slam_pkg kku_simulation.launch.py floor:=F1
```

F2:

```bash
ros2 launch slam_pkg kku_simulation.launch.py floor:=F2
```

F3:

```bash
ros2 launch slam_pkg kku_simulation.launch.py floor:=F3
```

Gazebo와 RViz2가 뜨면 실행 성공이다.

### 1.5 로봇 조종

컨테이너 VSCode 터미널을 하나 더 열고:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run drive_pkg keyboard_teleop
```

조작:

```text
w  전진
s  후진
a  좌회전
d  우회전
k 또는 Space  정지
q  선속도 증가
z  선속도 감소
e  각속도 증가
c  각속도 감소
```

### 1.6 토픽 확인

컨테이너 VSCode 터미널을 하나 더 열고:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /map --once
```

`/scan`과 `/map`이 나오면 Gazebo LiDAR와 SLAM Toolbox가 연결된 것이다.

로봇이 조종키를 눌러도 안 움직이면 먼저 `/clock`과 `/odom`이 실제로 나오는지 확인한다.

```bash
timeout 3s ros2 topic echo /clock --once
timeout 3s ros2 topic echo /odom --once
```

둘 중 하나라도 timeout이면 teleop 문제가 아니라 Gazebo 서버/physics가 멈춘 상태일 가능성이 높다. 이때는 host 터미널에서 실행 스크립트를 다시 실행한다. 스크립트가 이전 `gzserver`, `gzclient`, RViz2, SLAM 프로세스를 정리한 뒤 새로 띄운다.

```bash
cd ~/2026_graduation_project
./scripts/run_kku_sim.sh F1
```

## 2. 더 빠른 방법: Host에서 스크립트 한 번에 실행

VSCode Attach까지 안 하고 바로 실행만 하고 싶으면 host 터미널에서 스크립트를 쓴다.

터미널 1: 시뮬레이션 실행

```bash
cd ~/2026_graduation_project
./scripts/run_kku_sim.sh F1
```

터미널 2: 로봇 조종

```bash
cd ~/2026_graduation_project
./scripts/teleop.sh
```

터미널 3: 맵 저장

```bash
cd ~/2026_graduation_project
./scripts/save_kku_map.sh F1
```

층만 바꾸면 된다.

```bash
./scripts/run_kku_sim.sh F2
./scripts/save_kku_map.sh F2

./scripts/run_kku_sim.sh F3
./scripts/save_kku_map.sh F3
```

## 3. 각 스크립트가 해주는 일

### `run_kku_sim.sh`

```bash
./scripts/run_kku_sim.sh F1
```

자동으로 처리하는 것:

- X11 GUI 권한 열기: `xhost +local:docker`, `xhost +local:root`
- Docker 컨테이너 실행: `docker compose -f docker_env/compose/docker-compose.linux.yml up -d`
- 이전 실행에서 남은 Gazebo/RViz2/SLAM 프로세스 정리
- ROS2 패키지 빌드: `common_pkg`, `slam_pkg`, `drive_pkg`
- Gazebo + 로봇 + SLAM Toolbox + RViz2 실행

실행 가능한 floor:

```text
F1
F2
F3
```

### `teleop.sh`

```bash
./scripts/teleop.sh
```

컨테이너 안에서 프로젝트 전용 WASD teleop을 실행한다.

조작:

```text
w  전진
s  후진
a  좌회전
d  우회전
k 또는 Space  정지
q  선속도 증가
z  선속도 감소
e  각속도 증가
c  각속도 감소
```

### `save_kku_map.sh`

```bash
./scripts/save_kku_map.sh F1
```

현재 SLAM map을 저장한다.

저장 위치:

```text
컨테이너: /ros2_ws/maps/kku_virtual/f1/kku_f1.yaml
컨테이너: /ros2_ws/maps/kku_virtual/f1/kku_f1.pgm

호스트: ~/2026_graduation_project/src/slam_pkg/maps/kku_virtual/f1/kku_f1.yaml
호스트: ~/2026_graduation_project/src/slam_pkg/maps/kku_virtual/f1/kku_f1.pgm
```

F2, F3는 각각 `f2/kku_f2`, `f3/kku_f3`에 저장된다.

## 4. VSCode에서 스크립트로 쓰는 방법

VSCode를 꼭 attach하지 않아도 스크립트는 호스트 터미널에서 실행된다.

그래도 VSCode 안에서 작업하고 싶으면:

1. VSCode에서 프로젝트 열기

   ```bash
   code ~/2026_graduation_project
   ```

2. VSCode 터미널에서 실행

   ```bash
   ./scripts/run_kku_sim.sh F1
   ```

3. 새 VSCode 터미널을 열고 teleop 실행

   ```bash
   ./scripts/teleop.sh
   ```

컨테이너 내부 파일을 직접 보고 싶을 때만 Dev Containers로 `ros2_humble`에 attach하면 된다.

## 5. 처음 한 번만 필요한 것

아래는 매번 하지 않는다.

- Docker 설치
- Docker 권한 설정
- VSCode 설치
- Dev Containers 확장 설치
- `docker_env` 준비
- GHCR 로그인
- X11 세션 설정

처음 세팅부터 필요한 경우에는 긴 문서를 본다.

- [VSCode + Docker KKU 시뮬레이션 실행 A to Z](../01_environment/03_vscode_docker_kku_simulation_atoz.md)
- [Linux Desktop Docker + ROS2 SLAM Setup Guide](../01_environment/01_linux_docker_slam_setup.md)

## 6. 잘 안 될 때

### GUI 창이 안 뜰 때

```bash
xhost +local:docker
xhost +local:root
docker compose -f docker_env/compose/docker-compose.linux.yml restart
./scripts/run_kku_sim.sh F1
```

### 로봇이 조종해도 안 움직일 때

컨테이너 터미널에서 아래를 확인한다.

```bash
timeout 3s ros2 topic echo /clock --once
timeout 3s ros2 topic echo /odom --once
ros2 topic info /cmd_vel -v
```

- `/clock` 또는 `/odom`이 timeout이면 Gazebo 서버가 멈춘 상태다. host 터미널에서 `./scripts/run_kku_sim.sh F1`을 다시 실행한다.
- `/cmd_vel`의 subscription에 `diff_drive`가 보이지 않으면 로봇이 제대로 spawn되지 않은 것이다. launch 터미널을 `Ctrl+C`로 종료하고 `./scripts/run_kku_sim.sh F1`을 다시 실행한다.
- teleop 터미널에서 키가 안 먹으면 teleop 창을 클릭한 뒤 `w`, `a`, `s`, `d`, `k`를 누른다.

### `ros2_humble is not running`이 나올 때

먼저 시뮬레이션을 실행한다.

```bash
./scripts/run_kku_sim.sh F1
```

### 맵 저장 권한 문제가 날 때

호스트에서 한 번 정리한다.

```bash
cd ~/2026_graduation_project
sudo chown -R "$USER:$USER" src/slam_pkg/maps
```

그 다음 다시 저장한다.

```bash
./scripts/save_kku_map.sh F1
```

## 7. 스크립트 점검

스크립트가 깨졌는지 확인하려면:

```bash
bash scripts/test_kku_sim_scripts.sh
```

성공 출력:

```text
PASS: KKU simulation helper scripts look ready.
```
