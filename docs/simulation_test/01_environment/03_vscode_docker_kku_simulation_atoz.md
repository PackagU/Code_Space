# VSCode + Docker KKU 시뮬레이션 실행 A to Z

작성일: 2026-05-27

대상 환경:

- Linux desktop
- VSCode
- Dev Containers 확장
- Docker 컨테이너: `ros2_humble`
- ROS2 workspace: `/ros2_ws`
- 실행 대상: KKU pre-simulation world + Gazebo + SLAM Toolbox + RViz2

## 1. 목표

Linux desktop의 VSCode에서 Docker 컨테이너 `ros2_humble`에 attach한 뒤, 아래 명령으로 층별 시뮬레이션을 실행한다.

```bash
ros2 launch slam_pkg kku_simulation.launch.py floor:=F1
```

층은 `F1`, `F2`, `F3` 중 하나를 고른다. 이 launch는 Gazebo world, 로봇 spawn, SLAM Toolbox, RViz2를 한 번에 올린다.

## 2. 전체 흐름

```text
Linux desktop
  1. Docker/VScode 준비 확인
  2. X11 GUI 권한 허용
  3. docker compose로 ros2_humble 컨테이너 실행
  4. VSCode에서 실행 중인 컨테이너에 attach
  5. /ros2_ws 폴더 열기
  6. colcon build
  7. KKU 시뮬레이션 launch
  8. teleop으로 주행
  9. /scan, /odom, /map 확인
 10. 층별 맵 저장
```

## 3. 사전 확인

호스트 Linux 터미널에서 확인한다.

```bash
cd ~/2026_graduation_project
docker ps
docker compose version
echo "$XDG_SESSION_TYPE"
echo "$DISPLAY"
```

기준:

- `docker ps`가 `permission denied` 없이 실행되어야 한다.
- `echo "$XDG_SESSION_TYPE"`은 `x11` 권장.
- `echo "$DISPLAY"`는 보통 `:0` 또는 `:1`이다.

`docker ps`가 권한 오류를 내면 한 번 로그아웃/로그인 후 다시 확인한다. 그래도 안 되면 Docker 그룹 설정을 다시 확인한다.

```bash
groups
sudo usermod -aG docker "$USER"
```

## 4. X11 GUI 권한 열기

Gazebo와 RViz2는 GUI 창을 띄우므로 호스트에서 X11 권한을 열어야 한다.

```bash
xhost +local:docker
xhost +local:root
```

확인:

```bash
xhost
```

출력에 `LOCAL:` 관련 항목이 보이면 된다.

만약 Wayland 세션이면 Gazebo/RViz GUI가 까다로울 수 있다. 로그인 화면에서 톱니바퀴 메뉴를 눌러 `Ubuntu on Xorg`로 로그인하는 편이 안정적이다.

## 5. 컨테이너 실행

호스트 Linux 터미널에서 실행한다.

```bash
cd ~/2026_graduation_project
docker compose -f docker_env/compose/docker-compose.linux.yml up -d
docker ps
```

성공 기준:

```text
ros2_humble   Up ...
```

컨테이너 로그에서 DISPLAY도 확인할 수 있다.

```bash
docker logs --tail 20 ros2_humble
```

`[entrypoint] DISPLAY: :0` 또는 비슷한 값이 보이면 정상이다.

## 6. VSCode에서 컨테이너 attach

VSCode를 연다.

```bash
code ~/2026_graduation_project
```

VSCode에서 실행한다.

```text
Ctrl + Shift + P
Dev Containers: Attach to Running Container...
ros2_humble 선택
```

새 VSCode 창이 뜨면 그 창은 컨테이너 내부에 붙은 상태다.

## 7. `/ros2_ws` 열기

컨테이너에 attach된 새 VSCode 창에서:

```text
File > Open Folder
```

경로:

```text
/ros2_ws
```

Explorer에 아래 구조가 보이면 맞다.

```text
/ros2_ws
├── src/
├── build/
├── install/
└── log/
```

현재 compose 설정은 호스트의 `src/`를 컨테이너의 `/ros2_ws/src`에 연결한다. 따라서 컨테이너 VSCode에서 `/ros2_ws/src`를 편집하면 호스트의 `~/2026_graduation_project/src`도 같이 바뀐다.

## 8. VSCode 터미널 열기

컨테이너 VSCode 창에서 터미널을 연다.

```text
Ctrl + `
```

이 터미널은 이미 컨테이너 안이다. 그래도 새 터미널마다 아래처럼 ROS setup을 명시적으로 source하는 습관을 권장한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
```

`install/setup.bash`는 빌드 후에 source한다.

## 9. 패키지 빌드

컨테이너 VSCode 터미널에서 실행한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg
source install/setup.bash
```

성공 기준:

```text
Finished <<< common_pkg
Finished <<< slam_pkg
Summary: 2 packages finished
```

패키지 확인:

```bash
ros2 pkg list | grep -E "common_pkg|slam_pkg"
```

중요: `docker exec ros2_humble colcon build ...`처럼 bash 없이 바로 실행하면 `/opt/ros/humble/setup.bash`가 source되지 않아 `ament_package`를 못 찾는 경우가 있다. 그럴 때는 아래처럼 실행한다.

```bash
docker exec -w /ros2_ws ros2_humble bash -lc \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg'
```

VSCode attach 터미널에서는 보통 직접 `source /opt/ros/humble/setup.bash` 후 빌드하면 된다.

## 10. KKU 시뮬레이션 실행

컨테이너 VSCode 터미널 1에서 실행한다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_pkg kku_simulation.launch.py floor:=F1
```

다른 층:

```bash
ros2 launch slam_pkg kku_simulation.launch.py floor:=F2
ros2 launch slam_pkg kku_simulation.launch.py floor:=F3
```

성공 기준:

- Gazebo 창이 뜬다.
- RViz2 창이 뜬다.
- Gazebo에 `elevator_robot_f1`, `elevator_robot_f2`, 또는 `elevator_robot_f3`가 spawn된다.
- RViz2에서 `LaserScan`, `TF`, `Map`, `RobotModel`이 보인다.
- 터미널에 `Spawn status: SpawnEntity: Successfully spawned entity`가 보인다.
- SLAM Toolbox가 `Registering sensor` 메시지를 출력한다.

## 11. GUI가 안 뜰 때 빠른 점검

증상:

```text
Authorization required, but no authorization protocol specified
qt.qpa.xcb: could not connect to display
```

호스트 터미널에서 다시 실행한다.

```bash
xhost +local:docker
xhost +local:root
docker compose -f docker_env/compose/docker-compose.linux.yml restart
```

컨테이너 안에서 DISPLAY 확인:

```bash
echo "$DISPLAY"
```

호스트의 DISPLAY와 컨테이너의 DISPLAY가 같은지 확인한다.

```bash
# 호스트
echo "$DISPLAY"

# 컨테이너 VSCode 터미널
echo "$DISPLAY"
```

테스트용 GUI:

```bash
xeyes
```

`xeyes` 창이 뜨면 X11 연결은 정상이다. `xeyes`가 없다면 컨테이너 이미지에 `x11-apps`가 포함되어 있는지 확인한다.

## 12. 토픽 확인

시뮬레이션 launch를 켜둔 상태에서 컨테이너 VSCode 터미널 2를 연다.

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 node list
ros2 topic list
```

핵심 토픽:

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /map --once
```

`/scan`은 Gazebo LiDAR, `/odom`은 diff drive, `/map`은 SLAM Toolbox 결과다.

TF 확인:

```bash
ros2 run tf2_tools view_frames
```

생성된 `frames.pdf`에서 대략 아래 연결이 있어야 한다.

```text
map -> odom -> base_footprint -> base_link -> laser
```

## 13. Teleop으로 로봇 움직이기

컨테이너 VSCode 터미널 3을 연다.

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

권장 주행:

- F1: 엘리베이터 앞 → 오른쪽 복도 → 택배존 앞 → 뒤쪽 복도 방향 확인
- F2/F3: 엘리베이터 앞 → 오른쪽 복도 → 좌측 코너 → 뒤쪽 복도 → 방 문 앞 왕복

RViz2에서 `/map`이 조금씩 채워지는지 확인한다.

## 14. 맵 저장 폴더 준비

호스트와 컨테이너 모두에서 같은 위치를 바라본다.

```text
호스트:     ~/2026_graduation_project/src/slam_pkg/maps
컨테이너:   /ros2_ws/maps
```

컨테이너 VSCode 터미널에서 폴더를 만든다.

```bash
mkdir -p /ros2_ws/maps/kku_virtual/f1
mkdir -p /ros2_ws/maps/kku_virtual/f2
mkdir -p /ros2_ws/maps/kku_virtual/f3
```

호스트 권한 문제가 있으면 호스트 터미널에서 한 번 정리한다.

```bash
cd ~/2026_graduation_project
sudo chown -R "$USER:$USER" src/slam_pkg/maps
```

## 15. 맵 저장

시뮬레이션을 켜고 teleop으로 충분히 주행한 뒤, 컨테이너 VSCode 터미널 2에서 실행한다.

F1:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/kku_virtual/f1/kku_f1
```

F2:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/kku_virtual/f2/kku_f2
```

F3:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/kku_virtual/f3/kku_f3
```

저장 확인:

```bash
find /ros2_ws/maps/kku_virtual -maxdepth 3 -type f | sort
```

호스트에서도 확인한다.

```bash
find ~/2026_graduation_project/src/slam_pkg/maps/kku_virtual -maxdepth 3 -type f | sort
```

각 층마다 `.pgm`과 `.yaml` 한 쌍이 있으면 성공이다.

## 16. 층별 반복 절차

층별 맵 생성을 할 때는 한 번에 한 층만 띄운다.

1. 터미널 1에서 기존 launch 종료: `Ctrl + C`
2. 다음 층 launch:

   ```bash
   ros2 launch slam_pkg kku_simulation.launch.py floor:=F2
   ```

3. teleop 터미널은 그대로 두거나 다시 실행
4. 충분히 주행
5. 해당 층 경로에 map 저장

권장 순서:

```text
F1 -> 저장: /ros2_ws/maps/kku_virtual/f1/kku_f1
F2 -> 저장: /ros2_ws/maps/kku_virtual/f2/kku_f2
F3 -> 저장: /ros2_ws/maps/kku_virtual/f3/kku_f3
```

## 17. 종료

컨테이너 VSCode 터미널에서 launch를 종료한다.

```text
Ctrl + C
```

호스트 터미널에서 컨테이너를 내린다.

```bash
cd ~/2026_graduation_project
docker compose -f docker_env/compose/docker-compose.linux.yml down
```

다음에도 바로 쓰려면 `down` 대신 컨테이너를 켜둬도 된다.

## 18. 자주 나는 문제

### `ament_package`를 못 찾는 빌드 오류

원인: ROS setup이 source되지 않은 상태에서 `colcon build`를 실행했다.

해결:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg
```

### RViz2/Gazebo GUI가 안 뜸

원인: X11 권한 또는 DISPLAY 불일치.

해결:

```bash
xhost +local:docker
xhost +local:root
docker compose -f docker_env/compose/docker-compose.linux.yml restart
```

### `/scan`이 안 나옴

확인:

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

Gazebo에서 로봇 spawn이 실패했거나 LiDAR plugin이 로드되지 않았을 수 있다. launch 터미널에서 `Successfully spawned entity`와 `Registering sensor` 메시지를 먼저 확인한다.

### `/map`이 비어 있음

SLAM Toolbox는 로봇이 움직이고 `/scan`, `/odom`, `/tf`가 맞아야 map을 채운다.

확인:

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run drive_pkg keyboard_teleop
```

teleop으로 조금 움직인 뒤 RViz2의 Map display를 다시 본다.

### map 저장 위치가 헷갈림

이 프로젝트 기준은 아래다.

```text
컨테이너 저장 경로: /ros2_ws/maps/kku_virtual/f{1,2,3}
호스트 실제 경로:   ~/2026_graduation_project/src/slam_pkg/maps/kku_virtual/f{1,2,3}
```

`docker_env/compose/docker-compose.linux.yml`에서 `PACKAGU_MAPS`를 바꿨다면 실제 저장 위치도 달라질 수 있다.

## 19. 성공 체크리스트

- [ ] `docker ps`에서 `ros2_humble`이 `Up`
- [ ] VSCode가 `ros2_humble` 컨테이너에 attach됨
- [ ] VSCode에서 `/ros2_ws` 폴더를 열었음
- [ ] `colcon build --symlink-install --packages-select common_pkg slam_pkg drive_pkg` 성공
- [ ] `ros2 launch slam_pkg kku_simulation.launch.py floor:=F1` 실행
- [ ] Gazebo 창 표시
- [ ] RViz2 창 표시
- [ ] `/scan` 수신
- [ ] `/odom` 수신
- [ ] `/map` 생성
- [ ] teleop으로 로봇 이동
- [ ] F1/F2/F3 맵을 `/ros2_ws/maps/kku_virtual/` 아래 저장
