# Docker 환경 사용 가이드

## 1. 처음 시작 (이미지 빌드 + 컨테이너 실행)

**Windows PowerShell** (`PS C:\...` 프롬프트) 에서 실행:

```powershell
# VcXsrv 먼저 실행 (GUI 필요 시)
.\docker\scripts\start_vcxsrv.bat

# 이미지 빌드 + 컨테이너 실행
docker compose -f docker/docker-compose.yml up --build -d
```

> `--build` 는 Dockerfile이 바뀌었을 때만 필요. 평소엔 `up -d` 만 써도 됨.

---

## 2. 컨테이너 접속 방법

### 방법 A — VS Code "Attach to Running Container" (권장)
1. VS Code 좌하단 `><` 클릭 → **Attach to Running Container**
2. `ros2_humble` 선택
3. 새 VS Code 창에서 터미널 열면 바로 ROS2 환경 적용된 상태

### 방법 B — PowerShell에서 직접 접속
```powershell
docker exec -it ros2_humble bash
```

---

## 3. 환경변수 확인

컨테이너 안에서:

```bash
echo $DISPLAY          # host.docker.internal:0.0  (GUI 출력)
echo $ROS_DOMAIN_ID    # 0
echo $ROS_DISTRO       # humble
printenv | grep ROS    # ROS 관련 전체 확인
```

> `docker-compose.yml` 에서 설정된 값:
> - `DISPLAY=host.docker.internal:0.0`
> - `QT_X11_NO_MITSHM=1`
> - `ROS_DOMAIN_ID=0`

---

## 4. 빌드 (colcon)

```bash
cd /ros2_ws
colcon build --symlink-install

# 특정 패키지만 빌드할 때
colcon build --symlink-install --packages-select slam_pkg
colcon build --symlink-install --packages-select common_pkg
```

> `src/` 폴더는 호스트(Windows)와 마운트되어 있어서 Windows에서 파일 수정 → 컨테이너 안에서 바로 반영됨.
> `--symlink-install` 덕분에 Python 파일은 빌드 없이도 반영됨. C++ 파일은 재빌드 필요.

---

## 5. 시뮬레이션 실행

터미널을 **3개** 열어서 순서대로 실행:

### 터미널 1 — Gazebo (로봇 시뮬레이터)
```bash
ros2 launch common_pkg gazebo.launch.py
```

### 터미널 2 — SLAM (지도 생성 + RViz2)
```bash
ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true
```

### 터미널 3 — 키보드 원격 조종 (로봇 움직이기)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```
> 터미널 3 포커스 상태에서 `i` = 전진, `,` = 후진, `j/l` = 회전, `k` = 정지

---

## 6. 실제 하드웨어 실행 (RPLiDAR 연결 시)

`docker-compose.yml` 에서 devices 주석 해제 후 재시작:
```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0   # RPLiDAR
  - /dev/ttyACM0:/dev/ttyACM0   # OpenCR
```

```bash
# SLAM — use_sim_time:=false (기본값이라 생략 가능)
ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=false
```

---

## 7. Dockerfile 수정 후 이미지 재빌드

**PowerShell** 에서:
```powershell
# 컨테이너 중지 + 이미지 재빌드 + 재시작
docker compose -f docker/docker-compose.yml down
docker compose -f docker/docker-compose.yml up --build -d
```

> 주의: `down` 하면 컨테이너 안에서 직접 설치한 패키지는 사라짐.
> 필요한 패키지는 반드시 `Dockerfile` 에 추가할 것.

---

## 8. 팀원 공유 체크리스트

```
[ ] git pull origin dev
[ ] docker compose -f docker/docker-compose.yml up --build -d  (첫 실행 또는 Dockerfile 변경 시)
[ ] docker compose -f docker/docker-compose.yml up -d          (이후 일반 실행)
[ ] VS Code → Attach to Running Container → ros2_humble
[ ] cd /ros2_ws && colcon build --symlink-install
```

---

## 9. 자주 쓰는 명령어 모음

```bash
# 컨테이너 상태 확인 (PowerShell)
docker ps

# 컨테이너 중지
docker compose -f docker/docker-compose.yml down

# 로그 확인
docker logs ros2_humble

# ROS2 토픽 목록
ros2 topic list

# ROS2 노드 목록
ros2 node list

# 맵 저장 (SLAM 중)
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/my_map
```

---

## 10. 자주 발생하는 오류 & 해결

### ❌ `'source'은(는) 내부 또는 외부 명령이 아닙니다`

**원인**: `source`는 bash 명령어 — PowerShell에서는 동작하지 않음.

**해결**: 컨테이너 안에서 실행해야 함.
```powershell
# PowerShell에서 컨테이너 접속 후 bash에서 실행
docker exec -it ros2_humble bash
# 이후 컨테이너 내부에서 source 명령 사용 가능
```

---

### ❌ `ModuleNotFoundError: No module named 'ament_package'` (colcon build 중)

**원인**: ROS2 환경이 소싱되지 않은 상태에서 `colcon build` 실행.

**해결**: 빌드 전 ROS2 환경 소싱 후 재시도.
```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install
```

> **근본 원인 — VS Code "Attach to Running Container"**: VS Code Attach는 Docker `ENTRYPOINT`를 우회하므로 `entrypoint.sh`의 `source` 명령이 실행되지 않음.
> 현재 `Dockerfile`에서 `/etc/bash.bashrc`에 자동 소싱을 추가해 해결함:
> ```dockerfile
> RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
>     echo 'if [ -f "/ros2_ws/install/setup.bash" ]; then source /ros2_ws/install/setup.bash; fi' >> /etc/bash.bashrc
> ```
> 이미지 재빌드 후에는 Attach 터미널에서도 자동으로 ROS2 환경이 소싱됨.

---

### ❌ `InvalidPackage: Maintainers must have an email address` (colcon build 중)

**원인**: `package.xml`의 `<maintainer>` 태그에 `email=""` 빈 값.

**해결**: `package.xml`에 이메일 추가 (placeholder도 가능).
```xml
<!-- 수정 전 -->
<maintainer email="">inonewater</maintainer>

<!-- 수정 후 -->
<maintainer email="inonewater@todo.com">inonewater</maintainer>
```

> 현재 `src/drive_pkg/package.xml`, `src/robot_arm_pkg/package.xml` 모두 수정 완료.

---

### ❌ `Package 'common_pkg' not found` (빌드 직후 launch 실행 시)

**원인**: `colcon build` 완료 후 워크스페이스 오버레이(`/ros2_ws/install/setup.bash`)가 소싱되지 않은 상태에서 `ros2 launch` 실행.

**해결**: 빌드 후 반드시 워크스페이스 재소싱.
```bash
colcon build --symlink-install
source /ros2_ws/install/setup.bash   # ← 이 줄이 빠지면 launch 실패
ros2 launch common_pkg gazebo.launch.py
```

> **새 터미널 열면 자동 해결**: Dockerfile에 `/etc/bash.bashrc` 자동 소싱이 추가되어 있어, 이미지 재빌드 후 새로 연 터미널은 별도 `source` 없이도 동작함. 단, 빌드와 같은 터미널에서 바로 launch할 때는 위처럼 수동 소싱 필요.

---

### ❌ 시뮬레이션 SLAM에서 use_sim_time 누락

**원인**: Gazebo 시뮬레이션에서 SLAM을 실행할 때 `use_sim_time:=true` 없이 실행.

**해결**:
```bash
# 시뮬레이션 시
ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true

# 실제 하드웨어 시
ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=false
```
