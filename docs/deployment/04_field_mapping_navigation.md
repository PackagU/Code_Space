# 2026-09-12 실차 LiDAR 매핑 → 저장 지도 Nav2 가이드

기준일: 2026-09-12 KST

## 현재 판정

- LiDAR 단독 스캔과 핸드헬드 SLAM: **실물 검증**. 2026-09-11 저장 지도는 SLAM 동작 증명용이며 실차 주행용이 아니다.
- 실차 베이스·매핑·Nav2 분리 실행 코드: **오프라인 검증**.
- OpenCR Jetson 브리지 펌웨어: **코드 존재**. 실제 OpenCR 컴파일·업로드·watchdog 정지는 ⚠️미확인.
- 실제 wheel odom, LiDAR 장착 TF, AMCL 위치추정, Nav2 주행: **⚠️미확인**.
- 이 프로필은 로봇팔·리프트 노드를 시작하지 않으며 Docker에서도 관련 포트를 `/dev/null`로 막는다.

## 0. 현장 연결 구조

```text
노트북 ── Wi-Fi/SSH ──> Jetson
                          ├─ USB ──> OpenCR ──> Dynamixel 바퀴
                          └─ USB ──> LiDAR
```

노트북은 OpenCR에 직접 연결하지 않는다. 매핑 중에는 SSH 터미널의 텔레옵 노드가 `/cmd_vel`을 발행하고, 저장 지도 주행 중에는 Jetson의 Nav2가 같은 토픽을 발행한다. 두 경우 모두 `nav_safety_gate`를 지난 `/cmd_vel_safe`만 OpenCR 브리지에 전달된다. SSH 또는 Wi-Fi가 끊겨 새 명령이 오지 않으면 소프트웨어 gate와 OpenCR의 500 ms watchdog이 각각 정지 명령을 만든다. 물리 E-Stop은 별도로 필요하다.

현재 벤치용 `w/a/s/d/q` 펌웨어만 올라가 있다면 SSH로 문자 제어는 가능해도 wheel odom 피드백이 없어 이 구성의 SLAM/Nav2에는 쓸 수 없다. 아래 브리지 펌웨어를 먼저 올려야 한다. 펌웨어 업로드가 끝난 뒤에는 OpenCR USB를 Jetson에 계속 연결한다.

## 오늘 필요한 사람과 안전 조건

실제 바퀴가 움직이는 단계부터는 두 사람이 있는 것을 권장한다. 한 명은 노트북/Jetson, 한 명은 물리 E-Stop과 로봇을 담당한다.

1. 팔·리프트 전원은 끄고, 적재물은 제거한다.
2. 첫 OpenCR 확인은 바퀴를 지면에서 띄운 상태로 한다.
3. 로봇 전후좌우 1 m 이상을 비우고, 계단·엘리베이터 문·유리문 근처에서 시작하지 않는다.
4. 물리 E-Stop이 실제로 구동 전원을 끊는지 먼저 확인한다. 소프트웨어 정지는 E-Stop 대체가 아니다.
5. 이상 방향, 지속 회전, 소음, 케이블 말림, odom 역방향 중 하나라도 보이면 즉시 E-Stop 후 종료한다.

## 1. LiDAR 고정과 TF 값

전원을 끈 상태에서 LiDAR를 차체에 단단히 고정한다. 손으로 흔들었을 때 자세가 바뀌면 지도도 겹친다. 스캔 평면은 바닥과 수평이고, 주변 구조물이나 케이블이 360° 시야를 가리지 않게 한다.

`base_link` 기준으로 LiDAR 중심을 재서 기록한다.

- X: 로봇 앞쪽이 양수
- Y: 로봇 왼쪽이 양수
- Z: 위쪽이 양수
- yaw: 위에서 봤을 때 반시계가 양수, 단위 rad

현재 기본값 `x=-0.1015, y=0, z=0.750, roll=pitch=yaw=0`은 기존 모델값이며 실제 장착값은 ⚠️미확인이다. 다르면 아래 실행 때 `LASER_X/Y/Z/YAW`로 바꾼다.

## 2. OpenCR 펌웨어 준비

기존 `w/a/s/d/q` 벤치 코드는 Jetson 명령 `V left_rpm right_rpm`과 실측 RPM 피드백 `F left_rpm right_rpm`이 없으므로 실차 wheel odom에 사용할 수 없다.

현장 승인 후 Arduino IDE에서 다음 스케치를 연다.

```text
/home/hsm/Code_Space/src/drive_pkg/firmware/opencr/opencr_drive_bridge.ino
```

보드는 OpenCR, USB 포트는 실제 OpenCR 포트를 선택한다. 이 스케치는 사용자가 확인한 Dynamixel ID 1/2, Protocol 2.0, 1 Mbps를 사용하며, Jetson USB CDC는 115200 baud다. 업로드 후 Serial Monitor는 반드시 닫는다.

업로드 전 상태는 **코드 존재**일 뿐이다. 바퀴를 띄운 상태에서 명령 두절 500 ms 뒤 정지와 좌우 방향을 확인하기 전에는 지면 주행으로 넘어가지 않는다.

## 3. 노트북에서 Jetson 접속과 장치 전달

```bash
ssh hsm@192.168.0.7
cd /home/hsm/Code_Space
systemctl --user stop jetson-thermal-watchdog-v2.service 2>/dev/null || true
readlink -f /dev/rplidar
readlink -f /dev/opencr
```

두 경로가 `/dev/null`이 아니고 실제 `ttyUSB*` 또는 `ttyACM*`를 가리켜야 한다. 그다음 LiDAR+OpenCR만 컨테이너에 전달한다.

```bash
docker compose \
  -f docker/compose/docker-compose.jetson.yml \
  -f docker/compose/docker-compose.jetson.mapping.yml \
  up -d --force-recreate ros2
```

팔과 리프트 포트는 이 구성에서 계속 차단된다.

설치본이 최신인지 한 번 빌드한다.

```bash
docker exec -it -w /ros2_ws ros2_humble bash -lc \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select common_pkg drive_pkg slam_pkg'
```

## 4. H01 바퀴 들림 확인

터미널 A에서 베이스를 시작한다. `ENABLE_DRIVE=1`은 실제 OpenCR 포트를 열지만, 새 명령이 없으면 0 RPM만 전송한다.

```bash
cd /home/hsm/Code_Space
ENABLE_DRIVE=1 \
LASER_X=-0.1015 LASER_Y=0 LASER_Z=0.750 LASER_YAW=0 \
./scripts/start_field_base.sh
```

다른 터미널에서 상태를 확인한다.

```bash
docker exec -it -w /ros2_ws ros2_humble bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 topic hz /scan'
```

```bash
docker exec -it -w /ros2_ws ros2_humble bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 topic echo --once /drive/ready'
```

`data: true`가 나오고, 정지 상태에서 `/odom`이 크게 움직이지 않아야 한다. 바퀴 방향/500 ms watchdog의 실물 검증이 끝나지 않았다면 지면에 내리지 않는다.

## 5. SLAM 시작과 저속 수동 매핑

터미널 B:

```bash
cd /home/hsm/Code_Space
./scripts/start_field_mapping.sh
```

터미널 C:

```bash
cd /home/hsm/Code_Space
./scripts/teleop.sh
```

키는 `w/a/s/d`, 정지는 `k` 또는 Space다. 기본 속도는 `[제안값] 0.10 m/s`, `[제안값] 0.35 rad/s`이고, 0.5초 동안 새 키가 없으면 정지한다.

매핑 순서:

1. 시작점에서 5~10초 정지해 초기 스캔을 안정화한다.
2. 복도 중앙을 매우 천천히 한 바퀴 돈다. 회전은 가능하면 정지에 가깝게 천천히 한다.
3. 시작점으로 돌아와 같은 벽이 겹치는지 확인한다.
4. 긴 복도·코너를 한 차례 더 돌아 폐루프 보정을 만든다.
5. 사람이나 이동 장애물이 많은 구간은 잠시 기다렸다가 다시 스캔한다.

벽이 이중으로 갈라지거나 지도가 계속 회전하면 저장하지 말고 LiDAR 고정, TF, 좌우 바퀴 부호, wheel odom을 먼저 고친다.

## 6. 지도 저장

SLAM이 실행 중인 상태에서 터미널 D:

```bash
cd /home/hsm/Code_Space
./scripts/save_field_map.sh F1
```

출력 마지막의 `MAP_YAML=/ros2_ws/maps/field/f1/....yaml`을 복사한다. 저장물은 YAML, PGM, posegraph, data, SHA256 manifest이며 기존 이름을 덮어쓰지 않는다.

텔레옵에서 `k`를 누르고 Ctrl+C로 끝낸 뒤, 매핑 터미널 B도 Ctrl+C로 끝낸다. 터미널 A의 베이스는 계속 켜 둔다.

## 7. 저장 지도 기반 Nav2 무동작 기동

터미널 B에서 방금 출력된 지도 경로를 넣는다.

```bash
cd /home/hsm/Code_Space
FLOOR=F1 ./scripts/start_field_navigation.sh \
  /ros2_ws/maps/field/f1/방금_생성된_지도.yaml
```

이 명령은 다음을 확인한 뒤 AMCL/Nav2만 시작한다.

- 지도 YAML/PGM 구조가 정상
- SLAM Toolbox가 종료됨
- `/scan`, `/odom`, `/drive/ready`, `/nav_safety/ready`가 정상
- 팔·리프트·서보 노드가 없음

이 단계는 목표를 보내지 않으므로 로봇이 움직이지 않아야 한다. RViz에서 지도를 열고 **2D Pose Estimate**로 실제 위치와 방향을 먼저 지정한다. 스캔이 지도 벽과 안정적으로 겹치기 전에는 목표를 보내지 않는다.

## 8. 최초 자율주행 시험

이 단계부터 다시 실제 구동 승인과 E-Stop 담당자가 필요하다.

1. 장애물이 없는 동일 복도에서 0.5~1 m 앞 지점만 Nav2 Goal로 지정한다.
2. 출발 즉시 실제 진행 방향, RViz odom/AMCL 방향, 스캔 정합을 확인한다.
3. 목표 도달 후 10초간 완전 정지를 확인한다.
4. 다음으로 90° 회전 1회, 짧은 왕복, 코너 1개 순서로 범위를 늘린다.
5. 엘리베이터, 문턱, 사람 통행 구간은 이 시험에 포함하지 않는다.

현재 Nav2 상한은 `[제안값] 0.10 m/s`, `[제안값] 0.35 rad/s`; 두 값을 동시에 명령해도 펌웨어의 60 rpm 상한 아래에 남는다. 미탐색 영역 경로는 금지하며 실제 검증 전에는 올리지 않는다.

## 9. 종료

Nav2 터미널 Ctrl+C → 텔레옵 `k` 확인 → 베이스 터미널 Ctrl+C 순서다. 비정상 시에는 순서보다 물리 E-Stop이 우선이다.

매핑 중 센서·TF 기록, motion-safe 재생, 30분 부하 측정과 복구 절차는 [05_record_replay_recovery.md](05_record_replay_recovery.md)를 따른다. 기본 재생은 `/cmd_vel`, 팔, 리프트, goal 토픽을 제외하고 실제 graph와 다른 ROS domain만 사용한다.

## 내일 성공 판정

- **SLAM 실물 검증**: 고정 LiDAR + 실제 wheel odom으로 폐루프 지도 생성, YAML/PGM/posegraph 저장, 재로딩 성공.
- **Nav2 무동작 실물 검증**: 저장 지도 로딩, AMCL 초기 위치 후 스캔 정합, 목표 없이 정지 유지.
- **자율주행 실물 검증**: 별도 승인 아래 짧은 목표를 저속 완주하고 명령 중단/목표 도달 뒤 정지 확인.
- 위 조건을 실제 로그로 확인하기 전에는 코드 존재 또는 오프라인 검증으로만 기록한다.
