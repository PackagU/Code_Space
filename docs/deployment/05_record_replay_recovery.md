# Field rosbag, replay, load, and recovery

이 문서는 P07의 기록·재생·부하·복구 절차다. 현재 판정은 하드웨어 없는 합성 입력 기준 **오프라인 검증**이며, 실제 LiDAR·wheel odom·IMU·카메라의 30분 부하는 H02와 C02 이후 별도로 측정한다.

## 1. 기록

Jetson 호스트의 `Code_Space`에서 base와 mapping을 먼저 시작한 뒤 별도 터미널에서 실행한다.

```bash
cd /home/hsm/Code_Space
./scripts/record_field_bag.sh
```

기본 기록 대상은 `/scan`, `/odom`, `/tf`, `/tf_static`이며 `/imu`가 있으면 포함한다. 카메라는 C02 이후에만 명시적으로 포함한다.

```bash
INCLUDE_CAMERA=1 REQUIRE_IMU=1 ./scripts/record_field_bag.sh
```

`/cmd_vel`, `/cmd_vel_safe`, `/diagnostics`는 `INCLUDE_CONTROL_DIAGNOSTICS=1`일 때만 진단 목적으로 기록한다. 기록 여부와 무관하게 재생 스크립트는 구동·팔·리프트 토픽을 재생하지 않는다.

무인 제한시간 기록 예시는 다음과 같다. `[제안값]` 30분은 현장 측정용 기본값일 뿐 합격을 뜻하지 않는다.

```bash
DURATION_SEC=1800 ./scripts/record_field_bag.sh f1_mapping_run_01
```

결과는 호스트에 영속되는 `/ros2_ws/logs/field_bags/<name>`에 저장된다. 기존 이름은 덮어쓰지 않는다.

## 2. 검사

```bash
./scripts/inspect_field_bag.sh /ros2_ws/logs/field_bags/f1_mapping_run_01
```

검사는 `metadata.yaml`, SQLite 파일, 실제 파일 크기, 기간, 전체 메시지 수, 토픽별 타입·건수, SHA256을 확인한다. `/scan`, `/odom`, `/tf`, `/tf_static`가 비었으면 실패한다. IMU가 현장 필수라면 다음처럼 요구한다.

```bash
REQUIRE_IMU=1 ./scripts/inspect_field_bag.sh /ros2_ws/logs/field_bags/f1_mapping_run_01
```

map PGM/YAML과 SLAM posegraph는 rosbag에 의존하지 않고 `save_field_map.sh` 결과와 그 SHA256 파일로 별도 보존한다.

## 3. 재생

재생은 실제 장치 graph와 다른 ROS domain에서만 실행한다. 기본 `reslam` 프로필은 센서·odom·로봇 내부 TF만 선택하고 `/cmd_vel`, 팔, 리프트, goal 토픽을 제외한다.

```bash
REPLAY_DOMAIN_ID=227 PHYSICAL_DOMAIN_ID=0 \
  ./scripts/replay_field_bag.sh /ros2_ws/logs/field_bags/f1_mapping_run_01
```

재SLAM 소비자는 같은 replay domain과 `use_sim_time=true`로 시작한다. 기존 AMCL 또는 다른 `map→odom` 발행자를 동시에 시작하지 않는다. 재생 domain에서 OpenCR·팔·리프트·controller·안전 게이트 노드가 발견되면 스크립트가 시작을 거부한다.

## 4. 30분 부하 측정

base와 센서가 정상 발행 중일 때 Jetson 호스트에서 실행한다.

```bash
DURATION_SEC=1800 INTERVAL_SEC=5 ./scripts/monitor_field_load.sh f1_load_run_01
```

`logs/field_load/<name>/`에 다음 근거가 남는다.

- ROS 센서 수신 건수, 측정 주기, gap p50·p95·p99·최댓값, header 시각 역행, header age 상위값
- thermal zone 최고값, load average, 가용 RAM, 루트 디스크 증가, 컨테이너 CPU·RAM
- 가능한 경우 원시 `tegrastats`와 모든 결과의 SHA256

합격 임계는 실제 제어 주기와 허용 누락을 정한 뒤 판정한다. 평균 FPS나 평균 scan rate만으로 PASS하지 않는다. 현재 카메라가 없어 카메라 포함 부하는 **미확인**이다.

## 5. 복구와 실패 처리

- recorder가 센서보다 늦게 시작해도 transient-local `/tf_static`와 계속 발행되는 센서가 기록돼야 한다.
- 재생은 별도 domain, `--clock`, safe allowlist로만 수행한다.
- 앱 또는 subscriber를 다시 시작하면 이전 명령을 복원하지 않는다. drive safety gate는 새 scan·odom·drive-ready가 모두 들어오기 전 0속도이고, arm 계약은 새 요청 없이 시작하지 않는다.
- 장치 없는 컨테이너 재생성 뒤에도 host `logs/`와 `maps/`의 파일과 SHA256이 유지돼야 한다.
- 실제 LiDAR 종료·USB 재연결·OpenCR watchdog·물리 E-Stop은 H01/H02 현장 시험으로 남긴다.

합성 late-join 기록과 안전 재생 회귀 시험은 다음 한 줄로 실행한다.

```bash
bash scripts/test_p07_rosbag_runtime.sh
```

이 시험은 bag에 `/cmd_vel`을 일부러 기록한 뒤 safe replay에서 해당 토픽 수신 건수가 0인지 확인한다. 합성 입력 성공은 실제 센서 처리량이나 자율주행 성공 근거가 아니다.
