# HW 확정 시 업데이트 체크리스트

구동부·센서·로봇팔 하드웨어가 확정/장착될 때 바꿔야 하는 파일과 검증 명령의 단일 목록.
시뮬 스택은 HW 없이 완결된 상태(2026-08-17 왕복 체인 기준)이며, 아래 항목만 갱신하면 실기 테스트로 전환된다.

## 1. 구동부 모터 확정 시 (Han)

| 항목 | 파일 | 바꿀 값 |
|------|------|--------|
| 바퀴 반경/질량/관성 | `src/common_pkg/urdf/delivery_robot.urdf.xacro` | 바퀴 반경(현 0.033m), `wheel_separation`(현 0.51324m), dynamics damping/friction |
| 속도·가속 한계 | `src/slam_pkg/config/nav2_params.yaml` | controller 속도 한계를 모터 실측 최고속/가속으로 |
| OpenCR 브리지 | `src/drive_pkg/drive_pkg/opencr_bridge_node.py` 파라미터 | `wheel_radius`, `wheel_separation`, `left_sign`/`right_sign`(배선 방향), `cmd_timeout_sec`(현 0.5s — improvement_report §1.22d 검토) |
| 시리얼 장치 | `scripts/udev/99-packagu-devices.rules` + Jetson `docker/compose/.env` | `OPENCR_DEVICE=/dev/opencr`, `MOTOR_NANO_DEVICE=/dev/motor_nano` |

검증:

```bash
python3 scripts/check_portability.py
bash scripts/run_offline_tests.sh
# Jetson: 브리지 dry-run
ros2 launch drive_pkg drive_bringup.launch.py
```

## 2. RPLiDAR 재장착 시

| 항목 | 내용 |
|------|------|
| udev | `sudo bash scripts/udev/install_udev_rules.sh` 후 `ls -l /dev/rplidar` 확인 |
| compose | Jetson `docker/compose/.env` 에 `RPLIDAR_DEVICE=/dev/rplidar` 추가 후 `docker compose -f docker/compose/docker-compose.jetson.yml up -d` (recreate) |
| 주의 | 장치 매핑은 컨테이너 생성 시 고정 — 케이블 재연결만으로는 반영되지 않으니 recreate 필요 |

## 3. 로봇팔 재장착 시 (Kim)

| 항목 | 내용 |
|------|------|
| udev | CH340 `1a86:7523` — Nano 클론과 겹치면 rules 파일의 KERNELS 분리 주석 참조 |
| compose | `.env` 에 `ARM_SERVO_DEVICE=/dev/arm_servo` + recreate |
| smoke | `WITH_ARM=1 ARM_SERIAL_PORT=/dev/arm_servo` — 미지정 시 topic 전용 mock |
| 벤치 확인 | `ros2 run robot_arm_pkg arm_sequence` self_test (9초 사이클 구동 확인) |

## 4. Depth 카메라 확정 시

| 항목 | 내용 |
|------|------|
| udev | `scripts/udev/99-packagu-devices.rules` 에 VID/PID 규칙 추가 (`lsusb` 로 실측) |
| compose | `docker-compose.jetson.yml` `devices:` 에 `${DEPTH_CAM_DEVICE:-/dev/null}:/dev/depth_cam` 패턴으로 추가 |
| 드라이버 | ROS2 드라이버 패키지를 `docker/Dockerfile.jetson` 에 추가 (시뮬 의존 금지 원칙 — portability §3 확인) |

## 5. 공통 규칙

- 장치 매핑은 전부 env 변수 + 기본 `/dev/null` — compose 파일 직접 수정(주석 토글) 금지, `.env` 만 편집 (pull 충돌 방지)
- `.env` 는 gitignore 대상 — 머신별 로컬 설정
- HW 값 변경 후 커밋 전: `python3 scripts/check_portability.py` + `bash scripts/run_offline_tests.sh` PASS 필수
- 변경 내역은 `docs/hardware_spec.md` (HW SSOT) 에도 반영
