# Jetson 실측 준비 구현 계획 (신공학관 LiDAR 맵핑)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 스펙 `docs/superpowers/specs/2026-08-08-jetson-field-mapping-design.md`의 코드 산출물(§5.4, §7)을 전부 구현한다 — OpenCR 브리지 스택, teleop dead-man, 실기 매핑 launch, 실맵 저장/field 원커맨드 스크립트, udev, compose 정비, Stage 0 이미지.

**Architecture:** drive_pkg에 순수 파이썬 모듈(프로토콜/오도메트리)을 먼저 TDD로 만들고, 그 위에 rclpy 브리지 노드를 얹는다. 실기 매핑은 기존 `slam_toolbox.launch.py`를 리팩터링해 RSP/RViz 조건부/드라이브 include를 추가한다. 모든 산출물은 기존 오프라인 테스트 러너(`scripts/run_offline_tests.sh`)에 등록된다.

**Tech Stack:** ROS2 Humble(rclpy), pyserial, SLAM Toolbox, ament_cmake(+ament_cmake_python), bash 계약 테스트 패턴(기존 repo 관례).

## Global Constraints

- 이식성 규칙 R1~R4 준수, 모든 태스크 커밋 전 `python3 scripts/check_portability.py` 통과 (`docs/deployment/01_portability_policy.md`)
- 시리얼 포트는 launch `DeclareLaunchArgument`로만 (R3). 기본값은 udev 별칭 `/dev/opencr`, `/dev/rplidar`
- 프레임 계약: `map→odom`(SLAM Toolbox) / `odom→base_footprint`(브리지 TF) / URDF `base_footprint`→`laser` (slam_toolbox_params.yaml `base_frame: base_footprint` 기준)
- 토픽 계약: `/cmd_vel`(sub), `/odom`·`/imu`(pub). watchdog 0.5s 양측(펌웨어+브리지)
- 시리얼 프로토콜은 v0.1 초안 — Han 합의 전. 합의로 바뀌면 Task 1 문서와 Task 2 모듈만 수정하면 되는 구조 유지
- `docs/` 하위 새 문서는 Notion+Obsidian 호환 스타일(H1~H3, 코드블록 언어 태그, wikilink/HTML 금지 — AGENTS.md #10)
- `src/drive_pkg/`(기본 Codex 담당)와 `docker/`(공용) 편집 전 `TODO.md`에 "편집 중" lock 표시, 세션 종료 시 해제
- 커밋 메시지는 repo 관례(`feat:`/`fix:`/`docs:` + 한국어 요약) + `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`
- OpenCR 펌웨어 자체는 Han 담당 — 이 계획 범위 밖 (프로토콜 문서와 브리지가 인터페이스)

---

### Task 1: 시리얼 프로토콜 계약 문서 v0.1

**Files:**
- Create: `docs/deployment/02_opencr_serial_protocol.md`

**Interfaces:**
- Produces: V/F 라인 프레임 정의 — Task 2의 인코더/파서, Han의 펌웨어가 이 문서를 따름

- [ ] **Step 1: 문서 작성**

```markdown
# OpenCR Serial Protocol v0.1 (draft)

> Jetson(브리지) <-> OpenCR(펌웨어) 시리얼 계약. 상태: v0.1 초안 — Han 합의 후 v1.0 승격.
> 소비자: `src/drive_pkg/drive_pkg/opencr_protocol.py`(Lee), OpenCR 펌웨어(Han).
> 변경 절차: 이 문서 갱신 -> 양측 코드 갱신 -> 버전 문자열 동기.

## 1. 물리 계층

| 항목 | 값 |
|------|-----|
| 포트 | USB CDC (`/dev/opencr` udev 별칭) |
| 속도 | 115200 8N1 |
| 인코딩 | ASCII, 공백 구분, `\n` 종료 |

## 2. 명령 프레임 (Jetson -> OpenCR, 20Hz)

```text
V <left_rpm> <right_rpm>\n
```

- float, 소수 2자리. 양수 = 로봇 전진 방향 회전 (좌우 동일 규약)
- 부호 보정(모터 배선 반전)은 브리지의 `left_sign`/`right_sign` 파라미터가 담당.
  펌웨어는 받은 값을 그대로 모터에 적용한다.

## 3. 피드백 프레임 (OpenCR -> Jetson, 50Hz)

```text
F <left_rpm> <right_rpm> <gx> <gy> <gz> <ax> <ay> <az> <qw> <qx> <qy> <qz>\n
```

| 필드 | 단위 | 의미 |
|------|------|------|
| left_rpm, right_rpm | rpm | 엔코더 실측 바퀴 속도 (전진 = 양수) |
| gx gy gz | rad/s | 자이로 |
| ax ay az | m/s^2 | 가속도 |
| qw qx qy qz | - | IMU 자세 쿼터니언 (정규화) |

## 4. 안전 규약 (watchdog)

- 펌웨어: `V` 프레임 500ms 미수신 -> 모터 정지 (필수, Han)
- 브리지: `/cmd_vel` 500ms 미수신 -> `V 0.00 0.00` 송신 (이중 안전)

## 5. 부팅/에러 (선택 구현)

```text
HELLO opencr <fw_version>\n
E <code> <message...>\n
```

브리지는 `V`/`F` 이외 라인을 로그만 남기고 무시한다 (전방 호환).
```

- [ ] **Step 2: 스타일 확인** — H1~H3만 사용, 코드블록 언어 태그(text/markdown), wikilink 없음 육안 확인
- [ ] **Step 3: Commit**

```bash
git add docs/deployment/02_opencr_serial_protocol.md
git commit -m "docs(drive): OpenCR 시리얼 프로토콜 v0.1 초안 — V/F 프레임 + 0.5s watchdog 계약"
```

---

### Task 2: drive_pkg 파이썬 패키지화 + 프로토콜 모듈 (TDD)

**Files:**
- Create: `src/drive_pkg/drive_pkg/opencr_protocol.py`
- Create: `scripts/test_opencr_protocol.py`
- Modify: `src/drive_pkg/CMakeLists.txt` (ament_cmake_python 추가)
- Modify: `src/drive_pkg/drive_pkg/__init__.py` (내용 없으면 그대로)
- Modify: `scripts/run_offline_tests.sh:21` 근처 PY_TESTS 목록

**Interfaces:**
- Produces:
  - `encode_velocity_command(left_rpm: float, right_rpm: float) -> bytes` — `b"V %.2f %.2f\n"`
  - `parse_feedback_line(line: str) -> dict | None` — 키: `left_rpm right_rpm gyro accel quat` (gyro/accel=tuple3, quat=tuple4). 형식 불일치 시 None
  - `twist_to_wheel_rpm(v: float, w: float, wheel_radius: float, wheel_separation: float) -> tuple[float, float]`

- [ ] **Step 1: 실패하는 테스트 작성** — `scripts/test_opencr_protocol.py` (기존 `test_wasd_teleop.py`의 importlib 패턴 그대로)

```python
#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "src" / "drive_pkg" / "drive_pkg" / "opencr_protocol.py"


def load_module():
    spec = importlib.util.spec_from_file_location("opencr_protocol", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def approx(a, b, tol=1e-6):
    if abs(a - b) > tol:
        raise AssertionError(f"expected {b}, got {a}")


def main():
    proto = load_module()

    # encode: 포맷/개행/소수2자리 (이진 반올림 모호값 금지 — 12.5처럼 정확한 값 사용)
    assert proto.encode_velocity_command(12.5, -6.0) == b"V 12.50 -6.00\n", "encode format"

    # parse: 정상 F 라인
    fb = proto.parse_feedback_line("F 60.0 -60.0 0.1 0.2 0.3 0.0 0.0 9.81 1.0 0.0 0.0 0.0")
    assert fb is not None, "parse returned None for valid line"
    approx(fb["left_rpm"], 60.0)
    approx(fb["right_rpm"], -60.0)
    approx(fb["gyro"][2], 0.3)
    approx(fb["accel"][2], 9.81)
    approx(fb["quat"][0], 1.0)

    # parse: 불량 라인은 None (프리픽스/필드수/비숫자)
    assert proto.parse_feedback_line("HELLO opencr 1.0") is None
    assert proto.parse_feedback_line("F 1.0 2.0") is None
    assert proto.parse_feedback_line("F a b c d e f g h i j k l") is None

    # twist -> rpm: v=0.2073 m/s 직진, r=0.033, L=0.51324 -> 양쪽 동일 rpm
    r, L = 0.033, 0.51324
    v = 0.033 * 2.0 * math.pi  # 바퀴 1rev/s = 60rpm이 되는 전진 속도
    left, right = proto.twist_to_wheel_rpm(v, 0.0, r, L)
    approx(left, 60.0)
    approx(right, 60.0)

    # 제자리 좌회전(w>0): 왼쪽 후진, 오른쪽 전진, 크기 동일
    left, right = proto.twist_to_wheel_rpm(0.0, 1.0, r, L)
    approx(left, -right)
    assert right > 0, "left turn must spin right wheel forward"

    print("opencr_protocol tests passed")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 실패 확인**

Run: `python3 scripts/test_opencr_protocol.py`
Expected: FAIL — `FileNotFoundError`/`AttributeError` (모듈 없음)

- [ ] **Step 3: 최소 구현** — `src/drive_pkg/drive_pkg/opencr_protocol.py`

```python
"""OpenCR 시리얼 프로토콜 v0.1 인코더/디코더 + 차동구동 변환.

계약 문서: docs/deployment/02_opencr_serial_protocol.md
순수 파이썬 (ROS 비의존) — 오프라인 테스트: scripts/test_opencr_protocol.py
"""
import math

FEEDBACK_FIELD_COUNT = 13  # "F" + 12 floats
PROTOCOL_VERSION = "0.1"


def encode_velocity_command(left_rpm, right_rpm):
    return f"V {left_rpm:.2f} {right_rpm:.2f}\n".encode("ascii")


def parse_feedback_line(line):
    tokens = line.strip().split()
    if len(tokens) != FEEDBACK_FIELD_COUNT or tokens[0] != "F":
        return None
    try:
        values = [float(token) for token in tokens[1:]]
    except ValueError:
        return None
    return {
        "left_rpm": values[0],
        "right_rpm": values[1],
        "gyro": tuple(values[2:5]),
        "accel": tuple(values[5:8]),
        "quat": tuple(values[8:12]),
    }


def twist_to_wheel_rpm(v, w, wheel_radius, wheel_separation):
    left_rad_s = (v - w * wheel_separation / 2.0) / wheel_radius
    right_rad_s = (v + w * wheel_separation / 2.0) / wheel_radius
    to_rpm = 60.0 / (2.0 * math.pi)
    return left_rad_s * to_rpm, right_rad_s * to_rpm
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `python3 scripts/test_opencr_protocol.py`
Expected: `opencr_protocol tests passed`

- [ ] **Step 5: CMake 파이썬 패키지 설치 추가** — `src/drive_pkg/CMakeLists.txt`를 아래로 교체

```cmake
cmake_minimum_required(VERSION 3.8)
project(drive_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# drive_pkg/ 파이썬 모듈 전체를 site-packages로 설치 (노드 간 import용)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  drive_pkg/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_teleop
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
```

- [ ] **Step 6: 러너 등록** — `scripts/run_offline_tests.sh`의 PY_TESTS 배열에서 `scripts/test_wasd_teleop.py` 다음 줄에 추가

```bash
  scripts/test_opencr_protocol.py
```

- [ ] **Step 7: 전체 오프라인 확인 + Commit**

Run: `bash scripts/run_offline_tests.sh`
Expected: 기존 PASS 유지 + `PASS scripts/test_opencr_protocol.py`, FAIL 0

```bash
git add src/drive_pkg/CMakeLists.txt src/drive_pkg/drive_pkg/opencr_protocol.py scripts/test_opencr_protocol.py scripts/run_offline_tests.sh
git commit -m "feat(drive): OpenCR 프로토콜 v0.1 인코더/파서 + twist->rpm 변환 (오프라인 테스트 포함)"
```

---

### Task 3: 차동구동 오도메트리 모듈 (TDD)

**Files:**
- Create: `src/drive_pkg/drive_pkg/diff_drive_odometry.py`
- Create: `scripts/test_diff_drive_odometry.py`
- Modify: `scripts/run_offline_tests.sh` (PY_TESTS 추가)

**Interfaces:**
- Consumes: 없음 (순수 모듈)
- Produces:
  - `DiffDriveOdometry(wheel_radius, wheel_separation, left_sign=1.0, right_sign=1.0)`
  - `.update(left_rpm: float, right_rpm: float, dt: float) -> None` — 내부 적분
  - 속성 `.x .y .yaw .v .w` (float)
  - `yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]` — (x, y, z, w)

- [ ] **Step 1: 실패하는 테스트 작성** — `scripts/test_diff_drive_odometry.py`

```python
#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "src" / "drive_pkg" / "drive_pkg" / "diff_drive_odometry.py"


def load_module():
    spec = importlib.util.spec_from_file_location("diff_drive_odometry", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def approx(a, b, tol=1e-6, label=""):
    if abs(a - b) > tol:
        raise AssertionError(f"{label}: expected {b}, got {a}")


def main():
    m = load_module()
    r, L = 0.033, 0.51324

    # 1) 직진: 양쪽 60rpm, 1.0s (dt=0.02 x 50) -> x = v*t (닫힌 형태, yaw 불변이라 오일러 적분 정확)
    odo = m.DiffDriveOdometry(wheel_radius=r, wheel_separation=L)
    for _ in range(50):
        odo.update(60.0, 60.0, 0.02)
    v_expected = 60.0 * 2.0 * math.pi / 60.0 * r  # rpm -> rad/s -> m/s
    approx(odo.x, v_expected * 1.0, 1e-9, "straight x")
    approx(odo.y, 0.0, 1e-9, "straight y")
    approx(odo.yaw, 0.0, 1e-9, "straight yaw")
    approx(odo.v, v_expected, 1e-9, "straight v")

    # 2) 제자리 회전: 좌 -30rpm / 우 +30rpm, 2.0s -> yaw = w*t, x=y=0
    odo = m.DiffDriveOdometry(wheel_radius=r, wheel_separation=L)
    for _ in range(100):
        odo.update(-30.0, 30.0, 0.02)
    w_expected = (30.0 * 2.0 * math.pi / 60.0) * r * 2.0 / L
    approx(odo.yaw, w_expected * 2.0, 1e-9, "spin yaw")
    approx(odo.x, 0.0, 1e-9, "spin x")
    approx(odo.w, w_expected, 1e-9, "spin w")

    # 3) sign 보정: 오른쪽 배선 반전 로봇 (right_sign=-1) 에서 피드백 (+60, -60) = 직진
    odo = m.DiffDriveOdometry(wheel_radius=r, wheel_separation=L, right_sign=-1.0)
    odo.update(60.0, -60.0, 1.0)
    approx(odo.x, v_expected, 1e-9, "sign-corrected x")
    approx(odo.yaw, 0.0, 1e-9, "sign-corrected yaw")

    # 4) 쿼터니언: yaw=pi/2 -> z=sin(pi/4), w=cos(pi/4)
    qx, qy, qz, qw = m.yaw_to_quaternion(math.pi / 2.0)
    approx(qx, 0.0, 1e-9, "qx")
    approx(qz, math.sin(math.pi / 4.0), 1e-9, "qz")
    approx(qw, math.cos(math.pi / 4.0), 1e-9, "qw")

    print("diff_drive_odometry tests passed")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 실패 확인**

Run: `python3 scripts/test_diff_drive_odometry.py`
Expected: FAIL (모듈 없음)

- [ ] **Step 3: 최소 구현** — `src/drive_pkg/drive_pkg/diff_drive_odometry.py`

```python
"""바퀴 rpm 피드백 -> (x, y, yaw, v, w) 오도메트리 적분 (순수 파이썬).

calib 값(wheel_radius/separation/sign)은 R1B 지면 캘리브레이션에서 확정 후
drive_calib.yaml에 반영한다. 오프라인 테스트: scripts/test_diff_drive_odometry.py
"""
import math

RPM_TO_RAD_S = 2.0 * math.pi / 60.0


class DiffDriveOdometry:
    def __init__(self, wheel_radius, wheel_separation, left_sign=1.0, right_sign=1.0):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.left_sign = left_sign
        self.right_sign = right_sign
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

    def update(self, left_rpm, right_rpm, dt):
        left = self.left_sign * left_rpm * RPM_TO_RAD_S * self.wheel_radius
        right = self.right_sign * right_rpm * RPM_TO_RAD_S * self.wheel_radius
        self.v = (left + right) / 2.0
        self.w = (right - left) / self.wheel_separation
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = _normalize_angle(self.yaw + self.w * dt)


def yaw_to_quaternion(yaw):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
```

주의: 테스트 2)의 yaw 기대값 `w_expected * 2.0`이 pi를 넘지 않아 정규화와 충돌하지 않는다
(약 0.808 rad). 값 바꿀 때 pi 초과 여부 확인.

- [ ] **Step 4: 통과 확인**

Run: `python3 scripts/test_diff_drive_odometry.py`
Expected: `diff_drive_odometry tests passed`

- [ ] **Step 5: 러너 등록 + Commit**

`scripts/run_offline_tests.sh` PY_TESTS에 `scripts/test_diff_drive_odometry.py` 추가 후:

Run: `bash scripts/run_offline_tests.sh` → FAIL 0

```bash
git add src/drive_pkg/drive_pkg/diff_drive_odometry.py scripts/test_diff_drive_odometry.py scripts/run_offline_tests.sh
git commit -m "feat(drive): 차동구동 오도메트리 적분 모듈 (sign 보정 + 닫힌형태 검증 테스트)"
```

---

### Task 4: opencr_bridge 노드 + calib YAML + drive_bringup.launch.py

**Files:**
- Create: `src/drive_pkg/drive_pkg/opencr_bridge_node.py`
- Create: `src/drive_pkg/config/drive_calib.yaml`
- Create: `src/drive_pkg/launch/drive_bringup.launch.py`
- Create: `scripts/test_opencr_bridge_dryrun.py` (ROS 필요 — 호스트에서는 러너가 SKIP)
- Modify: `src/drive_pkg/CMakeLists.txt` (PROGRAMS에 opencr_bridge 추가)
- Modify: `src/drive_pkg/package.xml` (tf2_ros, python3-serial 의존 추가)
- Modify: `docker/Dockerfile` + `docker/Dockerfile.jetson` (`python3-serial` apt 추가 — docker/ lock 주의)
- Modify: `scripts/run_offline_tests.sh` (PY_TESTS 추가)

**Interfaces:**
- Consumes: Task 2 `opencr_protocol.*`, Task 3 `DiffDriveOdometry`/`yaw_to_quaternion`
- Produces:
  - 노드 `packagu_opencr_bridge`: sub `/cmd_vel`(Twist), pub `/odom`(Odometry)·`/imu`(Imu), TF `odom→base_footprint`
  - 파라미터: `serial_port`(기본 `/dev/opencr`), `baudrate`(115200), `wheel_radius`(0.033), `wheel_separation`(0.51324), `left_sign`(1.0), `right_sign`(1.0), `cmd_timeout_sec`(0.5), `cmd_rate_hz`(20.0), `odom_frame`("odom"), `base_frame`("base_footprint"), `imu_frame`("base_link"), `publish_tf`(True)
  - 클래스 `OpencrBridgeNode(transport=None)` — transport 주입 가능(테스트), None이면 pyserial 오픈
  - transport 계약: `.readline() -> bytes`(없으면 `b""`), `.write(data: bytes) -> None`
  - 테스트 관측용 속성: `.last_odom_msg`, `.last_imu_msg`, `.last_command_bytes`

- [ ] **Step 1: 실패하는 dry-run 테스트 작성** — `scripts/test_opencr_bridge_dryrun.py`

```python
#!/usr/bin/env python3
"""브리지 노드 dry-run: FakeSerial 주입, ROS 필요 (호스트에서는 러너가 SKIP).

컨테이너 실행:
  ROS_DOMAIN_ID=89 python3 scripts/test_opencr_bridge_dryrun.py
"""
import math
import sys
from pathlib import Path

import rclpy  # 호스트에 없으면 ModuleNotFoundError -> 러너 SKIP 규약

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "drive_pkg"))

from drive_pkg.opencr_bridge_node import OpencrBridgeNode  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402


class FakeSerial:
    def __init__(self, lines=None):
        self.lines = list(lines or [])
        self.written = []

    def readline(self):
        if self.lines:
            return self.lines.pop(0)
        return b""

    def write(self, data):
        self.written.append(bytes(data))


def approx(a, b, tol, label):
    if abs(a - b) > tol:
        raise AssertionError(f"{label}: expected {b}, got {a}")


def main():
    rclpy.init()
    fake = FakeSerial([
        b"HELLO opencr 0.1\n",                                 # 무시되어야 함
        b"F 60.0 60.0 0.0 0.0 0.0 0.0 0.0 9.81 1.0 0.0 0.0 0.0\n",
    ])
    node = OpencrBridgeNode(transport=fake)
    try:
        # 1) 신선한 cmd_vel -> V 프레임 인코딩 확인
        twist = Twist()
        twist.linear.x = 0.2
        node.on_cmd_vel(twist)
        node.send_command_tick()
        assert node.last_command_bytes.startswith(b"V "), "V frame not sent"
        assert node.last_command_bytes != b"V 0.00 0.00\n", "fresh cmd must not be zeroed"

        # 2) watchdog: cmd_vel 0.6s 경과 시 0 명령
        node.force_last_cmd_age_for_test(0.6)
        node.send_command_tick()
        assert node.last_command_bytes == b"V 0.00 0.00\n", "watchdog zero command missing"

        # 3) 피드백 처리: 첫 tick이 큐를 소진(HELLO 무시 + F 적분 1.0s), 둘째 tick은 no-op
        node.poll_feedback_tick(dt_override=1.0)
        node.poll_feedback_tick(dt_override=1.0)
        v_expected = 60.0 * 2.0 * math.pi / 60.0 * 0.033
        assert node.last_odom_msg is not None, "odom not published"
        approx(node.last_odom_msg.pose.pose.position.x, v_expected, 1e-6, "odom x")
        approx(node.last_odom_msg.twist.twist.linear.x, v_expected, 1e-6, "odom v")
        assert node.last_odom_msg.header.frame_id == "odom"
        assert node.last_odom_msg.child_frame_id == "base_footprint"
        assert node.last_imu_msg is not None, "imu not published"
        approx(node.last_imu_msg.linear_acceleration.z, 9.81, 1e-6, "imu az")
        print("opencr_bridge dry-run tests passed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 실패 확인 (컨테이너)**

Run: `docker exec -it ros2_humble bash -lc "cd /ros2_ws && ROS_DOMAIN_ID=89 python3 scripts/test_opencr_bridge_dryrun.py"`
Expected: FAIL — `ModuleNotFoundError: drive_pkg.opencr_bridge_node`
(호스트에서 실행하면 rclpy 없음 → 러너 SKIP 동작도 겸사 확인)

- [ ] **Step 3: 노드 구현** — `src/drive_pkg/drive_pkg/opencr_bridge_node.py`

```python
#!/usr/bin/env python3
"""OpenCR 시리얼 브리지: /cmd_vel -> V 프레임, F 피드백 -> /odom + TF + /imu.

프로토콜: docs/deployment/02_opencr_serial_protocol.md (v0.1)
테스트: scripts/test_opencr_bridge_dryrun.py (FakeSerial 주입)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from drive_pkg.opencr_protocol import (
    encode_velocity_command,
    parse_feedback_line,
    twist_to_wheel_rpm,
)
from drive_pkg.diff_drive_odometry import DiffDriveOdometry, yaw_to_quaternion

MAX_LINES_PER_TICK = 20


class OpencrBridgeNode(Node):
    def __init__(self, transport=None):
        super().__init__("packagu_opencr_bridge")
        self.declare_parameter("serial_port", "/dev/opencr")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.51324)
        self.declare_parameter("left_sign", 1.0)
        self.declare_parameter("right_sign", 1.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        p = self.get_parameter
        self.wheel_radius = p("wheel_radius").value
        self.wheel_separation = p("wheel_separation").value
        self.left_sign = p("left_sign").value
        self.right_sign = p("right_sign").value
        self.cmd_timeout = p("cmd_timeout_sec").value
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.imu_frame = p("imu_frame").value
        self.publish_tf = p("publish_tf").value

        self.transport = transport if transport is not None else self._open_serial()
        self.odometry = DiffDriveOdometry(
            self.wheel_radius, self.wheel_separation, self.left_sign, self.right_sign
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._last_cmd_time = None
        self._last_feedback_time = None
        self.last_command_bytes = b""
        self.last_odom_msg = None
        self.last_imu_msg = None

        cmd_period = 1.0 / p("cmd_rate_hz").value
        self.create_timer(cmd_period, self.send_command_tick)
        self.create_timer(0.01, self.poll_feedback_tick)  # 100Hz 폴링

    def _open_serial(self):
        import serial  # 지연 import: 오프라인 테스트는 transport 주입으로 우회

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baudrate").value
        self.get_logger().info(f"opening serial {port} @ {baud}")
        return serial.Serial(port, baud, timeout=0.0)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cmd_vel(self, msg):
        self._cmd_v = msg.linear.x
        self._cmd_w = msg.angular.z
        self._last_cmd_time = self._now_sec()

    def force_last_cmd_age_for_test(self, age_sec):
        self._last_cmd_time = self._now_sec() - age_sec

    def send_command_tick(self):
        stale = (
            self._last_cmd_time is None
            or (self._now_sec() - self._last_cmd_time) > self.cmd_timeout
        )
        if stale:
            left_rpm, right_rpm = 0.0, 0.0
        else:
            left_rpm, right_rpm = twist_to_wheel_rpm(
                self._cmd_v, self._cmd_w, self.wheel_radius, self.wheel_separation
            )
            left_rpm *= self.left_sign
            right_rpm *= self.right_sign
        self.last_command_bytes = encode_velocity_command(left_rpm, right_rpm)
        self.transport.write(self.last_command_bytes)

    def poll_feedback_tick(self, dt_override=None):
        for _ in range(MAX_LINES_PER_TICK):
            raw = self.transport.readline()
            if not raw:
                return
            feedback = parse_feedback_line(raw.decode("ascii", errors="replace"))
            if feedback is None:
                self.get_logger().debug(f"ignored line: {raw!r}")
                continue
            now = self._now_sec()
            if dt_override is not None:
                dt = dt_override
            elif self._last_feedback_time is None:
                dt = 0.0
            else:
                dt = now - self._last_feedback_time
            self._last_feedback_time = now
            self.odometry.update(feedback["left_rpm"], feedback["right_rpm"], dt)
            self._publish_odom()
            self._publish_imu(feedback)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.odometry.x
        msg.pose.pose.position.y = self.odometry.y
        qx, qy, qz, qw = yaw_to_quaternion(self.odometry.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.odometry.v
        msg.twist.twist.angular.z = self.odometry.w
        self.odom_pub.publish(msg)
        self.last_odom_msg = msg
        if self.publish_tf:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odometry.x
            tf.transform.translation.y = self.odometry.y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def _publish_imu(self, feedback):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation.w = feedback["quat"][0]
        msg.orientation.x = feedback["quat"][1]
        msg.orientation.y = feedback["quat"][2]
        msg.orientation.z = feedback["quat"][3]
        msg.angular_velocity.x = feedback["gyro"][0]
        msg.angular_velocity.y = feedback["gyro"][1]
        msg.angular_velocity.z = feedback["gyro"][2]
        msg.linear_acceleration.x = feedback["accel"][0]
        msg.linear_acceleration.y = feedback["accel"][1]
        msg.linear_acceleration.z = feedback["accel"][2]
        self.imu_pub.publish(msg)
        self.last_imu_msg = msg

    def send_zero_command_safe(self):
        """종료 경로 전용 — 실패해도 진행 (시리얼이 이미 닫혔을 수 있음)."""
        try:
            self.transport.write(encode_velocity_command(0.0, 0.0))
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = OpencrBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_command_safe()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: calib YAML** — `src/drive_pkg/config/drive_calib.yaml`

```yaml
# R1B 지면 캘리브레이션에서 확정 후 갱신 (스펙 §5.5)
# 초기값 근거: docs/hardware_spec.md §1.3 (바퀴 지름 0.066m, 간격 0.51324m)
packagu_opencr_bridge:
  ros__parameters:
    wheel_radius: 0.033
    wheel_separation: 0.51324
    left_sign: 1.0
    right_sign: 1.0
    cmd_timeout_sec: 0.5
    cmd_rate_hz: 20.0
```

- [ ] **Step 5: bringup launch** — `src/drive_pkg/launch/drive_bringup.launch.py`

```python
"""OpenCR 브리지 bringup. 시리얼 포트는 launch 인자 (이식성 R3)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("drive_pkg")
    calib = os.path.join(pkg_share, "config", "drive_calib.yaml")
    serial_port = LaunchConfiguration("serial_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/opencr",
            description="OpenCR 시리얼 포트 (udev 별칭 권장)",
        ),
        Node(
            package="drive_pkg",
            executable="opencr_bridge",
            name="packagu_opencr_bridge",
            output="screen",
            parameters=[calib, {"serial_port": serial_port}],
        ),
    ])
```

- [ ] **Step 6: 설치/의존 반영**

(a) `src/drive_pkg/CMakeLists.txt` — 기존 keyboard_teleop `install(PROGRAMS ...)` 블록 아래에 추가:

```cmake
install(PROGRAMS
  drive_pkg/opencr_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME opencr_bridge
)
```

(b) `src/drive_pkg/package.xml` — `<depend>sensor_msgs</depend>` 아래에 추가:

```xml
  <depend>tf2_ros</depend>
  <exec_depend>python3-serial</exec_depend>
```

(c) `docker/Dockerfile`·`docker/Dockerfile.jetson` apt 목록에 `python3-serial` 추가
(두 파일 동시 — 공통 패키지 목록 일치 원칙, TODO.md lock)

- [ ] **Step 7: 컨테이너 빌드 + 테스트 통과 확인**

Run:
```bash
docker exec -it ros2_humble bash -lc "cd /ros2_ws && colcon build --symlink-install --base-paths src --packages-select drive_pkg && source install/setup.bash && ROS_DOMAIN_ID=89 python3 scripts/test_opencr_bridge_dryrun.py"
```
Expected: `opencr_bridge dry-run tests passed`

- [ ] **Step 8: 러너 등록 + Commit**

`scripts/run_offline_tests.sh` PY_TESTS에 `scripts/test_opencr_bridge_dryrun.py` 추가 (호스트=SKIP, 컨테이너=PASS).

Run: `bash scripts/run_offline_tests.sh` → `SKIP scripts/test_opencr_bridge_dryrun.py (ROS 모듈 없음)` + FAIL 0
Run: `python3 scripts/check_portability.py` → PASS

```bash
git add src/drive_pkg docker/Dockerfile docker/Dockerfile.jetson scripts/test_opencr_bridge_dryrun.py scripts/run_offline_tests.sh
git commit -m "feat(drive): OpenCR 브리지 노드 — /cmd_vel->V, F->/odom+TF+/imu, 0.5s watchdog, calib YAML"
```

---

### Task 5: keyboard_teleop dead-man (TDD)

**Files:**
- Modify: `src/drive_pkg/drive_pkg/keyboard_teleop.py`
- Modify: `scripts/test_wasd_teleop.py` (케이스 추가)

**Interfaces:**
- Produces: `apply_deadman(command: tuple, idle_seconds: float, timeout: float = 0.5) -> tuple[tuple, bool]` — (적용된 command, 강제정지 여부)

- [ ] **Step 1: 실패하는 테스트 추가** — `scripts/test_wasd_teleop.py`의 `main()` 마지막(기존 assert들 뒤)에 추가

```python
    # dead-man: 0.5s 키 미수신 시 강제 정지
    cmd, stopped = teleop.apply_deadman((0.25, 0.0), idle_seconds=0.6)
    assert_tuple(cmd, (0.0, 0.0))
    if not stopped:
        raise AssertionError("deadman must report stop")

    cmd, stopped = teleop.apply_deadman((0.25, 0.0), idle_seconds=0.3)
    assert_tuple(cmd, (0.25, 0.0))
    if stopped:
        raise AssertionError("fresh command must not be stopped")

    cmd, stopped = teleop.apply_deadman((0.0, 0.0), idle_seconds=9.9)
    assert_tuple(cmd, (0.0, 0.0))
    if stopped:
        raise AssertionError("zero command must not report stop")
```

- [ ] **Step 2: 실패 확인**

Run: `python3 scripts/test_wasd_teleop.py`
Expected: FAIL — `AttributeError: apply_deadman`

- [ ] **Step 3: 구현** — `keyboard_teleop.py`

(a) 상수/함수 추가 (`MOVE_BINDINGS` 아래):

```python
DEADMAN_TIMEOUT_SEC = 0.5


def apply_deadman(command, idle_seconds, timeout=DEADMAN_TIMEOUT_SEC):
    """마지막 키 입력 후 timeout 초과 시 정지 명령으로 대체.

    latch 방식(키 안 눌러도 마지막 명령 유지)의 안전장치 — SSH 끊김/키 미수신 시
    로봇이 계속 달리는 것을 방지 (스펙 §5.4 teleop 안전화).
    """
    linear_x, angular_z = command
    moving = linear_x != 0.0 or angular_z != 0.0
    if moving and idle_seconds > timeout:
        return (0.0, 0.0), True
    return command, False
```

(b) `main()` 루프 수정 — `import time` 추가 후, 기존 루프의 `publish_twist` 직전에 적용:

```python
    last_key_time = time.monotonic()
    try:
        while rclpy.ok():
            key = read_key(settings)
            if key == "\x03":
                break
            if key:
                last_key_time = time.monotonic()
            current_binding, linear_speed, angular_speed, command, changed_label = process_key(
                key, current_binding, linear_speed, angular_speed,
            )
            command, deadman_stopped = apply_deadman(
                command, time.monotonic() - last_key_time,
            )
            if deadman_stopped:
                current_binding = (0.0, 0.0)
                print("deadman stop: no key for "
                      f"{DEADMAN_TIMEOUT_SEC:.1f}s -> cmd_vel=(0.00, 0.00)")
            linear_x, angular_z = command
```

(이하 기존 출력/publish 코드는 그대로 — `linear_x, angular_z = command` 줄이 기존
`command` 언패킹을 대체한다.)

- [ ] **Step 4: 통과 확인**

Run: `python3 scripts/test_wasd_teleop.py`
Expected: 기존 + 신규 케이스 전부 통과

- [ ] **Step 5: Commit**

```bash
git add src/drive_pkg/drive_pkg/keyboard_teleop.py scripts/test_wasd_teleop.py
git commit -m "fix(drive): teleop dead-man — 0.5s 키 미수신 시 자동 정지 (latch 주행 방지)"
```

---

### Task 6: slam_toolbox.launch.py 리팩터링 (RSP/RViz 조건부/드라이브 include)

**Files:**
- Modify: `src/slam_pkg/launch/slam_toolbox.launch.py` (전체 교체)
- Modify: `src/slam_pkg/launch/kku_simulation.launch.py:52-54` (rviz 인자 추가)
- Modify: `src/slam_pkg/package.xml` (`<exec_depend>robot_state_publisher</exec_depend>`, `<exec_depend>xacro</exec_depend>` 추가)
- Create: `scripts/test_field_mapping_launch.py`
- Modify: `scripts/run_offline_tests.sh`

**Interfaces:**
- Consumes: Task 4의 `drive_bringup.launch.py`
- Produces: launch 인자 계약 — `use_sim_time`(false), `serial_port`(`/dev/rplidar`), `rviz`(false), `enable_lidar`(true), `enable_drive`(false)

- [ ] **Step 1: 실패하는 계약 테스트 작성** — `scripts/test_field_mapping_launch.py` (기존 `test_kku_navigation_launch.py` 패턴)

```python
#!/usr/bin/env python3
"""실기 매핑 launch 계약 검사 (스펙 §5.4 '매핑 launch 정비')."""
from pathlib import Path
import ast
import sys

ROOT = Path(__file__).resolve().parents[1]
SLAM_LAUNCH = ROOT / "src/slam_pkg/launch/slam_toolbox.launch.py"
SIM_LAUNCH = ROOT / "src/slam_pkg/launch/kku_simulation.launch.py"
DRIVE_LAUNCH = ROOT / "src/drive_pkg/launch/drive_bringup.launch.py"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def main():
    for path in (SLAM_LAUNCH, SIM_LAUNCH, DRIVE_LAUNCH):
        require(path.exists(), f"missing {path}")
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    slam_src = SLAM_LAUNCH.read_text(encoding="utf-8")
    for needle in [
        "robot_state_publisher",          # 실기 TF 체인 (RSP)
        "delivery_robot.urdf.xacro",      # URDF 로드
        "UnlessCondition",                # 시뮬에서는 RSP/LiDAR 중복 기동 금지
        '"/dev/rplidar"',                 # udev 별칭 기본값
        '"rviz"',                         # RViz 조건부 인자
        '"enable_lidar"',
        '"enable_drive"',
        "drive_bringup.launch.py",        # 드라이브 include
    ]:
        require(needle in slam_src, f"slam_toolbox.launch.py missing: {needle}")
    require('default_value="false"' in slam_src, "rviz must default to false")

    sim_src = SIM_LAUNCH.read_text(encoding="utf-8")
    require('"rviz": "true"' in sim_src, "sim must keep RViz on (regression guard)")

    drive_src = DRIVE_LAUNCH.read_text(encoding="utf-8")
    require("DeclareLaunchArgument" in drive_src and '"serial_port"' in drive_src,
            "drive serial port must be a launch argument (portability R3)")

    print("field mapping launch contract passed")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 실패 확인**

Run: `python3 scripts/test_field_mapping_launch.py`
Expected: FAIL — `slam_toolbox.launch.py missing: robot_state_publisher`

- [ ] **Step 3: launch 전체 교체** — `src/slam_pkg/launch/slam_toolbox.launch.py`

```python
"""SLAM Toolbox launch (시뮬레이션 + 실제 하드웨어 공용).

시뮬 (kku_simulation.launch.py가 include — RSP/LiDAR는 Gazebo 쪽이 담당):
  ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true rviz:=true

실기 매핑 (Jetson — RViz 없음, RSP+LiDAR+드라이브 포함):
  ros2 launch slam_pkg slam_toolbox.launch.py \
    use_sim_time:=false enable_drive:=true serial_port:=/dev/rplidar
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("slam_pkg")
    pkg_common = get_package_share_directory("common_pkg")
    pkg_drive = get_package_share_directory("drive_pkg")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    urdf_file = os.path.join(pkg_common, "urdf", "delivery_robot.urdf.xacro")
    robot_desc = xacro.process_file(urdf_file).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    rviz = LaunchConfiguration("rviz")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_drive = LaunchConfiguration("enable_drive")

    real_lidar = PythonExpression(
        ["'", use_sim_time, "' == 'false' and '", enable_lidar, "' == 'true'"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Gazebo: true, 실기: false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLiDAR 포트 (udev 별칭 권장)"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="RViz 실행 여부 (Jetson 이미지에는 rviz2 없음)"),
        DeclareLaunchArgument("enable_lidar", default_value="true",
                              description="실기 LiDAR 드라이버 기동 여부 (dry-run: false)"),
        DeclareLaunchArgument("enable_drive", default_value="false",
                              description="OpenCR 브리지 bringup 포함 여부"),

        # 실기 TF 체인: base_footprint -> ... -> laser (시뮬은 Gazebo 쪽 RSP가 담당)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            condition=UnlessCondition(use_sim_time),
            parameters=[{"robot_description": robot_desc,
                         "use_sim_time": use_sim_time}],
        ),

        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar",
            condition=IfCondition(real_lidar),
            parameters=[{
                "serial_port": serial_port,
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "angle_compensate": True,
                "scan_mode": "Standard",
                "use_sim_time": use_sim_time,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_drive, "launch", "drive_bringup.launch.py")
            ),
            condition=IfCondition(enable_drive),
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(rviz),
            arguments=["-d", os.path.join(pkg_share, "config", "slam_view.rviz")]
            if os.path.exists(os.path.join(pkg_share, "config", "slam_view.rviz"))
            else [],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
```

- [ ] **Step 4: 시뮬 회귀 방지** — `kku_simulation.launch.py`의 slam_toolbox include 인자를 다음으로 교체

```python
            launch_arguments={
                "use_sim_time": "true",
                "rviz": "true",
            }.items(),
```

- [ ] **Step 5: package.xml 의존 추가** — `src/slam_pkg/package.xml`에 `<exec_depend>robot_state_publisher</exec_depend>`, `<exec_depend>xacro</exec_depend>`

- [ ] **Step 6: 통과 확인 + 시뮬 스모크**

Run: `python3 scripts/test_field_mapping_launch.py` → `field mapping launch contract passed`
Run: `python3 scripts/test_kku_navigation_launch.py` → 기존 계약 유지 확인
Run (컨테이너, 시뮬 회귀 — RViz 창 + `/map` 갱신 육안 확인):
```bash
./scripts/run_kku_sim.sh F1
```
Expected: 기존과 동일하게 Gazebo+SLAM+RViz 기동 (RSP 중복 기동 없음 — `ros2 node list`에 robot_state_publisher 1개)

- [ ] **Step 7: 러너 등록 + Commit**

```bash
git add src/slam_pkg/launch/ src/slam_pkg/package.xml scripts/test_field_mapping_launch.py scripts/run_offline_tests.sh
git commit -m "feat(slam): 실기 매핑 launch — RSP/LiDAR 조건부, rviz 기본 off, drive include (시뮬 회귀 가드 포함)"
```

---

### Task 7: 실맵 저장 — save_kku_map.sh `--real` 모드 + posegraph

**Files:**
- Modify: `scripts/save_kku_map.sh` (전체 교체)
- Create: `scripts/test_field_scripts_contract.py`
- Modify: `scripts/run_offline_tests.sh`

**Interfaces:**
- Produces: `./scripts/save_kku_map.sh F1 --real` → `/ros2_ws/maps/kku_real/f1/kku_f1_real.{yaml,pgm,posegraph,data}`

- [ ] **Step 1: 실패하는 계약 테스트 작성** — `scripts/test_field_scripts_contract.py`

```python
#!/usr/bin/env python3
"""실맵 저장/field 스크립트 계약 검사 (스펙 §5.4 '실맵 저장', 'field 원커맨드')."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SAVE = ROOT / "scripts/save_kku_map.sh"
FIELD = ROOT / "scripts/run_field_mapping.sh"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def bash_syntax_ok(path):
    return subprocess.run(["bash", "-n", str(path)], capture_output=True).returncode == 0


def main():
    require(SAVE.exists(), "missing save_kku_map.sh")
    require(bash_syntax_ok(SAVE), "save_kku_map.sh syntax error")
    save_src = SAVE.read_text(encoding="utf-8")
    for needle in [
        "--real",                      # 실맵 모드 플래그
        "kku_real",                    # 실맵 저장 루트
        "_real",                       # kku_f?_real 네이밍
        "serialize_map",               # posegraph 저장 (slam_toolbox 서비스)
        "map_saver_cli",               # 기존 pgm/yaml 저장 유지
    ]:
        require(needle in save_src, f"save_kku_map.sh missing: {needle}")
    require("kku_virtual" in save_src, "virtual map path must remain (regression)")

    require(FIELD.exists(), "missing run_field_mapping.sh")
    require(bash_syntax_ok(FIELD), "run_field_mapping.sh syntax error")
    field_src = FIELD.read_text(encoding="utf-8")
    for needle in [
        "set -euo pipefail",
        "ros2 bag record",
        "/scan", "/odom", "/imu", "/cmd_vel", "/tf", "/tf_static",
        "enable_drive:=true",
        "rviz:=",                      # 관찰 옵션 env 게이트 (E2E 정책)
        "use_sim_time:=false",
        "save_kku_map.sh",             # 저장 경로 단일화 (--real 재사용)
        "trap",                        # cleanup 보장
    ]:
        require(needle in field_src, f"run_field_mapping.sh missing: {needle}")

    print("field scripts contract passed")


if __name__ == "__main__":
    main()
```

주: `run_field_mapping.sh`는 Task 8에서 생성 — 이 시점 Expected는 `missing run_field_mapping.sh` FAIL. Task 7에서는 save 부분만 통과시키고, 테스트 러너 등록은 Task 8 완료 후에 한다.

- [ ] **Step 2: 실패 확인**

Run: `python3 scripts/test_field_scripts_contract.py`
Expected: FAIL — `save_kku_map.sh missing: --real`

- [ ] **Step 3: 스크립트 교체** — `scripts/save_kku_map.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${1:-F1}"
MODE="${2:-}"

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/save_kku_map.sh [F1|F2|F3]          # 시뮬 가상맵 저장 (기존 동작)
  ./scripts/save_kku_map.sh [F1|F2|F3] --real   # 실측 맵 + posegraph 저장

Saves to:
  virtual: /ros2_ws/maps/kku_virtual/f1/kku_f1        (.yaml/.pgm)
  real:    /ros2_ws/maps/kku_real/f1/kku_f1_real      (.yaml/.pgm/.posegraph/.data)
USAGE
}

case "${FLOOR}" in
  F1) FLOOR_DIR="f1" ;;
  F2) FLOOR_DIR="f2" ;;
  F3) FLOOR_DIR="f3" ;;
  -h|--help) usage; exit 0 ;;
  *) echo "error: floor must be one of F1, F2, F3; got '${FLOOR}'" >&2; usage >&2; exit 2 ;;
esac

REAL=0
if [[ "${MODE}" == "--real" ]]; then
  REAL=1
elif [[ -n "${MODE}" ]]; then
  echo "error: unknown option '${MODE}' (expected --real)" >&2; usage >&2; exit 2
fi

if [[ ${REAL} -eq 1 ]]; then
  OUTPUT_DIR="/ros2_ws/maps/kku_real/${FLOOR_DIR}"
  MAP_NAME="kku_${FLOOR_DIR}_real"
else
  OUTPUT_DIR="/ros2_ws/maps/kku_virtual/${FLOOR_DIR}"
  MAP_NAME="kku_${FLOOR_DIR}"
fi
OUTPUT_PATH="${OUTPUT_DIR}/${MAP_NAME}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "error: ${CONTAINER_NAME} is not running." >&2
  exit 1
fi

docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && mkdir -p '${OUTPUT_DIR}' && ros2 run nav2_map_server map_saver_cli -f '${OUTPUT_PATH}'"

if [[ ${REAL} -eq 1 ]]; then
  # posegraph 직렬화 — 오프라인 재개/재맵핑용 (스펙 §5.4)
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '${OUTPUT_PATH}'}\""
  echo "Saved real map: ${OUTPUT_PATH}.yaml/.pgm + ${OUTPUT_PATH}.posegraph/.data"
else
  echo "Saved map to ${OUTPUT_PATH}.yaml and ${OUTPUT_PATH}.pgm"
fi
```

- [ ] **Step 4: 부분 통과 확인**

Run: `python3 scripts/test_field_scripts_contract.py`
Expected: save 항목 전부 통과 후 `missing run_field_mapping.sh`에서 FAIL (Task 8에서 해소)

- [ ] **Step 5: 기존 스크립트 회귀 확인 + Commit**

Run: `bash scripts/test_kku_sim_scripts.sh`
Expected: PASS (기존 가상맵 경로/인자 회귀 없음)

```bash
git add scripts/save_kku_map.sh scripts/test_field_scripts_contract.py
git commit -m "feat(scripts): save_kku_map --real 모드 — kku_real 경로 + slam_toolbox posegraph 직렬화"
```

---

### Task 8: field 원커맨드 스크립트 (start→launch→rosbag→저장→검증→cleanup)

**Files:**
- Create: `scripts/run_field_mapping.sh`
- Modify: `scripts/run_offline_tests.sh` (`scripts/test_field_scripts_contract.py` 등록)

**Interfaces:**
- Consumes: Task 6 launch 인자 계약, Task 7 `save_kku_map.sh --real`
- Produces: `FLOOR=F1 ./scripts/run_field_mapping.sh` — Ctrl+C 시 맵 저장+검증+정리까지 자동

- [ ] **Step 1: 스크립트 작성** — `scripts/run_field_mapping.sh`

```bash
#!/usr/bin/env bash
# 현장 실측 원커맨드 (스펙 §5.4, AGENTS E2E 정책).
# 흐름: 컨테이너 확인 -> 실기 매핑 launch -> 토픽 대기 -> rosbag 기록(전경)
#       -> Ctrl+C -> 맵+posegraph 저장 -> 산출물 검증 -> cleanup
# 사용:
#   FLOOR=F1 ./scripts/run_field_mapping.sh
#   WITH_RVIZ=1  : (데스크톱 검증용) RViz 켜기 — Jetson에서는 0 유지
#   BAG=0        : rosbag 생략 (권장 안 함 — bag은 재SLAM 보험)
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
FLOOR="${FLOOR:-F1}"
WITH_RVIZ="${WITH_RVIZ:-0}"
BAG="${BAG:-1}"
TS="$(date +%Y-%m-%d_%H-%M-%S)"
BAG_DIR="/ros2_ws/logs/field_${TS}"
RVIZ_ARG="false"
[[ "${WITH_RVIZ}" == "1" ]] && RVIZ_ARG="true"

case "${FLOOR}" in F1|F2|F3) ;; *) echo "error: FLOOR must be F1|F2|F3" >&2; exit 2 ;; esac

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "error: ${CONTAINER_NAME} is not running (docker compose -f docker/compose/docker-compose.jetson.yml up -d)" >&2
  exit 1
fi

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}

cleanup() {
  echo "[field] cleanup: stopping launch/bag processes in container"
  docker exec -i "${CONTAINER_NAME}" bash -lc \
    "pkill -INT -f 'ros2 bag record' 2>/dev/null; pkill -INT -f slam_toolbox 2>/dev/null; pkill -INT -f rplidar 2>/dev/null; pkill -INT -f opencr_bridge 2>/dev/null; pkill -INT -f robot_state_publisher 2>/dev/null" || true
}
trap cleanup EXIT

echo "[field] launching mapping stack (floor=${FLOOR}, rviz=${RVIZ_ARG})"
docker exec -d -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=false enable_drive:=true rviz:=${RVIZ_ARG} serial_port:=/dev/rplidar"

echo "[field] waiting for /scan and /odom (max 60s)"
for _ in $(seq 1 30); do
  if in_container "ros2 topic list" | grep -q "^/scan$" \
     && in_container "ros2 topic list" | grep -q "^/odom$"; then
    READY=1; break
  fi
  sleep 2
done
if [[ "${READY:-0}" != "1" ]]; then
  echo "error: /scan or /odom not available — 철수 기준 확인 (스펙 §6.4)" >&2
  exit 1
fi

# Ctrl+C는 전경 자식(bag record)만 멈추고 스크립트는 저장 단계로 진행해야 한다.
# trap 없이는 bash가 SIGINT로 함께 종료되어 맵 저장을 건너뛴다 (bash WCE 규칙).
trap ':' INT
if [[ "${BAG}" == "1" ]]; then
  echo "[field] recording rosbag to ${BAG_DIR} — 맵핑 종료 시 Ctrl+C"
  in_container "mkdir -p '${BAG_DIR}' && ros2 bag record -o '${BAG_DIR}/bag' /scan /scan_raw /odom /imu /cmd_vel /tf /tf_static" || true
else
  echo "[field] BAG=0 — 기록 없이 대기. 맵핑 종료 시 Ctrl+C"
  sleep infinity || true
fi
trap - INT

echo "[field] saving real map for ${FLOOR}"
"$(dirname "$0")/save_kku_map.sh" "${FLOOR}" --real

FLOOR_DIR="$(tr '[:upper:]' '[:lower:]' <<<"${FLOOR}")"
MAP_BASE="/ros2_ws/maps/kku_real/${FLOOR_DIR}/kku_${FLOOR_DIR}_real"
echo "[field] verifying artifacts"
in_container "test -f '${MAP_BASE}.yaml' && test -f '${MAP_BASE}.pgm' && test -f '${MAP_BASE}.posegraph'"
echo "[field] OK: ${MAP_BASE}.{yaml,pgm,posegraph}"
[[ "${BAG}" == "1" ]] && echo "[field] bag: ${BAG_DIR} (로컬 전용 — 외부 업로드 금지)"
```

- [ ] **Step 2: 계약 테스트 전체 통과 확인**

Run: `python3 scripts/test_field_scripts_contract.py`
Expected: `field scripts contract passed`

- [ ] **Step 3: 실행권한 + 러너 등록**

```bash
chmod +x scripts/run_field_mapping.sh
```
`scripts/run_offline_tests.sh` PY_TESTS에 `scripts/test_field_scripts_contract.py` 추가.

Run: `bash scripts/run_offline_tests.sh` → FAIL 0

- [ ] **Step 4: 데스크톱 부분 검증 (센서 없이 동작 경로만)**

Run: `FLOOR=F1 BAG=0 ./scripts/run_field_mapping.sh`
Expected: launch 기동 → `/odom` 미발행(하드웨어 없음)으로 60s 후 "철수 기준" 에러 exit 1 + cleanup 동작 (에러 경로 검증이 목적)

- [ ] **Step 5: Commit**

```bash
git add scripts/run_field_mapping.sh scripts/run_offline_tests.sh
git commit -m "feat(scripts): 현장 실측 원커맨드 — launch->rosbag->실맵/posegraph 저장->검증->cleanup"
```

---

### Task 9: udev 규칙 + 설치 스크립트

**Files:**
- Create: `scripts/udev/99-packagu-devices.rules`
- Create: `scripts/udev/install_udev_rules.sh`
- Create: `scripts/test_udev_contract.py`
- Modify: `scripts/run_offline_tests.sh`

**Interfaces:**
- Produces: Jetson에서 `/dev/rplidar`, `/dev/opencr` 고정 별칭 (+ 벤치에서 확정할 motor_nano/arm_servo 템플릿)

- [ ] **Step 1: 규칙 파일 작성** — `scripts/udev/99-packagu-devices.rules`

```text
# PackagU 장치 고정 별칭 (스펙 §5.4). 설치: sudo bash scripts/udev/install_udev_rules.sh
# 확인: udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'

# RPLiDAR A1m8 — Silicon Labs CP210x USB-UART
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"

# OpenCR 1.0 — STM32 Virtual COM (ROBOTIS)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="opencr", MODE="0666"

# TODO(R1A 벤치에서 idVendor/idProduct 실측 후 주석 해제 — 스펙 §5.4):
# Arduino Nano (모터 제어 보조 — hardware_spec §1.1 표기 준수)
# SUBSYSTEM=="tty", ATTRS{idVendor}=="____", ATTRS{idProduct}=="____", SYMLINK+="motor_nano", MODE="0666"
# 로봇팔 서보 드라이버 HPRO-0098 (ZX Serial Bus Servo, USB-TTL)
# SUBSYSTEM=="tty", ATTRS{idVendor}=="____", ATTRS{idProduct}=="____", SYMLINK+="arm_servo", MODE="0666"
```

(motor_nano/arm_servo의 VID/PID는 실물 연결 없이는 확정 불가 — CH340 클론 등 변형이 많아
추측 기입 금지. R1A 벤치 체크리스트에서 `udevadm info`로 채운다.)

- [ ] **Step 2: 설치 스크립트** — `scripts/udev/install_udev_rules.sh`

```bash
#!/usr/bin/env bash
# Jetson 호스트에서 실행: sudo bash scripts/udev/install_udev_rules.sh
set -euo pipefail

RULES_SRC="$(cd "$(dirname "$0")" && pwd)/99-packagu-devices.rules"
RULES_DST="/etc/udev/rules.d/99-packagu-devices.rules"

if [[ ${EUID} -ne 0 ]]; then
  echo "error: run with sudo" >&2
  exit 1
fi

cp "${RULES_SRC}" "${RULES_DST}"
udevadm control --reload-rules
udevadm trigger
echo "installed ${RULES_DST}. 확인: ls -l /dev/rplidar /dev/opencr"
```

- [ ] **Step 3: 계약 테스트** — `scripts/test_udev_contract.py`

```python
#!/usr/bin/env python3
"""udev 규칙/설치 스크립트 계약 검사."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
RULES = ROOT / "scripts/udev/99-packagu-devices.rules"
INSTALL = ROOT / "scripts/udev/install_udev_rules.sh"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def main():
    require(RULES.exists(), "missing udev rules file")
    src = RULES.read_text(encoding="utf-8")
    for needle in ['SYMLINK+="rplidar"', 'SYMLINK+="opencr"', "motor_nano", "arm_servo"]:
        require(needle in src, f"rules missing: {needle}")
    active = [l for l in src.splitlines() if l.strip() and not l.strip().startswith("#")]
    require(all('SYMLINK+="' in l for l in active), "active rule lines must define SYMLINK")
    require(not any("____" in l for l in active), "placeholder VID/PID must stay commented")

    require(INSTALL.exists(), "missing install script")
    rc = subprocess.run(["bash", "-n", str(INSTALL)], capture_output=True).returncode
    require(rc == 0, "install script syntax error")
    install_src = INSTALL.read_text(encoding="utf-8")
    for needle in ["udevadm control --reload-rules", "udevadm trigger", "/etc/udev/rules.d/"]:
        require(needle in install_src, f"install script missing: {needle}")

    print("udev contract passed")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 통과 확인 + 러너 등록 + Commit**

Run: `python3 scripts/test_udev_contract.py` → `udev contract passed`
`run_offline_tests.sh` PY_TESTS에 `scripts/test_udev_contract.py` 추가 → `bash scripts/run_offline_tests.sh` FAIL 0

```bash
chmod +x scripts/udev/install_udev_rules.sh
git add scripts/udev/ scripts/test_udev_contract.py scripts/run_offline_tests.sh
git commit -m "feat(scripts): Jetson udev 고정 별칭 — rplidar/opencr + 벤치 확정용 motor_nano/arm_servo 템플릿"
```

---

### Task 10: docker-compose.jetson.yml 권한 축소 + devices + logs 마운트

**Files:**
- Modify: `docker/compose/docker-compose.jetson.yml` (전체 교체 — docker/ lock 주의)

**Interfaces:**
- Consumes: Task 9 udev 별칭
- Produces: 컨테이너 안에서 `/dev/rplidar`, `/dev/opencr` 접근 + `chrt`(SYS_NICE) 가능, `/ros2_ws/logs` 영속화

- [ ] **Step 1: compose 교체**

```yaml
# Jetson Xavier NX 배포용 compose (GUI 없음, aarch64 이미지)
# 사용: docker compose -f docker/compose/docker-compose.jetson.yml up -d
# 사전: (1) GHCR 로그인(read-only PAT) (2) sudo bash scripts/udev/install_udev_rules.sh
services:
  ros2:
    image: ghcr.io/packagu/ros2-humble-slam:humble-jetson
    container_name: ros2_humble
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      - RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
    volumes:
      - ${PACKAGU_SRC:-../../src}:/ros2_ws/src:rw
      - ${PACKAGU_MAPS:-../../src/slam_pkg/maps}:/ros2_ws/maps:rw
      - ${PACKAGU_TEST_WORKSPACE:-../../test_workspace}:/ros2_ws/test_workspace:rw
      - ${PACKAGU_SCRIPTS:-../../scripts}:/ros2_ws/scripts:ro
      - ${PACKAGU_LOGS:-../../logs}:/ros2_ws/logs:rw   # rosbag 영속화 (스펙 §6.3)
    network_mode: host
    # privileged 대신 최소 권한: 시리얼은 devices로, RT(chrt/SCHED_RR)는 SYS_NICE로.
    # udev 별칭이 컨테이너에서 안 보이면 B-1 체크리스트의 'ls -l /dev/rplidar' 절차 참조.
    # (문제 지속 시 임시 복구: privileged: true 재활성 — 사유를 세션 일지에 기록)
    devices:
      - /dev/rplidar:/dev/rplidar     # RPLiDAR A1m8
      - /dev/opencr:/dev/opencr       # OpenCR 1.0
      # - /dev/arm_servo:/dev/arm_servo   # R1A 벤치에서 udev 확정 후 해제
      # - /dev/motor_nano:/dev/motor_nano
    cap_add:
      - SYS_NICE
    stdin_open: true
    tty: true
    restart: unless-stopped
```

- [ ] **Step 2: 문법 검증**

Run: `docker compose -f docker/compose/docker-compose.jetson.yml config -q`
Expected: 출력 없음(성공). (장치 파일이 데스크톱에 없어도 config 파싱은 통과)

- [ ] **Step 3: Commit**

```bash
git add docker/compose/docker-compose.jetson.yml
git commit -m "fix(docker): jetson compose 최소 권한화 — privileged 제거, devices+SYS_NICE, logs 마운트, RMW env"
```

---

### Task 11: Stage 0 이미지 빌드/push + QEMU smoke + 최종 검증

**Files:**
- 없음 (ops + 최종 게이트). digest/commit 기록은 `TODO.md` 세션 메모에.

**Interfaces:**
- Consumes: Task 4의 Dockerfile 변경(python3-serial) 포함된 상태의 `docker/`
- Produces: GHCR `humble-jetson` 태그(arm64) + 기록된 image digest — 스펙 §3 게이트

주의: GHCR push는 외부 서비스 반출 — 실행 전 사용자 확인 필수 (쓰기 권한 PAT 필요).
민감정보 규칙(AGENTS #7~9) 적용: PAT를 파일/로그에 남기지 않는다.

- [ ] **Step 1: 현 이미지 아키텍처 확인**

Run: `docker manifest inspect ghcr.io/packagu/ros2-humble-slam:humble | grep -A2 platform`
Expected: `"architecture": "amd64"`만 존재 (Roadmap 07 체크박스 근거 확보)

- [ ] **Step 2: buildx 크로스 빌드 + push (사용자 승인 후)**

```bash
docker buildx create --use 2>/dev/null || true
docker buildx build --platform linux/arm64 \
  -f docker/Dockerfile.jetson \
  -t ghcr.io/packagu/ros2-humble-slam:humble-jetson \
  --push docker/
```
Expected: `pushing manifest ... done`

- [ ] **Step 3: 빌드 결과 검증 + digest 기록**

Run: `docker manifest inspect ghcr.io/packagu/ros2-humble-slam:humble-jetson | grep -E 'architecture|digest'`
Expected: `"architecture": "arm64"` 포함. digest 값을 `TODO.md` 세션 메모에 기록 (현장 고정용)

- [ ] **Step 4: secret 미포함 확인**

Run: `docker history ghcr.io/packagu/ros2-humble-slam:humble-jetson --no-trunc | grep -iE "token|secret|ghp_" || echo "no secrets in history"`
Expected: `no secrets in history`

- [ ] **Step 5: QEMU smoke (스펙 §3 게이트)**

Run: `docker run --rm --platform linux/arm64 ghcr.io/packagu/ros2-humble-slam:humble-jetson bash -lc "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E 'slam_toolbox|rplidar'"`
Expected:
```text
rplidar_ros
slam_toolbox
```
(qemu binfmt 미설치 시 `docker run --privileged --rm tonistiigi/binfmt --install arm64` 선행)

- [ ] **Step 6: 최종 전체 게이트**

Run: `bash scripts/run_offline_tests.sh`
Expected: 신규 5종(test_opencr_protocol / test_diff_drive_odometry / test_field_mapping_launch / test_field_scripts_contract / test_udev_contract) PASS + test_opencr_bridge_dryrun SKIP(호스트), FAIL 0

Run: `python3 scripts/check_portability.py`
Expected: PASS

Run (컨테이너): `docker exec -it ros2_humble bash -lc "cd /ros2_ws && colcon build --symlink-install --base-paths src && source install/setup.bash && ROS_DOMAIN_ID=89 python3 scripts/test_opencr_bridge_dryrun.py"`
Expected: `opencr_bridge dry-run tests passed`

- [ ] **Step 7: 마무리 Commit + TODO.md 갱신**

`TODO.md`: drive_pkg/docker lock 해제, 세션 메모(digest, 남은 벤치 항목 — udev VID/PID 2종, R1A/R1B는 실기에서) 기록.

```bash
git add TODO.md
git commit -m "chore: Jetson 실측 준비 코드 산출물 완료 — 남은 항목은 실기 벤치(R1A/R1B) 게이트"
```

---

## 계획 범위 밖 (실기/현장 절차 — 스펙이 런북)

다음은 코드가 아니라 실기 절차라 이 계획에 태스크로 넣지 않는다. 스펙 §5(B-0~B-5)와
§6(현장 런북)을 그대로 따른다: Jetson 인벤토리/nvpmodel, 핫스팟 멀티캐스트 검증,
R1A/R1B 게이트, bag 재생 재SLAM 게이트, 현장 실측. udev VID/PID 2종(motor_nano,
arm_servo)과 compose devices 주석 해제도 R1A 벤치에서 수행한다.
OpenCR 펌웨어는 Han 담당 — Task 1 프로토콜 문서가 인터페이스.

외부 입력 대기 항목 (도착 시 별도 마이크로 태스크로 처리):

- Kim 팔 의존성 목록 → `docker/Dockerfile` 양쪽 반영 + 재빌드 (스펙 §3-5, 비블로킹)
- HPRO-0098 연결/전원 사양 확인(Kim) → `docs/hardware_spec.md` 팔 서보 시스템 등재 (스펙 §7)
