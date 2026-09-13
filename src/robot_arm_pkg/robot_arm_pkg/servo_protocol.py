"""Kim 실기 서보 컨트롤러 프로토콜 순수 로직 (ROS 의존 없음 — host 오프라인 테스트 대상).

벤치 확정 스크립트(Kim, 2026-08-17)에서 이식. 명령 포맷:
- 이동:   {#000P1500T2000!#001P...} — PWM 목표까지 T(ms) 동안 컨트롤러가 자체 보간
- 위치:   #000PRAD!  ->  #000P1500!
- 정지:   #000PDPT!
PWM/포즈 값은 사용자가 2026-09-13 제공한 최신 servo_test.py를 그대로 유지한다.
PRAD 응답이 실제 축 엔코더인지 목표값 echo인지는 아직 ⚠️미확인이다.
"""

from __future__ import annotations

import re
import time

SERVO_IDS = ("000", "001", "002", "003")

# 조립 후 관절별로 실제 안전 범위를 측정해서 수정 (Kim)
PWM_LIMITS = {
    "000": (900, 2600),
    "001": (900, 2600),
    "002": (900, 2600),
    "003": (900, 2600),
}

# 사용자 구현 휴지/수납 자세. 이름은 home이지만 원점 센서 homing을 뜻하지 않는다.
HOME = {"000": 1500, "001": 1100, "002": 2400, "003": 1500}
STOW = HOME
STOW_USER_CONFIRMED = True
STOW_PHYSICALLY_VERIFIED = False

# 사이클별 포즈 표 — press_cycle N 은 POSES_N 의 값으로 움직인다. 튜닝은 해당 표의 숫자만 고치면 된다.
# (PWM 은 PWM_LIMITS 안이어야 하며, 새 자세가 필요하면 그 표에 키를 추가하고 PRESS_CYCLE_N 에서 이름으로 쓴다)
POSES_1 = {  # 사용자 메뉴 6: 엘리베이터 열림/닫힘 버튼
    "home": HOME,
    "press_ready": {"000": 1500, "001": 1500, "002": 1500, "003": 1500},
    "pre_press": {"000": 1600, "001": 1600, "002": 1900, "003": 1500},
    "press": {"000": 1600, "001": 1900, "002": 1800, "003": 1500},
    "retreat": {"000": 1600, "001": 1600, "002": 1900, "003": 1500},
}

POSES_2 = {  # 사용자 메뉴 7: 상단 좌측 장착 팔로 로봇 왼쪽 버튼
    "home": HOME,
    "press_ready": {"000": 1500, "001": 1500, "002": 1500, "003": 1500},
    "pre_press2": {"000": 1900, "001": 1600, "002": 1900, "003": 1500},
    "press2": {"000": 1900, "001": 1900, "002": 1800, "003": 1500},
    "retreat2": {"000": 1900, "001": 1600, "002": 1900, "003": 1500},
}

POSE_TABLES = {1: POSES_1, 2: POSES_2}
POSES = POSES_1  # 기본 포즈 표(하위 호환 — 기존 호출부/테스트는 이 이름을 쓴다)
CYCLE_LABELS = {1: "elevator_door_open_close", 2: "robot_left_button"}

# 사용자 메뉴 6/7의 버튼 누르기 사이클 2종 — 각 스텝은 (pose, duration_ms, send_command).
# send_command=False 는 '버튼 누른 상태 유지' 구간 — 재전송 없이 대기만 한다.
# 1번은 메뉴 6, 2번은 메뉴 7에 대응한다. 노드는 press_cycle 1/2로 선택한다.
PRESS_CYCLE_1 = (
    ("press_ready", 2000, True),
    ("pre_press", 1500, True),
    ("press", 1000, True),
    ("press", 500, False),
    ("retreat", 1000, True),
    ("press_ready", 1500, True),
    ("home", 1500, True),
)

PRESS_CYCLE_2 = (  # 스텝은 1번과 동일 — 포즈 값은 POSES_2
    ("press_ready", 2000, True),
    ("pre_press2", 1500, True),
    ("press2", 1000, True),
    ("press2", 500, False),
    ("retreat2", 1000, True),
    ("press_ready", 1500, True),
    ("home", 1500, True),
)

PRESS_CYCLES = {1: PRESS_CYCLE_1, 2: PRESS_CYCLE_2}
PRESS_CYCLE = PRESS_CYCLE_1  # 기본 사이클(하위 호환 — 기존 호출부/테스트는 이 이름을 쓴다)

# 명시적 stow 요청용 이름. 노드는 home_on_start=false이므로 기동 시 자동 전송하지 않는다.
HOME_POSE = "home"
HOMING_DURATION_MS = 2000


def check_pwm(servo_id, pwm):
    if servo_id not in SERVO_IDS:
        raise ValueError(f"알 수 없는 모터 ID: {servo_id}")
    minimum, maximum = PWM_LIMITS[servo_id]
    if not minimum <= pwm <= maximum:
        raise ValueError(f"ID {servo_id}의 PWM {pwm}이 안전 범위 {minimum}~{maximum}를 벗어났습니다.")


def pose_command(pose_name, duration_ms, poses=POSES):
    """네 모터 동시 이동 명령 문자열. PWM 안전 범위·이동 시간 검증 포함. poses 로 사이클별 표 지정."""
    if pose_name not in poses:
        raise ValueError(f"알 수 없는 자세: {pose_name}")
    if not 0 <= duration_ms <= 9999:
        raise ValueError("이동 시간은 0~9999ms 범위여야 합니다.")
    pose = poses[pose_name]
    for servo_id, pwm in pose.items():
        check_pwm(servo_id, pwm)
    parts = [f"#{servo_id}P{pose[servo_id]:04d}T{duration_ms:04d}!" for servo_id in SERVO_IDS]
    return "{" + "".join(parts) + "}"


def homing_command(duration_ms=HOMING_DURATION_MS):
    """명시적 home/stow 요청. 원점 센서 homing을 뜻하지 않는다."""
    return pose_command(HOME_POSE, duration_ms, {HOME_POSE: HOME})


def stop_command(servo_id):
    return f"#{servo_id}PDPT!"


def read_position_command(servo_id):
    return f"#{servo_id}PRAD!"


def parse_position(servo_id, response):
    match = re.search(rf"#{servo_id}P(\d+)\!", response or "")
    return int(match.group(1)) if match else None


def positions_reached(positions, target, tolerance_pwm):
    """모든 관절 실측 PWM이 목표 허용오차 안인지 판정한다."""
    if tolerance_pwm < 0:
        raise ValueError("position tolerance must be non-negative")
    if set(positions) != set(SERVO_IDS) or set(target) != set(SERVO_IDS):
        return False
    return all(abs(int(positions[sid]) - int(target[sid])) <= tolerance_pwm for sid in SERVO_IDS)


class SerialPoseDriver:
    """Serial backend that requires parseable position feedback from every servo."""

    def __init__(self, port, baud=115200, timeout=0.1, serial_module=None, logger=None):
        if serial_module is None:
            import serial as serial_module  # type: ignore[no-redef]

        self._conn = serial_module.Serial(port, baud, timeout=timeout)
        self._timeout = float(timeout)
        self._logger = logger
        if logger is not None:
            logger.info(f"arm serial open: {port} @ {baud}")

    def send_pose(self, pose_name, duration_ms, poses=POSES):
        payload = pose_command(pose_name, duration_ms, poses).encode("ascii")
        written = self._conn.write(payload)
        if written is not None and written != len(payload):
            raise IOError(f"short serial write: {written}/{len(payload)}")
        if hasattr(self._conn, "flush"):
            self._conn.flush()

    def read_positions(self):
        positions = {}
        for servo_id in SERVO_IDS:
            # 컨트롤러는 query ID를 되돌려주지만 시퀀스 번호가 없다. 각 query 전에
            # 이미 도착해 있던 바이트를 버려 이전 요청 응답을 fresh 응답으로 쓰지 않는다.
            waiting = int(getattr(self._conn, "in_waiting", 0) or 0)
            if waiting:
                self._conn.read(waiting)
            payload = read_position_command(servo_id).encode("ascii")
            written = self._conn.write(payload)
            if written is not None and written != len(payload):
                raise IOError(f"short serial write: {written}/{len(payload)}")
            if hasattr(self._conn, "flush"):
                self._conn.flush()
            deadline = time.monotonic() + max(self._timeout, 0.05)
            response = b""
            position = None
            while time.monotonic() < deadline:
                chunk = self._conn.read_until(b"!")
                if chunk:
                    response += chunk
                    position = parse_position(servo_id, response.decode("ascii", errors="ignore"))
                    if position is not None:
                        break
            if position is None:
                raise TimeoutError(f"missing position feedback for servo {servo_id}")
            check_pwm(servo_id, position)
            positions[servo_id] = position
        return positions

    def stop_all(self):
        for servo_id in SERVO_IDS:
            payload = stop_command(servo_id).encode("ascii")
            written = self._conn.write(payload)
            if written is not None and written != len(payload):
                raise IOError(f"short serial write: {written}/{len(payload)}")
        if hasattr(self._conn, "flush"):
            self._conn.flush()

    def close(self):
        self._conn.close()


def get_cycle(cycle_id):
    """press_cycle 1(메뉴6)/2(메뉴7) → 사이클 튜플."""
    try:
        return PRESS_CYCLES[int(cycle_id)]
    except (KeyError, ValueError, TypeError):
        raise ValueError(f"알 수 없는 press_cycle: {cycle_id!r} (가능: {sorted(PRESS_CYCLES)})") from None


def get_poses(cycle_id):
    """press_cycle 1(메뉴6)/2(메뉴7) → 그 사이클의 포즈 표."""
    try:
        return POSE_TABLES[int(cycle_id)]
    except (KeyError, ValueError, TypeError):
        raise ValueError(f"알 수 없는 press_cycle: {cycle_id!r} (가능: {sorted(POSE_TABLES)})") from None


def cycle_duration_ms(cycle=PRESS_CYCLE):
    return sum(duration for _, duration, _ in cycle)


def interpolate_pwm(elapsed_ms, cycle=PRESS_CYCLE, start_pose="home", poses=POSES):
    """사이클 시작 후 elapsed_ms 시점의 관절별 PWM 선형 보간 (토픽 관측/부하 발행용).

    컨트롤러 실동작의 근사치다 — 시리얼 전송과 무관하게 JointState 발행에 쓴다.
    범위 밖은 양끝 포즈로 clamp. poses 로 사이클별 표 지정.
    """
    prev = poses[start_pose]
    if elapsed_ms <= 0:
        return dict(prev)
    t = 0
    for pose_name, duration, _ in cycle:
        target = poses[pose_name]
        if elapsed_ms < t + duration:
            ratio = (elapsed_ms - t) / duration
            return {sid: prev[sid] + (target[sid] - prev[sid]) * ratio for sid in SERVO_IDS}
        t += duration
        prev = target
    return dict(prev)
