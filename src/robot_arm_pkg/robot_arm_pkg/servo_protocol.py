"""Kim 실기 서보 컨트롤러 프로토콜 순수 로직 (ROS 의존 없음 — host 오프라인 테스트 대상).

벤치 확정 스크립트(Kim, 2026-08-17)에서 이식. 명령 포맷:
- 이동:   {#000P1500T2000!#001P...} — PWM 목표까지 T(ms) 동안 컨트롤러가 자체 보간
- 위치:   #000PRAD!  ->  #000P1500!
- 정지:   #000PDPT!
PWM/포즈 값은 조립 후 관절별 실측으로 갱신한다 (Kim 벤치 값 그대로 유지).
"""

from __future__ import annotations

import re

SERVO_IDS = ("000", "001", "002", "003")

# 조립 후 관절별로 실제 안전 범위를 측정해서 수정 (Kim)
PWM_LIMITS = {
    "000": (1000, 2500),
    "001": (1000, 2500),
    "002": (1000, 2500),
    "003": (1000, 2500),
}

# 공용 대기 자세 — 기동 homing 과 모든 사이클의 마지막 스텝이 이 값으로 돌아온다(사이클별로 다르게 두지 않는다).
HOME = {"000": 1500, "001": 1200, "002": 2000, "003": 1500}

# 사이클별 포즈 표 — press_cycle N 은 POSES_N 의 값으로 움직인다. 튜닝은 해당 표의 숫자만 고치면 된다.
# (PWM 은 PWM_LIMITS 안이어야 하며, 새 자세가 필요하면 그 표에 키를 추가하고 PRESS_CYCLE_N 에서 이름으로 쓴다)
POSES_1 = {  # 기준 — Kim 벤치 실측 2026-08-22
    "home": HOME,
    "press_ready": {"000": 1500, "001": 1500, "002": 1500, "003": 1500},
    "pre_press": {"000": 1500, "001": 1600, "002": 1600, "003": 1600},
    "press": {"000": 1500, "001": 1900, "002": 1700, "003": 1300},
    "retreat": {"000": 1500, "001": 1600, "002": 1600, "003": 1600},
}

POSES_2 = {  # 베이스(000) 1700 — Jetson 벤치 2026-08-22, 나머지 관절은 1번과 동일
    "home": HOME,
    "press_ready": {"000": 1700, "001": 1500, "002": 1500, "003": 1500},
    "pre_press": {"000": 1700, "001": 1600, "002": 1600, "003": 1600},
    "press": {"000": 1700, "001": 1900, "002": 1700, "003": 1300},
    "retreat": {"000": 1700, "001": 1600, "002": 1600, "003": 1600},
}

POSES_3 = {  # 베이스(000) 1300 — Jetson 벤치 2026-08-22, 나머지 관절은 1번과 동일
    "home": HOME,
    "press_ready": {"000": 1300, "001": 1500, "002": 1500, "003": 1500},
    "pre_press": {"000": 1300, "001": 1600, "002": 1600, "003": 1600},
    "press": {"000": 1300, "001": 1900, "002": 1700, "003": 1300},
    "retreat": {"000": 1300, "001": 1600, "002": 1600, "003": 1600},
}

POSE_TABLES = {1: POSES_1, 2: POSES_2, 3: POSES_3}
POSES = POSES_1  # 기본 포즈 표(하위 호환 — 기존 호출부/테스트는 이 이름을 쓴다)

# 버튼 누르기 사이클 3종 — 각 스텝은 (pose, duration_ms, send_command).
# send_command=False 는 '버튼 누른 상태 유지' 구간 — 재전송 없이 대기만 한다.
# 1번 = Kim run_press_cycle 과 동일한 기준 사이클. 2·3번은 스텝 순서는 1번과 같고 포즈 값(POSES_N)만
# 버튼 위치별로 다르게 튜닝한다. 노드는 press_cycle 파라미터(1~3)로 스텝표+포즈표를 함께 선택.
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
    ("pre_press", 1500, True),
    ("press", 1000, True),
    ("press", 500, False),
    ("retreat", 1000, True),
    ("press_ready", 1500, True),
    ("home", 1500, True),
)

PRESS_CYCLE_3 = (  # 스텝은 1번과 동일 — 포즈 값은 POSES_3
    ("press_ready", 2000, True),
    ("pre_press", 1500, True),
    ("press", 1000, True),
    ("press", 500, False),
    ("retreat", 1000, True),
    ("press_ready", 1500, True),
    ("home", 1500, True),
)

PRESS_CYCLES = {1: PRESS_CYCLE_1, 2: PRESS_CYCLE_2, 3: PRESS_CYCLE_3}
PRESS_CYCLE = PRESS_CYCLE_1  # 기본 사이클(하위 호환 — 기존 호출부/테스트는 이 이름을 쓴다)

# 기동 homing: 전원 인가 직후 서보는 전부 1500(중립)에 있다. 노드가 뜨면 한 번 home 으로
# 보내 "대기 중 = home" 을 맞춘다. 사이클 마지막 스텝도 home 이라 이후 대기는 자동 유지.
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
    """기동 시 1회 전송 — 전원 인가 직후(전 모터 1500) → home 대기 자세(공용 HOME)."""
    return pose_command(HOME_POSE, duration_ms, {HOME_POSE: HOME})


def stop_command(servo_id):
    return f"#{servo_id}PDPT!"


def read_position_command(servo_id):
    return f"#{servo_id}PRAD!"


def parse_position(servo_id, response):
    match = re.search(rf"#{servo_id}P(\d+)\!", response or "")
    return int(match.group(1)) if match else None


def get_cycle(cycle_id):
    """press_cycle 파라미터(1~3) → 사이클 튜플. 없는 번호는 ValueError."""
    try:
        return PRESS_CYCLES[int(cycle_id)]
    except (KeyError, ValueError, TypeError):
        raise ValueError(f"알 수 없는 press_cycle: {cycle_id!r} (가능: {sorted(PRESS_CYCLES)})") from None


def get_poses(cycle_id):
    """press_cycle 파라미터(1~3) → 그 사이클의 포즈 표. 없는 번호는 ValueError."""
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
