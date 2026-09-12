"""OpenCR 시리얼 프로토콜 v0.2 인코더/디코더 + 차동구동 변환.

계약 문서: docs/deployment/02_opencr_serial_protocol.md
순수 파이썬 (ROS 비의존) — 오프라인 테스트: scripts/test_opencr_protocol.py
"""
import math

FULL_FEEDBACK_FIELD_COUNT = 13  # "F" + 12 floats
MINIMAL_FEEDBACK_FIELD_COUNT = 3  # "F" + left/right rpm
PROTOCOL_VERSION = "0.2"


def encode_velocity_command(left_rpm, right_rpm):
    if not all(math.isfinite(value) for value in (left_rpm, right_rpm)):
        raise ValueError("wheel RPM command must be finite")
    return f"V {left_rpm:.2f} {right_rpm:.2f}\n".encode("ascii")


def parse_feedback_line(line, max_abs_rpm=None):
    tokens = line.strip().split()
    if len(tokens) not in (MINIMAL_FEEDBACK_FIELD_COUNT, FULL_FEEDBACK_FIELD_COUNT):
        return None
    if tokens[0] != "F":
        return None
    try:
        values = [float(token) for token in tokens[1:]]
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    if max_abs_rpm is not None and (
        abs(values[0]) > max_abs_rpm or abs(values[1]) > max_abs_rpm
    ):
        return None
    feedback = {
        "left_rpm": values[0],
        "right_rpm": values[1],
        "gyro": None,
        "accel": None,
        "quat": None,
    }
    if len(tokens) == FULL_FEEDBACK_FIELD_COUNT:
        quat_norm = math.sqrt(sum(value * value for value in values[8:12]))
        if not 0.5 <= quat_norm <= 1.5:
            return None
        feedback.update({
            "gyro": tuple(values[2:5]),
            "accel": tuple(values[5:8]),
            "quat": tuple(values[8:12]),
        })
    return feedback


def twist_to_wheel_rpm(v, w, wheel_radius, wheel_separation):
    if not all(math.isfinite(value) for value in (v, w, wheel_radius, wheel_separation)):
        raise ValueError("twist and wheel geometry must be finite")
    if wheel_radius <= 0.0 or wheel_separation <= 0.0:
        raise ValueError("wheel geometry must be positive")
    left_rad_s = (v - w * wheel_separation / 2.0) / wheel_radius
    right_rad_s = (v + w * wheel_separation / 2.0) / wheel_radius
    to_rpm = 60.0 / (2.0 * math.pi)
    return left_rad_s * to_rpm, right_rad_s * to_rpm
