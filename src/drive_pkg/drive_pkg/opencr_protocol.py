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
