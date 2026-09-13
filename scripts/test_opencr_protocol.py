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

    # v0.2 minimal frame: wheel odom is valid and IMU publication is intentionally omitted.
    minimal = proto.parse_feedback_line("F 12.5 -3.0", max_abs_rpm=120)
    assert minimal is not None
    approx(minimal["left_rpm"], 12.5)
    assert minimal["gyro"] is None and minimal["accel"] is None and minimal["quat"] is None

    # parse: 불량 라인은 None (프리픽스/필드수/비숫자)
    assert proto.parse_feedback_line("HELLO opencr 1.0") is None
    assert proto.parse_feedback_line("F 1.0") is None
    assert proto.parse_feedback_line("F a b c d e f g h i j k l") is None
    assert proto.parse_feedback_line("F nan 0 0 0 0 0 0 0 1 0 0 0") is None
    assert proto.parse_feedback_line("F inf 0 0 0 0 0 0 0 1 0 0 0") is None
    assert proto.parse_feedback_line("F 121 0 0 0 0 0 0 0 1 0 0 0", max_abs_rpm=120) is None
    assert proto.parse_feedback_line("F 0 0 0 0 0 0 0 0 0 0 0 0") is None
    for bad in ((float("nan"), 0.0), (0.0, float("inf"))):
        try:
            proto.encode_velocity_command(*bad)
        except ValueError:
            pass
        else:
            raise AssertionError("non-finite command must be rejected")

    # 거부 사유 분류는 수락 여부를 바꾸지 않고 현장 로그의 원인만 구분한다.
    classify = proto.classify_rejected_feedback
    assert classify("F 30.46 12.0", max_abs_rpm=30.0) == "feedback rpm over limit"
    assert classify("F 999 0 0 0 0 0 0 0 1 0 0 0", max_abs_rpm=30.0) == "feedback rpm over limit"
    assert classify("E dynamixel_write 3") == "firmware error: dynamixel_write"
    assert classify("E command_watchdog_stop") == "firmware error: command_watchdog_stop"
    assert classify("E $$$ 1") == "firmware error: unknown"
    assert classify("HELLO opencr 0.2-minimal") == "firmware hello line"
    for garbage in ("", "F nan 0", "F 0 0 0 0 0 0 0 0 0 0 0 0", "garbage"):
        assert classify(garbage, max_abs_rpm=30.0) == "invalid feedback frame", garbage
    assert proto.parse_feedback_line("F 30.46 12.0", max_abs_rpm=30.0) is None
    assert proto.parse_feedback_line("F 30.46 12.0", max_abs_rpm=33.0) is not None

    # twist -> rpm: 바퀴 1rev/s(=60rpm)가 되는 전진 속도에서 양쪽 동일 rpm
    r, L = 0.033, 0.51324
    v = 0.033 * 2.0 * math.pi
    left, right = proto.twist_to_wheel_rpm(v, 0.0, r, L)
    approx(left, 60.0)
    approx(right, 60.0)

    # 제자리 좌회전(w>0): 왼쪽 후진, 오른쪽 전진, 크기 동일
    left, right = proto.twist_to_wheel_rpm(0.0, 1.0, r, L)
    approx(left, -right)
    assert right > 0, "left turn must spin right wheel forward"

    for args in ((float("nan"), 0.0, r, L), (0.0, 0.0, 0.0, L)):
        try:
            proto.twist_to_wheel_rpm(*args)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid twist/geometry must be rejected")

    print("opencr_protocol tests passed")


if __name__ == "__main__":
    main()
