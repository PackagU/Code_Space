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

    print("opencr_protocol tests passed")


if __name__ == "__main__":
    main()
