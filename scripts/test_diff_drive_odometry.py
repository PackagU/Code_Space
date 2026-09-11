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
    #    (yaw 기대값 약 0.808 rad < pi — 정규화와 충돌하지 않음. 값 바꿀 때 pi 초과 여부 확인)
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
    approx(qy, 0.0, 1e-9, "qy")
    approx(qz, math.sin(math.pi / 4.0), 1e-9, "qz")
    approx(qw, math.cos(math.pi / 4.0), 1e-9, "qw")

    # 5) 유한하지 않은 입력과 시각 역행은 pose를 바꾸기 전에 거절한다.
    x_before = odo.x
    for args in ((float("nan"), 0.0, 0.02), (0.0, 0.0, -0.01)):
        try:
            odo.update(*args)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid odometry input must be rejected")
    approx(odo.x, x_before, 1e-12, "invalid input keeps pose")

    print("diff_drive_odometry tests passed")


if __name__ == "__main__":
    main()
