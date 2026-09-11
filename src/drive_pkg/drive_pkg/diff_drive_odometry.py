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
        if not all(math.isfinite(value) for value in (left_rpm, right_rpm, dt)):
            raise ValueError("odometry inputs must be finite")
        if dt < 0.0:
            raise ValueError("odometry dt must be non-negative")
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
