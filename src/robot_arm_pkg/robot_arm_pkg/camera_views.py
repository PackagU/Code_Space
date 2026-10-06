"""User-taught camera poses; share the existing controller protocol."""
import json

from robot_arm_pkg import servo_protocol as sp


def validate_views(config):
    duration = config.get("duration_ms", 2000)
    if type(duration) is not int or not 1 <= duration <= 9999:
        raise ValueError("camera view duration_ms must be 1..9999")
    poses = {}
    for name in ("front_view", "floor_view"):
        pose = config.get(name)
        if not isinstance(pose, dict) or set(pose) != set(sp.SERVO_IDS):
            raise ValueError(f"{name} requires all four servo IDs")
        for sid, value in pose.items():
            if type(value) is not int:
                raise ValueError(f"{name}/{sid} must be an integer PWM")
            sp.check_pwm(sid, value)
        poses[name] = dict(pose)
    return poses, duration


def load_views(path):
    with open(path, encoding="utf-8") as source:
        return validate_views(json.load(source))
