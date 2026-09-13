#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


HELP = """
WASD teleop for /cmd_vel

  w: forward      s: backward
  a: turn left    d: turn right
  k or space: stop

Also supported: i/j/k/l and comma like teleop_twist_keyboard.
w/s/a/d select motion; k or space stops.
q/z: linear speed up/down
e/c: angular speed up/down
Ctrl+C: quit
"""


MOVE_BINDINGS = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    "i": (1.0, 0.0),
    ",": (-1.0, 0.0),
    "j": (0.0, 1.0),
    "l": (0.0, -1.0),
    "k": (0.0, 0.0),
    " ": (0.0, 0.0),
}

DEADMAN_TIMEOUT_SEC = 0.5
DEFAULT_LINEAR_SPEED = 0.10   # H01/H02 전 저속 [제안값]
DEFAULT_ANGULAR_SPEED = 0.35  # nav_safety.yaml max_angular_speed와 같음 [제안값]
# 2026-09-13: gate는 한도를 넘는 명령을 깎지 않고 0으로 바꾼다(safety_gate.py).
# q/e로 이 값을 넘기면 로봇이 서므로 teleop 단계에서 nav_safety.yaml 한도에 묶는다.
FIELD_MAX_LINEAR_SPEED = 0.10
FIELD_MAX_ANGULAR_SPEED = 0.35
# 정지 뒤 0 명령은 이 시간만 보내고 조용히 한다. 다른 /cmd_vel 발행자와 0이 섞이지 않게 한다.
IDLE_ZERO_HOLD_SEC = 0.5


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


def clamp_speeds(linear_speed, angular_speed,
                 max_linear=FIELD_MAX_LINEAR_SPEED, max_angular=FIELD_MAX_ANGULAR_SPEED):
    """Keep q/e speed changes inside the gate limits instead of letting the gate zero them."""
    clamped = (min(linear_speed, max_linear), min(angular_speed, max_angular))
    return clamped[0], clamped[1], clamped != (linear_speed, angular_speed)


def should_publish(command, zero_since, now, hold=IDLE_ZERO_HOLD_SEC):
    """Always publish motion; publish zero only for a short hold after stopping."""
    if command != (0.0, 0.0):
        return True
    return zero_since is not None and 0.0 <= now - zero_since <= hold


def motion_label_for_key(key):
    key = key.lower()
    if key in ("k", " "):
        return "stop"
    if key in MOVE_BINDINGS:
        return "motion"
    return None


def command_for_key(key, linear_speed, angular_speed):
    binding = MOVE_BINDINGS.get(key.lower())
    if binding is None:
        return None
    linear_scale, angular_scale = binding
    return linear_scale * linear_speed, angular_scale * angular_speed


def binding_for_key(key):
    return MOVE_BINDINGS.get(key.lower())


def command_for_binding(binding, linear_speed, angular_speed):
    linear_scale, angular_scale = binding
    return linear_scale * linear_speed, angular_scale * angular_speed


def adjust_speeds_for_key(key, linear_speed, angular_speed):
    key = key.lower()
    if key == "q":
        return linear_speed * 1.1, angular_speed, "linear"
    if key == "z":
        return linear_speed * 0.9, angular_speed, "linear"
    if key == "e":
        return linear_speed, angular_speed * 1.1, "angular"
    if key == "c":
        return linear_speed, angular_speed * 0.9, "angular"
    return linear_speed, angular_speed, None


def process_key(key, current_binding, linear_speed, angular_speed):
    linear_speed, angular_speed, changed_label = adjust_speeds_for_key(
        key,
        linear_speed,
        angular_speed,
    )
    if changed_label is None:
        binding = binding_for_key(key)
        if binding is not None:
            current_binding = binding

    command = command_for_binding(
        current_binding,
        linear_speed,
        angular_speed,
    )
    return current_binding, linear_speed, angular_speed, command, changed_label


def read_key(settings):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if ready else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def publish_twist(publisher, linear_x, angular_z):
    twist = Twist()
    twist.linear.x = float(linear_x)
    twist.angular.z = float(angular_z)
    publisher.publish(twist)


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node("packagu_keyboard_teleop")
    max_linear = float(node.declare_parameter("max_linear_speed", FIELD_MAX_LINEAR_SPEED).value)
    max_angular = float(node.declare_parameter("max_angular_speed", FIELD_MAX_ANGULAR_SPEED).value)
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)

    linear_speed, angular_speed, _capped = clamp_speeds(
        DEFAULT_LINEAR_SPEED, DEFAULT_ANGULAR_SPEED, max_linear, max_angular,
    )
    current_binding = (0.0, 0.0)

    print(HELP)
    print(f"speed: linear={linear_speed:.2f} m/s angular={angular_speed:.2f} rad/s "
          f"(field limit {max_linear:.2f} m/s, {max_angular:.2f} rad/s)")

    last_key_time = time.monotonic()
    zero_since = last_key_time
    try:
        while rclpy.ok():
            key = read_key(settings)
            if key == "\x03":
                break
            now = time.monotonic()
            if key:
                last_key_time = now
            current_binding, linear_speed, angular_speed, command, changed_label = process_key(
                key,
                current_binding,
                linear_speed,
                angular_speed,
            )
            linear_speed, angular_speed, capped = clamp_speeds(
                linear_speed, angular_speed, max_linear, max_angular,
            )
            if capped:
                print(f"speed capped at field limit: linear={linear_speed:.2f} m/s "
                      f"angular={angular_speed:.2f} rad/s (gate sends 0 above it)")
            command = command_for_binding(current_binding, linear_speed, angular_speed)
            idle_seconds = now - last_key_time
            command, deadman_stopped = apply_deadman(command, idle_seconds)
            if deadman_stopped:
                current_binding = (0.0, 0.0)
                message = (f"deadman stop: no key for {idle_seconds:.2f} s "
                           f"(limit {DEADMAN_TIMEOUT_SEC:.1f} s) -> cmd_vel=(0.00, 0.00)")
                print(message)
                # rosout에 남겨 field_cmd_chain_probe가 입력 두절 정지를 다른 정지와 구분하게 한다.
                node.get_logger().warning(message)
            linear_x, angular_z = command
            if changed_label:
                print(
                    f"{changed_label} speed changed: "
                    f"linear={linear_speed:.2f} m/s "
                    f"angular={angular_speed:.2f} rad/s "
                    f"cmd_vel=({linear_x:.2f}, {angular_z:.2f})"
                )
            else:
                motion_label = motion_label_for_key(key)
                if motion_label:
                    print(f"{motion_label}: cmd_vel=({linear_x:.2f}, {angular_z:.2f})")
            if command == (0.0, 0.0):
                zero_since = now if zero_since is None else zero_since
            else:
                zero_since = None
            if should_publish(command, zero_since, now):
                publish_twist(publisher, linear_x, angular_z)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        publish_twist(publisher, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
