#!/usr/bin/env python3
import select
import sys
import termios
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
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)

    linear_speed = 0.25
    angular_speed = 0.9
    current_binding = (0.0, 0.0)

    print(HELP)
    print(f"speed: linear={linear_speed:.2f} m/s angular={angular_speed:.2f} rad/s")

    try:
        while rclpy.ok():
            key = read_key(settings)
            if key == "\x03":
                break
            current_binding, linear_speed, angular_speed, command, changed_label = process_key(
                key,
                current_binding,
                linear_speed,
                angular_speed,
            )
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
            publish_twist(publisher, linear_x, angular_z)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        publish_twist(publisher, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
