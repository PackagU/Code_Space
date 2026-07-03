#!/usr/bin/env python3
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "src" / "drive_pkg" / "drive_pkg" / "keyboard_teleop.py"


def load_module():
    spec = importlib.util.spec_from_file_location("keyboard_teleop", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def assert_tuple(actual, expected):
    rounded_actual = tuple(round(value, 6) for value in actual)
    rounded_expected = tuple(round(value, 6) for value in expected)
    if rounded_actual != rounded_expected:
        raise AssertionError(f"expected {expected}, got {actual}")


def assert_is_none(actual):
    if actual is not None:
        raise AssertionError(f"expected None, got {actual}")


def assert_speed_change(actual, expected_linear, expected_angular, expected_label):
    linear_speed, angular_speed, label = actual
    if round(linear_speed, 6) != round(expected_linear, 6):
        raise AssertionError(f"expected linear {expected_linear}, got {linear_speed}")
    if round(angular_speed, 6) != round(expected_angular, 6):
        raise AssertionError(f"expected angular {expected_angular}, got {angular_speed}")
    if label != expected_label:
        raise AssertionError(f"expected label={expected_label!r}, got {label!r}")


def assert_state(actual, expected_binding, expected_linear, expected_angular, expected_command, expected_label):
    binding, linear_speed, angular_speed, command, label = actual
    assert_tuple(binding, expected_binding)
    if round(linear_speed, 6) != round(expected_linear, 6):
        raise AssertionError(f"expected linear {expected_linear}, got {linear_speed}")
    if round(angular_speed, 6) != round(expected_angular, 6):
        raise AssertionError(f"expected angular {expected_angular}, got {angular_speed}")
    assert_tuple(command, expected_command)
    if label != expected_label:
        raise AssertionError(f"expected label {expected_label!r}, got {label!r}")


def main():
    teleop = load_module()

    assert_tuple(teleop.command_for_key("w", 0.3, 1.0), (0.3, 0.0))
    assert_tuple(teleop.command_for_key("s", 0.3, 1.0), (-0.3, 0.0))
    assert_tuple(teleop.command_for_key("a", 0.3, 1.0), (0.0, 1.0))
    assert_tuple(teleop.command_for_key("d", 0.3, 1.0), (0.0, -1.0))
    assert_tuple(teleop.command_for_key("k", 0.3, 1.0), (0.0, 0.0))
    assert_tuple(teleop.command_for_key("i", 0.3, 1.0), (0.3, 0.0))
    assert_tuple(teleop.command_for_key(",", 0.3, 1.0), (-0.3, 0.0))
    assert_tuple(teleop.command_for_key("j", 0.3, 1.0), (0.0, 1.0))
    assert_tuple(teleop.command_for_key("l", 0.3, 1.0), (0.0, -1.0))
    assert_is_none(teleop.command_for_key("", 0.3, 1.0))
    assert_is_none(teleop.command_for_key("x", 0.3, 1.0))
    if teleop.motion_label_for_key("w") != "motion":
        raise AssertionError("expected w to be labeled as motion")
    if teleop.motion_label_for_key("k") != "stop":
        raise AssertionError("expected k to be labeled as stop")
    if teleop.motion_label_for_key(" ") != "stop":
        raise AssertionError("expected space to be labeled as stop")
    if teleop.motion_label_for_key("x") is not None:
        raise AssertionError("expected unknown keys to have no motion label")

    assert_speed_change(teleop.adjust_speeds_for_key("q", 0.25, 0.9), 0.275, 0.9, "linear")
    assert_speed_change(teleop.adjust_speeds_for_key("z", 0.25, 0.9), 0.225, 0.9, "linear")
    assert_speed_change(teleop.adjust_speeds_for_key("e", 0.25, 0.9), 0.25, 0.99, "angular")
    assert_speed_change(teleop.adjust_speeds_for_key("c", 0.25, 0.9), 0.25, 0.81, "angular")
    assert_speed_change(teleop.adjust_speeds_for_key("x", 0.25, 0.9), 0.25, 0.9, None)

    state = teleop.process_key("w", (0.0, 0.0), 0.25, 0.9)
    assert_state(state, (1.0, 0.0), 0.25, 0.9, (0.25, 0.0), None)

    state = teleop.process_key("q", state[0], state[1], state[2])
    assert_state(state, (1.0, 0.0), 0.275, 0.9, (0.275, 0.0), "linear")

    state = teleop.process_key("a", state[0], state[1], state[2])
    assert_state(state, (0.0, 1.0), 0.275, 0.9, (0.0, 0.9), None)

    state = teleop.process_key("q", state[0], state[1], state[2])
    assert_state(state, (0.0, 1.0), 0.3025, 0.9, (0.0, 0.9), "linear")

    state = teleop.process_key("e", state[0], state[1], state[2])
    assert_state(state, (0.0, 1.0), 0.3025, 0.99, (0.0, 0.99), "angular")

    state = teleop.process_key("k", state[0], state[1], state[2])
    assert_state(state, (0.0, 0.0), 0.3025, 0.99, (0.0, 0.0), None)

    state = teleop.process_key("w", state[0], state[1], state[2])
    assert_state(state, (1.0, 0.0), 0.3025, 0.99, (0.3025, 0.0), None)

    state = teleop.process_key(" ", state[0], state[1], state[2])
    assert_state(state, (0.0, 0.0), 0.3025, 0.99, (0.0, 0.0), None)

    print("PASS: WASD teleop key mapping is correct.")


if __name__ == "__main__":
    main()
