#!/usr/bin/env python3
"""Standalone arm menu using the same servo_protocol core as the ROS adapter.

Default mode is dry-run and opens no port. Physical mode accepts only the stable
udev alias and never moves on startup: it reads positions first, then waits for an
explicit stow or pose/cycle command.
"""

import argparse
import importlib.util
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "robot_arm_pkg" / "robot_arm_pkg"
ARM_PORT = "/dev/arm_servo"
POSE_NAMES = (
    "press_ready", "pre_press", "press", "retreat",
    "pre_press2", "press2", "retreat2",
)


def load_protocol():
    spec = importlib.util.spec_from_file_location("servo_protocol", PKG / "servo_protocol.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def dry_payloads(sp, command):
    if command == "positions":
        return [sp.read_position_command(sid) for sid in sp.SERVO_IDS]
    if command == "stop":
        return [sp.stop_command(sid) for sid in sp.SERVO_IDS]
    if command in ("stow", "quit"):
        return [sp.homing_command(3000)]
    if command in POSE_NAMES:
        cycle_id = 2 if command.endswith("2") else 1
        return [sp.pose_command(command, 2000, sp.get_poses(cycle_id))]
    if command.startswith("press-cycle-"):
        menu_number = int(command.rsplit("-", 1)[1])
        cycle_id = {6: 1, 7: 2}[menu_number]
        return [
            sp.pose_command(name, duration, sp.get_poses(cycle_id))
            for name, duration, send in sp.get_cycle(cycle_id) if send
        ]
    raise ValueError(f"unsupported command: {command}")


def wait_and_check(sp, driver, pose_name, duration_ms, poses, tolerance, timeout):
    driver.send_pose(pose_name, duration_ms, poses)
    time.sleep(duration_ms / 1000.0)
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = driver.read_positions()
        if sp.positions_reached(last, poses[pose_name], tolerance):
            return last
    raise TimeoutError(f"{pose_name}: controller position response timeout; last={last}")


def run_cycle(sp, driver, cycle_id, tolerance, timeout):
    poses = sp.get_poses(cycle_id)
    for pose_name, duration_ms, send in sp.get_cycle(cycle_id):
        if send:
            measured = wait_and_check(
                sp, driver, pose_name, duration_ms, poses, tolerance, timeout
            )
        else:
            time.sleep(duration_ms / 1000.0)
            measured = driver.read_positions()
            if not sp.positions_reached(measured, poses[pose_name], tolerance):
                raise TimeoutError(f"hold response mismatch: {measured}")
        print(f"controller_response_matched pose={pose_name} pwm={measured}")


def menu_help():
    return (
        "positions | stow | press_ready | pre_press | press | retreat | "
        "pre_press2 | press2 | retreat2 | press-cycle-6 | press-cycle-7 | "
        "stop | quit"
    )


def main():
    parser = argparse.ArgumentParser(description="4-servo arm standalone menu")
    parser.add_argument("--execute", action="store_true", help="approved physical mode")
    parser.add_argument("--port", default="", help="physical mode requires /dev/arm_servo")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--command", default="menu")
    parser.add_argument("--tolerance-pwm", type=int, default=30)
    parser.add_argument("--feedback-timeout", type=float, default=1.0)
    args = parser.parse_args()
    sp = load_protocol()

    if not args.execute:
        if args.port:
            parser.error("dry-run must not receive a serial port")
        if args.command == "menu":
            print("DRY-RUN; no serial port is open. " + menu_help())
            while True:
                command = input("arm> ").strip().lower()
                if not command:
                    continue
                try:
                    for payload in dry_payloads(sp, command):
                        print(payload)
                except (KeyError, ValueError) as exc:
                    print(f"REJECTED: {exc}")
                    continue
                if command == "quit":
                    return
        else:
            try:
                for payload in dry_payloads(sp, args.command):
                    print(payload)
            except (KeyError, ValueError) as exc:
                parser.error(str(exc))
            print("SIMULATED: payload only; no controller response or physical motion")
            return

    if args.port != ARM_PORT:
        parser.error("physical mode requires the exact udev alias --port /dev/arm_servo")
    port = Path(args.port)
    if not port.exists() or not port.is_char_device():
        sys.exit("/dev/arm_servo is not a real character device")
    if args.feedback_timeout <= 0 or args.tolerance_pwm < 0:
        parser.error("feedback timeout must be positive and tolerance non-negative")

    driver = sp.SerialPoseDriver(args.port, args.baud)
    stopped_or_faulted = False
    explicitly_stowed = False

    def execute(command):
        nonlocal stopped_or_faulted, explicitly_stowed
        if command == "positions":
            print(f"fresh_controller_positions={driver.read_positions()}")
            return False
        if command == "stop":
            driver.stop_all()
            stopped_or_faulted = True
            explicitly_stowed = False
            print("STOP sent; mechanical stop and holding torque are unverified")
            return False
        if command == "stow":
            measured = wait_and_check(
                sp, driver, sp.HOME_POSE, 3000, {sp.HOME_POSE: sp.HOME},
                args.tolerance_pwm, args.feedback_timeout,
            )
            explicitly_stowed = True
            stopped_or_faulted = False
            print(
                f"stow controller_response_matched={measured}; "
                "physical_stow_verified=false"
            )
            return False
        if command in POSE_NAMES:
            cycle_id = 2 if command.endswith("2") else 1
            measured = wait_and_check(
                sp, driver, command, 2000, sp.get_poses(cycle_id),
                args.tolerance_pwm, args.feedback_timeout,
            )
            explicitly_stowed = False
            print(f"controller_response_matched pose={command} pwm={measured}")
            return False
        if command.startswith("press-cycle-"):
            if not explicitly_stowed:
                raise RuntimeError("run explicit stow first in this session")
            cycle_id = {6: 1, 7: 2}[int(command.rsplit("-", 1)[1])]
            run_cycle(sp, driver, cycle_id, args.tolerance_pwm, args.feedback_timeout)
            explicitly_stowed = True
            print(
                f"cycle={cycle_id} label={sp.CYCLE_LABELS[cycle_id]} completed; "
                "final=stow controller_response; physical_result_unverified"
            )
            return False
        if command == "quit":
            if not stopped_or_faulted:
                execute("stow")
            return True
        raise ValueError(f"unknown command; {menu_help()}")

    try:
        print(f"startup fresh_controller_positions={driver.read_positions()}")
        print("No startup motion was sent. " + menu_help())
        if args.command != "menu":
            execute(args.command)
            return
        while True:
            try:
                if execute(input("arm> ").strip().lower()):
                    break
            except (KeyError, ValueError, RuntimeError) as exc:
                print(f"REJECTED: {exc}")
    except KeyboardInterrupt:
        stopped_or_faulted = True
        driver.stop_all()
        raise SystemExit("Interrupted: STOP sent; no automatic stow after fault")
    except Exception as exc:  # noqa: BLE001
        stopped_or_faulted = True
        try:
            driver.stop_all()
        finally:
            raise SystemExit(f"FAILED: {type(exc).__name__}: {exc}")
    finally:
        driver.close()


if __name__ == "__main__":
    main()

