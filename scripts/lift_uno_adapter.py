#!/usr/bin/env python3
"""Fail-closed CLI adapter for the sensorless Uno+TB6600 jog firmware."""

import argparse
import json
import sys
import time
from pathlib import Path

LIFT_PORT = "/dev/lift_uno"
UNSUPPORTED = {"home", "move_to", "load", "unload"}


def emit(**fields):
    print(json.dumps(fields, sort_keys=True, separators=(",", ":")))


def frames_for(action, direction=None, pulses=20):
    if action == "jog":
        if direction not in ("d", "u"):
            raise ValueError("jog direction must be d or u")
        if not 1 <= pulses <= 12800:
            raise ValueError("pulses must be 1..12800")
        return [f"arm {pulses}\n".encode("ascii"), f"{direction}\n".encode("ascii")]
    if action == "stop":
        return [b"!"]
    if action == "status":
        return [b"status\n"]
    if action in UNSUPPORTED:
        return []
    raise ValueError(f"unknown action: {action}")


def read_lines(conn, timeout):
    deadline = time.monotonic() + timeout
    lines = []
    while time.monotonic() < deadline:
        raw = conn.readline()
        if raw:
            lines.append(raw.decode("ascii", errors="replace").strip())
        else:
            time.sleep(0.01)
    return lines


def main():
    parser = argparse.ArgumentParser(description="Uno TB6600 jog/stop/status adapter")
    parser.add_argument("action", choices=["jog", "stop", "status", *sorted(UNSUPPORTED)])
    parser.add_argument("direction", nargs="?", choices=["d", "u"])
    parser.add_argument("--pulses", type=int, default=20)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--port", default="")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--response-timeout", type=float, default=0.5)
    args = parser.parse_args()

    if args.action in UNSUPPORTED:
        emit(
            state="UNSUPPORTED",
            action=args.action,
            reason="no_limit_switch_or_position_feedback",
        )
        return 2
    try:
        frames = frames_for(args.action, args.direction, args.pulses)
    except ValueError as exc:
        parser.error(str(exc))

    if not args.execute:
        if args.port:
            parser.error("dry-run must not receive a serial port")
        emit(
            state="DRY_RUN",
            action=args.action,
            frames=[frame.decode("ascii", errors="replace") for frame in frames],
            position_verified=False,
            direction_verified=False,
        )
        return 0

    if args.port != LIFT_PORT:
        parser.error("physical mode requires exact udev alias --port /dev/lift_uno")
    port = Path(args.port)
    if not port.exists() or not port.is_char_device():
        sys.exit("/dev/lift_uno is not a real character device")
    if args.response_timeout <= 0:
        parser.error("response timeout must be positive")

    import fcntl
    import serial

    lock_path = Path("/tmp/packagu_lift_uno.lock")
    with lock_path.open("w", encoding="ascii") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            sys.exit("lift serial owner already active")
        with serial.Serial(args.port, args.baud, timeout=0.05) as conn:
            time.sleep(2.0)  # Uno reset after port open; physical mode only
            startup = read_lines(conn, 0.2)
            responses = []
            for frame in frames:
                written = conn.write(frame)
                conn.flush()
                if written != len(frame):
                    sys.exit(f"short serial write {written}/{len(frame)}")
                responses.extend(read_lines(conn, args.response_timeout))
            rejected = any(line.startswith(("REJECTED", "UNSUPPORTED")) for line in responses)
            emit(
                state="REJECTED" if rejected else "COMMAND_ACKNOWLEDGED",
                action=args.action,
                startup=startup,
                responses=responses,
                pulse_generation_complete=any("PULSES_COMPLETE" in line for line in responses),
                movement_verified=False,
                position_verified=False,
                direction_verified=False,
            )
            return 1 if rejected else 0


if __name__ == "__main__":
    raise SystemExit(main())

