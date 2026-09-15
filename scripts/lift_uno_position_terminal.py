#!/usr/bin/env python3
"""Guarded terminal for the UNO_TB6600_POSITION_GUARD_V1 sketch."""

import argparse
import os
import sys
import threading
import time

import serial
from serial.tools import list_ports


LIDAR_VID_PID = (0x10C4, 0xEA60)
KNOWN_ARDUINO_VIDS = {0x2341, 0x2A03}


def available_ports():
    return list(list_ports.comports())


def port_line(port):
    vid_pid = "unknown"
    if port.vid is not None and port.pid is not None:
        vid_pid = f"{port.vid:04x}:{port.pid:04x}"
    return f"{port.device} vid:pid={vid_pid} serial={port.serial_number or 'unknown'} {port.description}"


def print_ports(ports):
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(port_line(port))


def choose_port(requested, allow_unknown):
    ports = available_ports()
    if requested:
        requested_real = os.path.realpath(requested)
        matches = [port for port in ports if os.path.realpath(port.device) == requested_real]
        if not matches:
            raise RuntimeError(f"Requested port is not present: {requested}")
        selected = matches[0]
    else:
        matches = [port for port in ports if port.vid in KNOWN_ARDUINO_VIDS]
        if len(matches) != 1:
            raise RuntimeError("Expected exactly one known Arduino port; pass --port after checking --list")
        selected = matches[0]

    if (selected.vid, selected.pid) == LIDAR_VID_PID:
        raise RuntimeError("Refusing known CP2102 LiDAR port; do not use it for the lift Arduino")
    if selected.vid not in KNOWN_ARDUINO_VIDS and not allow_unknown:
        raise RuntimeError("Unknown USB serial adapter; verify the board, then add --allow-unknown")
    return requested if requested else selected.device


def reader_loop(connection, finished):
    while not finished.is_set():
        try:
            raw = connection.readline()
        except serial.SerialException as exc:
            print(f"\nSERIAL ERROR: {exc}")
            finished.set()
            return
        if raw:
            print(f"\nUNO> {raw.decode('utf-8', errors='replace').rstrip()}")


def run_terminal(device):
    if not os.access(device, os.R_OK | os.W_OK):
        username = os.environ.get("USER", "<user>")
        raise RuntimeError(
            f"No read/write permission for {device}. "
            f"Run 'sudo usermod -aG dialout {username}', then fully reconnect SSH."
        )
    print(f"Opening {device} at 115200 baud. Opening USB may reset the Uno.")
    print("Commands: t=set top, b=set bottom, u=up, d=down, s=status, !=stop pulses, h=help, q=quit")
    with serial.Serial(device, 115200, timeout=0.2) as connection:
        time.sleep(2.0)
        connection.reset_input_buffer()
        finished = threading.Event()
        reader = threading.Thread(target=reader_loop, args=(connection, finished), daemon=True)
        reader.start()
        connection.write(b"s\n")

        try:
            while not finished.is_set():
                command = input("lift> ").strip()
                if command.lower() == "q":
                    break
                if command not in {"t", "T", "b", "B", "u", "U", "d", "D", "s", "S", "!", "h", "H", "?"}:
                    print("Rejected locally. Use only t, b, u, d, s, !, h, or q.")
                    continue
                connection.write(command.encode("ascii") + b"\n")
                connection.flush()
        finally:
            finished.set()
            reader.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="list ports without opening one")
    parser.add_argument("--port", help="exact Arduino serial device, for example /dev/ttyACM0")
    parser.add_argument("--allow-unknown", action="store_true", help="allow a verified clone USB adapter")
    parser.add_argument("--execute", action="store_true", help="open the port for an intentional live test")
    args = parser.parse_args()

    if args.list:
        print_ports(available_ports())
        return 0
    if not args.execute:
        print("Dry safety mode: no serial port opened. Use --list first, then --execute --port DEVICE.")
        return 0

    try:
        device = choose_port(args.port, args.allow_unknown)
        run_terminal(device)
    except (RuntimeError, serial.SerialException) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
