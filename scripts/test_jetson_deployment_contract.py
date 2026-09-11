#!/usr/bin/env python3
"""Static contract checks for the device-free Jetson deployment profile."""
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DOCKERFILE = ROOT / "docker/Dockerfile.jetson"
COMPOSE = ROOT / "docker/compose/docker-compose.jetson.yml"
ENTRYPOINT = ROOT / "docker/scripts/entrypoint.sh"
ENV_EXAMPLE = ROOT / "docker/compose/jetson.env.example"


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        sys.exit(1)


def main():
    for path in (DOCKERFILE, COMPOSE, ENTRYPOINT, ENV_EXAMPLE):
        require(path.is_file(), f"missing {path.relative_to(ROOT)}")

    dockerfile = DOCKERFILE.read_text(encoding="utf-8")
    for package in (
        "python3-numpy",
        "python3-opencv",
        "python3-serial",
        "python3-yaml",
        "ros-humble-rviz2",
        "ros-humble-nav2-bringup",
        "ros-humble-slam-toolbox",
    ):
        require(package in dockerfile, f"Dockerfile missing {package}")
    for path in ("/ros2_ws/logs", "/ros2_ws/maps", "/opt/floor_reader/data"):
        require(path in dockerfile, f"Dockerfile missing runtime path {path}")

    compose = COMPOSE.read_text(encoding="utf-8")
    for token in (
        "floor_reader:",
        "profiles: [camera]",
        "${FLOOR_READER_SOURCE:-disabled}",
        "${FLOOR_READER_HOST:-127.0.0.1}",
        "${PACKAGU_FLOOR_READER:-../../../floor_reader}:/opt/floor_reader:ro",
        "${PACKAGU_FLOOR_DATA:-../../../floor_reader/data}:/opt/floor_reader/data:rw",
        'user: "${PACKAGU_UID:-1000}:${PACKAGU_GID:-1000}"',
        "PACKAGU_REQUIRED_READABLE_MOUNTS",
        "PACKAGU_REQUIRED_WRITABLE_MOUNTS",
        "${RPLIDAR_DEVICE:-/dev/null}",
        "${OPENCR_DEVICE:-/dev/null}",
        "${ARM_SERVO_DEVICE:-/dev/null}",
        "${MOTOR_NANO_DEVICE:-/dev/null}",
    ):
        require(token in compose, f"compose missing contract token: {token}")
    active_compose = "\n".join(line.split("#", 1)[0] for line in compose.splitlines())
    require("privileged: true" not in active_compose, "Jetson compose must not be privileged")

    entrypoint = ENTRYPOINT.read_text(encoding="utf-8")
    for token in ("check_paths", "findmnt", "exit 78"):
        require(token in entrypoint, f"entrypoint missing mount guard: {token}")
    require(
        subprocess.run(["bash", "-n", str(ENTRYPOINT)], capture_output=True).returncode == 0,
        "entrypoint shell syntax error",
    )

    example = ENV_EXAMPLE.read_text(encoding="utf-8")
    require("=/dev/null" in example, "example must keep hardware disabled")
    require("FLOOR_READER_SOURCE=disabled" in example, "camera must default disabled")
    for forbidden in ("PASSWORD=", "TOKEN=", "SECRET=", "KEY="):
        require(forbidden not in example.upper(), f"example contains forbidden field {forbidden}")

    print("Jetson deployment contract passed")


if __name__ == "__main__":
    main()
