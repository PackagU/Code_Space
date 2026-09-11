#!/usr/bin/env python3
"""실행 중인 Jetson 컨테이너의 무동작/라이다 배포 계약을 읽기 전용 검사한다."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import time


SERIAL_TARGETS = ("/dev/rplidar", "/dev/opencr", "/dev/arm_servo", "/dev/motor_nano")
REQUIRED_MOUNTS = {
    "/ros2_ws/src": None,
    "/ros2_ws/maps": True,
    "/ros2_ws/test_workspace": None,
    "/ros2_ws/scripts": False,
    "/ros2_ws/logs": True,
}


def command(*args: str, timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=False)


def fail(message: str, failures: list[str]) -> None:
    failures.append(message)
    print(f"FAIL {message}")


def check_ros(container: str, mode: str, failures: list[str]) -> None:
    ros_command = (
        "source /opt/ros/humble/setup.bash; "
        "test ! -f /ros2_ws/install/setup.bash || source /ros2_ws/install/setup.bash; "
        "ros2 node list --no-daemon"
    )
    result = command("docker", "exec", container, "bash", "-lc", ros_command, timeout=12)
    if result.returncode != 0:
        fail(f"ROS graph query failed: {result.stderr.strip()}", failures)
        return
    nodes = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    print(f"INFO ros_nodes={nodes}")
    duplicates = sorted({node for node in nodes if nodes.count(node) > 1})
    if duplicates:
        fail(f"duplicate node names: {duplicates}", failures)

    lowered = "\n".join(nodes).lower()
    unsafe_markers = ("opencr", "arm_sequence", "motor_nano", "lift")
    if mode in ("safe", "lidar"):
        active = [marker for marker in unsafe_markers if marker in lowered]
        if active:
            fail(f"motion-capable nodes active in {mode} mode: {active}", failures)

    # SLAM과 AMCL은 둘 다 map->odom 책임을 가질 수 있으므로 동시 실행을 금지한다.
    if "slam_toolbox" in lowered and "/amcl" in lowered:
        fail("possible duplicate map->odom publishers: slam_toolbox and amcl are both active", failures)

    for topic in ("/tf", "/tf_static"):
        info_command = (
            "source /opt/ros/humble/setup.bash; "
            f"ros2 topic info {topic} --verbose --no-daemon"
        )
        info = command("docker", "exec", container, "bash", "-lc", info_command, timeout=10)
        if info.returncode == 0:
            publisher_lines = [
                line.strip() for line in info.stdout.splitlines() if line.strip().startswith("Node name:")
            ]
            print(f"INFO {topic}_publishers={publisher_lines}")
        else:
            print(f"INFO {topic}_publishers=[]")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--container", default="ros2_humble")
    parser.add_argument("--mode", choices=("safe", "lidar", "inspect"), default="inspect")
    parser.add_argument("--expected-domain", default="0")
    parser.add_argument("--skip-ros", action="store_true")
    args = parser.parse_args()

    failures: list[str] = []
    inspected = command("docker", "inspect", args.container)
    if inspected.returncode != 0:
        print(f"FAIL container missing: {args.container}")
        return 1
    info = json.loads(inspected.stdout)[0]
    state = info["State"]
    host = info["HostConfig"]
    config = info["Config"]

    if not state.get("Running"):
        fail(f"container not running: {args.container}", failures)
    if host.get("Privileged"):
        fail("container is privileged", failures)
    if host.get("NetworkMode") != "host":
        fail(f"network mode is {host.get('NetworkMode')}, expected host", failures)
    if host.get("IpcMode") != "shareable":
        fail(f"IPC mode is {host.get('IpcMode')}, expected shareable", failures)

    env = {}
    for item in config.get("Env") or []:
        key, _, value = item.partition("=")
        env[key] = value
    domain = env.get("ROS_DOMAIN_ID", "")
    if domain != args.expected_domain:
        fail(f"ROS_DOMAIN_ID={domain!r}, expected {args.expected_domain!r}", failures)
    print(f"INFO container={args.container} image={info['Image']} mode={args.mode} domain={domain}")

    devices = {
        item["PathInContainer"]: item["PathOnHost"] for item in host.get("Devices") or []
    }
    print(f"INFO devices={devices}")
    for target in SERIAL_TARGETS:
        source = devices.get(target)
        if source is None:
            fail(f"device target missing: {target}", failures)
            continue
        should_be_real = args.mode == "lidar" and target == "/dev/rplidar"
        if args.mode == "safe" and source != "/dev/null":
            fail(f"safe mode exposes real device: {target} <- {source}", failures)
        if args.mode == "lidar" and not should_be_real and source != "/dev/null":
            fail(f"lidar mode exposes motion device: {target} <- {source}", failures)
        if should_be_real:
            path = Path(source)
            if source == "/dev/null" or not path.exists():
                fail(f"lidar source is unavailable: {source}", failures)
            elif not stat.S_ISCHR(path.resolve().stat().st_mode):
                fail(f"lidar source is not a character device: {source}", failures)

    mounts = {item["Destination"]: item for item in info.get("Mounts") or []}
    for target, must_write in REQUIRED_MOUNTS.items():
        mount = mounts.get(target)
        if mount is None:
            fail(f"required mount missing: {target}", failures)
            continue
        if must_write is True and not mount.get("RW"):
            fail(f"required writable mount is read-only: {target}", failures)
        if must_write is False and mount.get("RW"):
            fail(f"required read-only mount is writable: {target}", failures)
    mount_summary = ", ".join(
        f"{key}:RW={value.get('RW')}" for key, value in mounts.items()
    )
    print(f"INFO mounts={{{mount_summary}}}")

    clock = command("docker", "exec", args.container, "date", "+%s")
    if clock.returncode != 0:
        fail("container clock query failed", failures)
    else:
        delta = abs(int(clock.stdout.strip()) - int(time.time()))
        print(f"INFO clock_delta_sec={delta}")
        if delta > 2:
            fail(f"container clock differs from host by {delta}s", failures)

    if not args.skip_ros and state.get("Running"):
        check_ros(args.container, args.mode, failures)

    donor_id = info["Id"]
    sidecar = command("docker", "inspect", "packagu_rviz")
    if sidecar.returncode == 0:
        labels = json.loads(sidecar.stdout)[0]["Config"].get("Labels") or {}
        linked = labels.get("packagu.donor_id")
        if linked != donor_id:
            fail("RViz sidecar donor ID is stale; rerun run_rviz_jetson.sh", failures)
        else:
            print("INFO rviz_sidecar_donor=current")

    if failures:
        print(f"jetson_preflight=FAIL count={len(failures)}")
        return 1
    print("jetson_preflight=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
