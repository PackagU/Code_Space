#!/usr/bin/env python3
"""Static contract for tomorrow's base, mapping, save, and Nav2 scripts."""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = {
    "base": ROOT / "scripts/start_field_base.sh",
    "mapping": ROOT / "scripts/start_field_mapping.sh",
    "save": ROOT / "scripts/save_field_map.sh",
    "nav": ROOT / "scripts/start_field_navigation.sh",
}


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def main():
    for name, path in SCRIPTS.items():
        require(path.is_file(), f"missing {name} script")
        result = subprocess.run(["bash", "-n", str(path)], capture_output=True)
        require(result.returncode == 0, f"{path.name} shell syntax error")

    base = SCRIPTS["base"].read_text(encoding="utf-8")
    require('ENABLE_DRIVE="${ENABLE_DRIVE:-0}"' in base, "drive must default disabled")
    require("/dev/rplidar" in base and "/dev/opencr" in base, "stable ports missing")
    require("stat -c '%t:%T'" in base and "'1:3'" in base,
            "base script must reject /dev/null bind mounts")

    mapping = SCRIPTS["mapping"].read_text(encoding="utf-8")
    for needle in ("/rplidar", "/packagu_opencr_bridge", "/nav_safety_gate", "/scan", "/odom"):
        require(needle in mapping, f"mapping preflight missing {needle}")

    save = SCRIPTS["save"].read_text(encoding="utf-8")
    for needle in ("map_saver_cli", "serialize_map", "map_contract", "sha256sum", "latest_map.txt"):
        require(needle in save, f"save script missing {needle}")
    require("refusing to overwrite" in save, "save script needs no-overwrite guard")

    nav = SCRIPTS["nav"].read_text(encoding="utf-8")
    for needle in ("map_contract", "/slam_toolbox", "/nav_safety/ready", "kku_navigation.launch.py"):
        require(needle in nav, f"navigation preflight missing {needle}")
    require("ros2 action send_goal" not in nav, "navigation startup must not send a goal")
    print("field scripts contract passed")


if __name__ == "__main__":
    main()
