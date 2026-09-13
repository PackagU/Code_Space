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

    # /cmd_vel has one owner: Nav2, teleop, or the web UI (which zero-publishes when idle).
    for needle in ("/packagu_keyboard_teleop", "/packagu_field_web_ui", "command ownership"):
        require(needle in nav, f"navigation must refuse competing /cmd_vel owner {needle}")
    teleop = (ROOT / "scripts/teleop.sh").read_text(encoding="utf-8")
    web_ui = (ROOT / "scripts/start_field_web_ui.sh").read_text(encoding="utf-8")
    for path_name, text, needles in (
        ("teleop.sh", teleop, ("/controller_server", "/packagu_field_web_ui", "/packagu_keyboard_teleop")),
        ("start_field_web_ui.sh", web_ui, ("/controller_server", "/packagu_keyboard_teleop")),
    ):
        for needle in needles:
            require(needle in text and "command ownership" in text, f"{path_name} must refuse {needle}")
        result = subprocess.run(["bash", "-n", str(ROOT / "scripts" / path_name)], capture_output=True)
        require(result.returncode == 0, f"{path_name} shell syntax error")
    print("field scripts contract passed")


if __name__ == "__main__":
    main()
