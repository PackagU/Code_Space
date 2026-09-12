#!/usr/bin/env python3
"""Static fail-closed contract for P07 field scripts."""

from pathlib import Path
import os
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
FILES = {
    "record": ROOT / "scripts/record_field_bag.sh",
    "inspect": ROOT / "scripts/inspect_field_bag.sh",
    "replay": ROOT / "scripts/replay_field_bag.sh",
    "load": ROOT / "scripts/monitor_field_load.sh",
    "runtime": ROOT / "scripts/test_p07_rosbag_runtime.sh",
    "contract": ROOT / "scripts/bag_contract.py",
    "metrics": ROOT / "scripts/field_topic_metrics.py",
    "qos": ROOT / "scripts/rosbag_qos_overrides.yaml",
}


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def main():
    for path in FILES.values():
        require(path.is_file(), f"missing {path.relative_to(ROOT)}")
    bash = shutil.which("bash")
    if os.name == "nt":
        git_bash = Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "Git" / "bin" / "bash.exe"
        if git_bash.is_file():
            bash = str(git_bash)
    require(bash is not None, "bash is unavailable")
    for name in ("record", "inspect", "replay", "load", "runtime"):
        result = subprocess.run([bash, "-n", str(FILES[name])], capture_output=True)
        require(result.returncode == 0, f"shell syntax error: {FILES[name].name}")

    record = FILES["record"].read_text(encoding="utf-8")
    for token in ("/scan", "/odom", "/tf", "/tf_static", "rosbag_qos_overrides.yaml", "bag_contract.py"):
        require(token in record, f"record contract missing {token}")
    require('INCLUDE_CONTROL_DIAGNOSTICS:-0' in record, "control diagnostics must default off")
    require("refusing to overwrite" in record, "record path needs no-overwrite guard")

    replay = FILES["replay"].read_text(encoding="utf-8")
    for token in ("REPLAY_DOMAIN_ID", "PHYSICAL_DOMAIN_ID", "replay-topics", "--topics", "--clock"):
        require(token in replay, f"replay guard missing {token}")
    require("opencr|arm|lift|servo|controller_server|nav_safety_gate" in replay,
            "replay domain hardware-node guard missing")
    require("ros2 bag play '${BAG_PATH}'" in replay, "replay path must be quoted")

    contract = FILES["contract"].read_text(encoding="utf-8")
    for forbidden in ("/cmd_vel", "/arm", "/lift", "/servo"):
        require(forbidden in contract, f"forbidden replay prefix missing: {forbidden}")
    load = FILES["load"].read_text(encoding="utf-8")
    require('DURATION_SEC:-1800' in load, "field load proposal must default to 30 minutes")
    require("MONITOR_DOMAIN_ID" in load, "load monitor needs an isolated-domain option")
    require("resources.csv" in load and "topic_metrics.json" in load, "load evidence outputs missing")
    print("P07 scripts contract passed")


if __name__ == "__main__":
    main()
