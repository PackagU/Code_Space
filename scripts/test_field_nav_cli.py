#!/usr/bin/env python3
"""Pure offline contract for field_nav_cli waypoint handling."""

import importlib.util
import json
import subprocess
import tempfile
from argparse import Namespace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "scripts/field_nav_cli.py"


def load_module():
    spec = importlib.util.spec_from_file_location("field_nav_cli", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    mod = load_module()
    fieldctl = ROOT / "scripts/fieldctl"
    syntax = subprocess.run(["bash", "-n", str(fieldctl)], capture_output=True)
    assert syntax.returncode == 0, "fieldctl shell syntax failed"
    shell = fieldctl.read_text(encoding="utf-8")
    assert "pkill" not in shell, "fieldctl must not use broad pkill"
    assert shell.index("runtime_cli stop-state assert") < shell.index("runtime_cli cancel || true")
    assert "start/status/map 명령은 software stop을 해제하거나 goal을 보내지 않는다" in shell
    source = MODULE.read_text(encoding="utf-8")
    assert "/nav_safety/stopped" in source
    assert "saved active goal id is absent; refusing broad cancel" in source
    assert mod.quaternion_from_yaw(0.0) == (0.0, 1.0)
    try:
        mod.reject_unverified_origin(0.0, 0.0, 0.0, False)
    except ValueError:
        pass
    else:
        raise AssertionError("all-zero pose must be rejected")

    with tempfile.TemporaryDirectory() as directory:
        registry = Path(directory) / "waypoints.json"
        args = Namespace(
            registry=str(registry), name="f1_lobby", floor="f1", x="1.25", y="-0.5", yaw="1.57",
            allow_origin=False, replace=False,
        )
        assert mod.cmd_waypoint_save(args) == 0
        stored = json.loads(registry.read_text(encoding="utf-8"))["waypoints"]["f1_lobby"]
        assert stored["floor"] == "F1" and stored["frame_id"] == "map"
        assert stored["x"] == 1.25 and stored["y"] == -0.5 and stored["yaw_rad"] == 1.57
        try:
            mod.cmd_waypoint_save(args)
        except ValueError:
            pass
        else:
            raise AssertionError("overwrite must require --replace")
    print("field nav CLI pure contract passed")


if __name__ == "__main__":
    main()
