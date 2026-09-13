#!/usr/bin/env python3
"""Pure offline contract for field_nav_cli waypoint handling."""

import importlib.util
import json
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
