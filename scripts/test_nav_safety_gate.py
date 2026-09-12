#!/usr/bin/env python3
"""Pure offline contract tests for the physical navigation safety gate."""

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src/drive_pkg/drive_pkg/safety_gate.py"


def load_module():
    spec = importlib.util.spec_from_file_location("safety_gate", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    mod = load_module()
    gate = mod.SafetyGate()

    result = gate.filter_command(0.1, 0.2, 1.0, 1.0)
    assert result.linear_x == 0.0 and not result.system_ready
    assert result.reason == "scan stale"

    gate.observe_scan(1.0)
    gate.observe_odom(1.0)
    gate.observe_drive_ready(True, 1.0)
    result = gate.filter_command(0.1, 0.2, 1.0, 1.1)
    assert result.system_ready and result.command_forwarded
    assert (result.linear_x, result.angular_z) == (0.1, 0.2)

    result = gate.filter_command(0.1, 0.2, 1.0, 1.31)
    assert result.system_ready and not result.command_forwarded
    assert result.reason == "command stale"

    result = gate.filter_command(0.101, 0.0, 1.31, 1.31)
    assert result.linear_x == 0.0 and result.reason == "command exceeds field limit"

    result = gate.filter_command(float("nan"), 0.0, 1.31, 1.31)
    assert result.linear_x == 0.0 and result.reason == "non-finite command"

    gate.set_software_stop(True)
    result = gate.filter_command(0.1, 0.0, 1.31, 1.31)
    assert not result.system_ready and result.reason == "software stop asserted"

    gate.set_software_stop(False)
    gate.observe_drive_ready(False, 1.31)
    result = gate.filter_command(0.1, 0.0, 1.31, 1.31)
    assert not result.system_ready and result.reason == "drive not ready"

    gate.observe_drive_ready(True, 1.31)
    result = gate.filter_command(0.1, 0.0, 1.31, 1.61)
    assert not result.system_ready and result.reason == "scan stale"

    try:
        mod.SafetyGate(max_linear_speed=0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("zero speed limit must be rejected")

    print("nav safety gate tests passed")


if __name__ == "__main__":
    main()
