#!/usr/bin/env python3
"""Sensorless lift adapter/firmware contract; no port is opened."""

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "scripts" / "lift_uno_adapter.py"
SAFE = ROOT / "legacy" / "arduino" / "stepper_test" / "uno_tb6600_manual" / "uno_tb6600_manual.ino"
REFERENCE = ROOT / "legacy" / "arduino" / "stepper_test" / "uno_user_reference_20260913" / "uno_user_reference_20260913.ino"


def main():
    spec = importlib.util.spec_from_file_location("lift_uno_adapter", ADAPTER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.frames_for("jog", "d", 20) == [b"arm 20\n", b"d\n"]
    assert module.frames_for("jog", "u", 12800) == [b"arm 12800\n", b"u\n"]
    assert module.frames_for("stop") == [b"!"]
    assert module.frames_for("status") == [b"status\n"]
    for pulses in (0, 12801):
        try:
            module.frames_for("jog", "d", pulses)
        except ValueError:
            pass
        else:
            raise AssertionError("unsafe pulse count accepted")

    if not SAFE.exists() or not REFERENCE.exists():
        print("PASS Uno lift adapter; firmware source checks unavailable in this container mount")
        return

    safe = SAFE.read_text(encoding="utf-8")
    for token in (
        "STEP_PIN = 7", "DIR_PIN = 6", "EN_PIN = 5",
        "PULSE_HIGH_US = 10", "PULSE_LOW_US = 365",
        "MAX_JOG_PULSES = 12800", "ARM_WINDOW_MS = 10000",
        "if (c == '!')", "PULSES_COMPLETE; movement/distance/position unverified",
        "UNSUPPORTED no limit switch or position feedback", "printStatus()",
    ):
        assert token in safe, token
    assert "delayMicroseconds(365)" not in safe
    assert "for (int i = 0; i < 12800" not in safe

    reference = REFERENCE.read_text(encoding="utf-8")
    assert "for (int i = 0; i < 12800; i++)" in reference
    assert "delayMicroseconds(10);" in reference and "delayMicroseconds(365);" in reference
    print("PASS Uno lift: original preserved, nonblocking jog/stop/status adapter contract")


if __name__ == "__main__":
    main()
