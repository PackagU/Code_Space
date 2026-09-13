#!/usr/bin/env python3
"""One targeted offline contract check for the staged mission safety FSM."""

import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent / "src" / "elevator_mission_pkg"
DRIVE_PACKAGE = Path(__file__).resolve().parents[2] / "src" / "drive_pkg"
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(DRIVE_PACKAGE))

from elevator_mission_pkg.mission_safety_fsm import (  # noqa: E402
    MissionSafetyFSM,
    Mode,
    State,
    Subsystem,
)
from drive_pkg.safety_gate import SafetyGate  # noqa: E402


def rejected(call, text):
    try:
        call()
    except RuntimeError as exc:
        assert text in str(exc), exc
    else:
        raise AssertionError(f"expected rejection containing {text!r}")


fsm = MissionSafetyFSM("session-a", current_floor="F1", current_map="f1_raw")
assert fsm.state == State.BOOT_SAFE and fsm.drive_inhibit_reason
assert fsm.preflight(stopped_verified=True)
rejected(lambda: fsm.set_mode(Mode.FULL_MISSION), "disabled")
rejected(
    lambda: fsm.begin("arm-wrong-mode", Subsystem.ARM, "cycle6", 1.0, 5.0),
    "not allowed",
)

fsm.set_mode(Mode.ARM_BENCH)
fsm.begin("arm-1", Subsystem.ARM, "cycle6", 2.0, 10.0)
assert fsm.state == State.VERIFY_STOP and "ARM_BENCH" in fsm.drive_inhibit_reason
rejected(
    lambda: fsm.advance("old", "session-a", 2.1, success=True, reason="old"),
    "request_id mismatch",
)
assert not fsm.advance("arm-1", "session-a", 2.2, success=True, reason="zero sent")
assert fsm.advance(
    "arm-1", "session-a", 2.3, success=True, reason="fresh stopped", stopped_verified=True
)
assert fsm.state == State.ARM_STOW
fsm.cancel()
assert fsm.state == State.CANCELLED and fsm.drive_inhibit_reason
rejected(lambda: fsm.recover(operator_ack=True, stopped_verified=False), "required")
fsm.recover(operator_ack=True, stopped_verified=True)

fsm.set_mode(Mode.MISSION_DRY_RUN)
rejected(
    lambda: fsm.begin("real-lift", Subsystem.LIFT, "up20", 4.0, 5.0),
    "simulated requests only",
)
fsm.begin("dry-lift", Subsystem.LIFT, "up20", 4.0, 1.0, simulated=True)
assert not fsm.tick(5.1) and fsm.state == State.FAULT
assert "automatic" in fsm.blocked_reason

gate = SafetyGate()
gate.observe_scan(10.0)
gate.observe_odom(10.0)
gate.observe_drive_ready(True, 10.0)
assert gate.filter_command(0.05, 0.0, 10.0, 10.0).command_forwarded
gate.set_operation_inhibit("arm:arm-1")
blocked = gate.filter_command(0.05, 0.0, 10.0, 10.0)
assert not blocked.command_forwarded and blocked.linear_x == 0.0
assert blocked.reason == "operation inhibit: arm:arm-1"

print("PASS mission_safety_fsm: modes, identity, stop gate, cancel, timeout, drive interlock")
