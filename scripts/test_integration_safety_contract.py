#!/usr/bin/env python3
"""P08 duplicate, interrupt, disconnect, stale event, and mode-boundary tests."""

from integration_safety_contract import IntegrationSafetyContract


def ready(gate, now):
    for source in ("scan", "odom", "drive_ready"):
        gate.observe(source, True, now)


def main():
    gate = IntegrationSafetyContract()
    ready(gate, 1.0)

    result = gate.start("corridor-1", "drive_only", 1.1, arm_power_isolated=True)
    assert result.accepted and not result.stop_required
    duplicate = gate.start("corridor-1", "drive_only", 1.1, arm_power_isolated=True)
    assert not duplicate.accepted and duplicate.stop_required

    assert gate.health(1.7).reason == "scan stale"
    assert gate.active_mode == "idle" and gate.active_id is None
    ready(gate, 2.0)
    missing_isolation = gate.start("corridor-2", "drive_only", 2.1)
    assert not missing_isolation.accepted and "isolation" in missing_isolation.reason

    ready(gate, 3.0)
    missing_stow = gate.start("mission-1", "full_mission", 3.1)
    assert not missing_stow.accepted and "measured arm stow" in missing_stow.reason
    full = gate.start("mission-1", "full_mission", 3.1, arm_stowed_verified=True)
    assert full.accepted
    wait = gate.enter_floor_wait("session-9", "F2", 3.2)
    assert wait.accepted
    base_event = {
        "session_id": "session-9",
        "floor": "F2",
        "door_state": "open",
        "elevator_stopped": True,
    }
    stale = dict(base_event, observed_at=3.1)
    assert not gate.accept_floor_event(stale, 3.3).accepted
    wrong_session = dict(base_event, session_id="old", observed_at=3.3)
    assert not gate.accept_floor_event(wrong_session, 3.3).accepted
    moving = dict(base_event, elevator_stopped=False, observed_at=3.3)
    assert not gate.accept_floor_event(moving, 3.3).accepted
    valid = dict(base_event, observed_at=3.3)
    assert gate.accept_floor_event(valid, 3.4).accepted

    interrupted = gate.interrupt("Wi-Fi heartbeat lost")
    assert interrupted.stop_required and gate.software_stop
    gate.reset_stop()
    assert not gate.drive_readiness(4.0).accepted
    ready(gate, 4.0)
    assert gate.drive_readiness(4.1).accepted
    print("P08 integration safety contract tests passed")


if __name__ == "__main__":
    main()
