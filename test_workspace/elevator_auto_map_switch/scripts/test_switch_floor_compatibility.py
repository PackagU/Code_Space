#!/usr/bin/env python3
"""L4 offline test: legacy SwitchFloor contract compatibility.

Legacy ``SwitchFloor`` (test_workspace/elevator_mission/.../behaviors.py) only
advances the mission when the /floor_orchestrator/status JSON satisfies:

    status["current_floor"] == target  and  not status.get("pending", True)

This test (a) pins that predicate against the legacy source so drift is
detected, and (b) drives the new auto switch core through every phase and
asserts the gate opens ONLY in the ready phase.

Run:
  python3 test_workspace/elevator_auto_map_switch/scripts/test_switch_floor_compatibility.py
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parents[1]
sys.path.insert(0, str(ROOT / "src" / "auto_floor_orchestrator_pkg"))

from auto_floor_orchestrator_pkg.auto_switch_core import AutoSwitchCore, LOAD_MAP

LEGACY_BEHAVIORS = (
    PROJECT_ROOT / "test_workspace" / "elevator_mission" / "src"
    / "elevator_mission_pkg" / "elevator_mission_pkg" / "behaviors.py"
)
# Gate expression used by legacy SwitchFloor.update() in the wait_ack phase,
# with ALL whitespace stripped (the check below strips the source the same way).
LEGACY_GATE_SNIPPET = (
    's.get("current_floor")==self.target_floorandnots.get("pending",True)'
)


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def legacy_gate(status, target_floor):
    """Verbatim semantics of legacy SwitchFloor's wait_ack check."""
    return status.get("current_floor") == target_floor and not status.get("pending", True)


def main():
    # --- pin the legacy predicate so silent drift breaks this test ---
    source = LEGACY_BEHAVIORS.read_text(encoding="utf-8").replace(" ", "")
    require(LEGACY_GATE_SNIPPET in source,
            "legacy SwitchFloor gate changed — re-check compatibility assumptions")
    require('"/floor_orchestrator/status"' in LEGACY_BEHAVIORS.read_text(encoding="utf-8"),
            "legacy SwitchFloor no longer reads /floor_orchestrator/status")
    require('"/floor_orchestrator_node/set_parameters"'
            in LEGACY_BEHAVIORS.read_text(encoding="utf-8"),
            "legacy SwitchFloor no longer sets params on node 'floor_orchestrator_node' — "
            "the auto orchestrator node name must match whatever legacy expects")

    # --- status payload must be a superset of the legacy manual payload ---
    core = AutoSwitchCore(current_floor="F1")
    legacy_keys = {"current_floor", "target_floor", "spawn_point_id", "pending", "mode"}
    new_required_keys = {"map_loaded", "phase", "error"}
    status = core.status()
    require(legacy_keys <= set(status), f"status misses legacy keys: {legacy_keys - set(status)}")
    require(new_required_keys <= set(status),
            f"status misses new keys: {new_required_keys - set(status)}")

    # --- the gate must stay CLOSED in every phase except ready ---
    target = "F2"
    require(not legacy_gate(core.status(), target), "gate must be closed while idle")

    core.request_switch(target, "elevator_inside")
    require(not legacy_gate(core.status(), target), "gate must be closed while waiting_elevator")

    action = core.on_elevator_state(
        {"current_floor": "F2", "door_state": "open", "state": "ARRIVED_OPEN"})
    require(action == LOAD_MAP, "arrival should trigger load")
    require(not legacy_gate(core.status(), target), "gate must be closed while loading_map")

    core.on_map_load_failure("simulated nav2 failure")
    require(not legacy_gate(core.status(), target),
            "gate must be closed after a FAILED map load (mission must not drive on a stale map)")

    # retry to the happy path
    core.request_switch(target, "elevator_inside")
    core.on_elevator_state({"current_floor": "F2", "door_state": "open"})
    core.on_map_load_success()
    require(not legacy_gate(core.status(), target), "gate must be closed while finalizing")

    core.on_finalize_done()
    require(legacy_gate(core.status(), target), "gate must OPEN when ready")
    require(core.status()["map_loaded"] is True and core.status()["phase"] == "ready",
            "ready status should carry map_loaded=true and phase=ready as agreed")

    print("PASS switch floor compatibility")


if __name__ == "__main__":
    main()
