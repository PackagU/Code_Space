#!/usr/bin/env python3
"""L1 offline test: auto switch state machine (no ROS).

Phases mirror the real node so this dry-run actually represents production:
  idle -> waiting_elevator -> loading_map -> finalizing -> ready
                                  \\-> failed (map load error; pending STAYS true)

Run:
  python3 test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "auto_floor_orchestrator_pkg"))

from auto_floor_orchestrator_pkg.auto_switch_core import AutoSwitchCore, LOAD_MAP


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def arrived(floor):
    return {"current_floor": floor, "target_floor": floor,
            "door_state": "open", "state": "ARRIVED_OPEN"}


def moving(floor, target):
    return {"current_floor": floor, "target_floor": target,
            "door_state": "closed", "state": "MOVING"}


def main():
    core = AutoSwitchCore(current_floor="F1")

    # --- initial state ---
    s = core.status()
    require(s["phase"] == "idle" and s["pending"] is False, "fresh core should be idle")
    require(s["map_loaded"] is False, "fresh core should not claim a loaded map")

    # --- elevator chatter while idle is ignored ---
    require(core.on_elevator_state(arrived("F2")) is None,
            "elevator state while idle must not trigger anything")
    require(core.status()["phase"] == "idle", "idle must stay idle on stray events")

    # --- request switch ---
    ok, _ = core.request_switch(target_floor="f2", spawn_point_id="elevator_inside")
    require(ok, "first request should be accepted")
    s = core.status()
    require(s["pending"] is True, "switch should become pending")
    require(s["phase"] == "waiting_elevator", "phase should wait for elevator")
    require(s["target_floor"] == "F2", "floor input should be normalized to upper case")
    require(s["map_loaded"] is False, "map_loaded must reset on new request")

    # --- wrong floor / closed door keeps waiting ---
    require(core.on_elevator_state(moving("F1", "F2")) is None, "moving must not trigger load")
    require(core.on_elevator_state(
        {"current_floor": "F2", "door_state": "closed"}) is None,
        "arrival with closed door must not trigger load")
    require(core.status()["pending"] is True, "still pending before arrival")

    # --- arrival triggers exactly one map load ---
    require(core.on_elevator_state(arrived("F2")) == LOAD_MAP, "arrival should request map load")
    s = core.status()
    require(s["phase"] == "loading_map", "arrival should move to loading_map")
    require(s["pending"] is True, "pending must stay true until map load succeeds")
    require(core.on_elevator_state(arrived("F2")) is None,
            "duplicate arrival during loading must not trigger a second load")

    # --- re-request while loading is rejected ---
    ok, reason = core.request_switch("F3", "elevator_inside")
    require(not ok and "loading" in reason, "request while loading must be rejected")

    # --- map load success -> finalizing (clear costmaps, initialpose) ---
    core.on_map_load_success()
    s = core.status()
    require(s["phase"] == "finalizing", "after load success node still clears costmaps")
    require(s["map_loaded"] is True, "map_loaded should be true after load success")
    require(s["pending"] is True, "pending false only when fully ready")
    require(s["current_floor"] == "F1", "current_floor flips only when ready")

    # --- finalize -> ready, legacy gate opens ---
    core.on_finalize_done()
    s = core.status()
    require(s["phase"] == "ready", "finalize should end in ready")
    require(s["pending"] is False, "ready must clear pending")
    require(s["current_floor"] == "F2", "ready must set current_floor to target")
    require(s["error"] == "", "no error on the happy path")

    # --- second trip (F2 -> F1) reuses the same core ---
    ok, _ = core.request_switch("F1", "elevator_inside")
    require(ok, "re-request after ready should be accepted")
    require(core.status()["map_loaded"] is False, "map_loaded must reset for the new trip")
    require(core.on_elevator_state(arrived("F1")) == LOAD_MAP, "second trip should also load")

    # --- failure path: pending stays true, legacy gate stays closed ---
    core.on_map_load_failure("load_map service unavailable")
    s = core.status()
    require(s["phase"] == "failed", "failure should be visible in phase")
    require(s["pending"] is True, "FAILURE MUST NOT publish pending=false")
    require(s["map_loaded"] is False, "failure must not claim a loaded map")
    require("unavailable" in s["error"], "failure reason must be in status")
    require(s["current_floor"] == "F2", "failure must not advance current_floor")

    # --- recovery: a new request after failure is allowed ---
    ok, _ = core.request_switch("F1", "elevator_inside")
    require(ok, "request after failure should be accepted for retry")

    print("PASS auto switch state machine")


if __name__ == "__main__":
    main()
