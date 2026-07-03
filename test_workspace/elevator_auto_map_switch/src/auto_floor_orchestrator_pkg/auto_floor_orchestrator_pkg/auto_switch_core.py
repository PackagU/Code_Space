"""Pure state machine for the automatic floor/map switch.

No ROS imports: the ROS node owns all side effects (service calls, topic
publishing) and reports their outcomes back into this core. Phases:

    idle -> waiting_elevator -> loading_map -> finalizing -> ready
                                     \\-> failed

Contract guarantees (consumed by legacy SwitchFloor):
- ``pending`` turns false ONLY in ready (never on failure, never mid-load).
- ``current_floor`` flips to the target ONLY in ready.
- ``map_loaded`` is true from map load success onward, false otherwise.
"""

from __future__ import annotations

# Action returned by on_elevator_state when the node must start loading the map.
LOAD_MAP = "load_map"

PHASE_IDLE = "idle"
PHASE_WAITING = "waiting_elevator"
PHASE_LOADING = "loading_map"
PHASE_FINALIZING = "finalizing"
PHASE_READY = "ready"
PHASE_FAILED = "failed"

# Phases in which a new request would race an in-flight Nav2 service call.
_BUSY_PHASES = (PHASE_LOADING, PHASE_FINALIZING)


class AutoSwitchCore:
    def __init__(self, current_floor="F1", arrival_door_state="open"):
        self.current_floor = str(current_floor).upper()
        self.target_floor = self.current_floor
        self.spawn_point_id = "elevator_inside"
        self.arrival_door_state = arrival_door_state
        self.pending = False
        self.phase = PHASE_IDLE
        self.map_loaded = False
        self.error = ""

    def request_switch(self, target_floor, spawn_point_id):
        """Start a switch. Returns (accepted, reason)."""
        if self.phase in _BUSY_PHASES:
            return False, f"busy: {self.phase} in progress, request rejected"
        self.target_floor = str(target_floor).upper()
        self.spawn_point_id = str(spawn_point_id)
        self.pending = True
        self.phase = PHASE_WAITING
        self.map_loaded = False
        self.error = ""
        return True, f"auto switch armed: target={self.target_floor}"

    def on_elevator_state(self, state):
        """Feed one /elevator/state JSON dict. Returns LOAD_MAP exactly once
        when the elevator has arrived at the target floor with the door open,
        otherwise None."""
        if self.phase != PHASE_WAITING or not self.pending:
            return None
        arrived = (
            str(state.get("current_floor", "")).upper() == self.target_floor
            and state.get("door_state") == self.arrival_door_state
        )
        if not arrived:
            return None
        self.phase = PHASE_LOADING
        return LOAD_MAP

    def on_map_load_success(self):
        self.map_loaded = True
        self.phase = PHASE_FINALIZING

    def on_map_load_failure(self, error):
        self.map_loaded = False
        self.error = str(error)
        self.phase = PHASE_FAILED
        # pending intentionally stays True: the mission must not resume.

    def on_finalize_done(self):
        self.current_floor = self.target_floor
        self.pending = False
        self.phase = PHASE_READY

    def status(self):
        return {
            "current_floor": self.current_floor,
            "target_floor": self.target_floor,
            "spawn_point_id": self.spawn_point_id,
            "pending": self.pending,
            "phase": self.phase,
            "map_loaded": self.map_loaded,
            "error": self.error,
            "mode": "auto_map_switch",
        }
