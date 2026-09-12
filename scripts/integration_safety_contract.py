#!/usr/bin/env python3
"""Pure fail-closed contract shared by field-only and future full missions."""

from dataclasses import dataclass
import math


REQUIRED_DRIVE_SOURCES = ("scan", "odom", "drive_ready")


@dataclass(frozen=True)
class Decision:
    accepted: bool
    stop_required: bool
    reason: str


class IntegrationSafetyContract:
    def __init__(self, freshness_sec=0.5, floor_event_sec=1.0):
        if not all(math.isfinite(value) and value > 0 for value in (freshness_sec, floor_event_sec)):
            raise ValueError("timeouts must be finite and positive")
        self.freshness_sec = freshness_sec
        self.floor_event_sec = floor_event_sec
        self.samples = {name: (None, False) for name in REQUIRED_DRIVE_SOURCES}
        self.active_id = None
        self.active_mode = "idle"
        self.software_stop = False
        self.session_id = None
        self.state_entered_at = None
        self.target_floor = None

    @staticmethod
    def _fresh(sample_time, now, timeout):
        if sample_time is None or not all(math.isfinite(value) for value in (sample_time, now)):
            return False
        return 0.0 <= now - sample_time <= timeout

    def observe(self, source, ready, now):
        if source not in self.samples:
            raise ValueError(f"unknown source: {source}")
        self.samples[source] = (now if math.isfinite(now) else None, bool(ready))

    def drive_readiness(self, now):
        if self.software_stop:
            return Decision(False, True, "software stop asserted")
        for source in REQUIRED_DRIVE_SOURCES:
            sample_time, ready = self.samples[source]
            if not self._fresh(sample_time, now, self.freshness_sec):
                return Decision(False, True, f"{source} stale")
            if not ready:
                return Decision(False, True, f"{source} not ready")
        return Decision(True, False, "drive sources ready")

    def start(self, request_id, mode, now, arm_power_isolated=False, arm_stowed_verified=False):
        request_id = str(request_id).strip()
        if not request_id:
            return Decision(False, True, "request id missing")
        if self.active_id is not None:
            return Decision(False, True, "duplicate start rejected")
        if mode not in ("drive_only", "full_mission"):
            return Decision(False, True, "unknown mode")
        if mode == "drive_only" and not arm_power_isolated:
            return Decision(False, True, "drive-only requires arm/lift power isolation")
        if mode == "full_mission" and not arm_stowed_verified:
            return Decision(False, True, "full mission requires measured arm stow")
        readiness = self.drive_readiness(now)
        if not readiness.accepted:
            return readiness
        self.active_id = request_id
        self.active_mode = mode
        return Decision(True, False, "start accepted")

    def health(self, now):
        readiness = self.drive_readiness(now)
        if not readiness.accepted:
            self.active_id = None
            self.active_mode = "idle"
        return readiness

    def interrupt(self, reason="operator interrupt"):
        self.active_id = None
        self.active_mode = "idle"
        self.software_stop = True
        return Decision(False, True, str(reason))

    def reset_stop(self):
        self.software_stop = False
        return Decision(False, True, "stop reset; fresh readiness required")

    def enter_floor_wait(self, session_id, target_floor, now):
        if self.active_mode != "full_mission":
            return Decision(False, True, "floor event unavailable outside full mission")
        self.session_id = str(session_id)
        self.target_floor = str(target_floor).upper()
        self.state_entered_at = now
        return Decision(True, False, "waiting for fresh floor event")

    def accept_floor_event(self, event, now):
        try:
            event_time = float(event["observed_at"])
        except (KeyError, TypeError, ValueError):
            return Decision(False, True, "floor event timestamp invalid")
        if event.get("session_id") != self.session_id:
            return Decision(False, True, "floor event session mismatch")
        if self.state_entered_at is None or event_time < self.state_entered_at:
            return Decision(False, True, "floor event predates state entry")
        if not self._fresh(event_time, now, self.floor_event_sec):
            return Decision(False, True, "floor event stale")
        if str(event.get("floor", "")).upper() != self.target_floor:
            return Decision(False, True, "floor event target mismatch")
        if event.get("door_state") != "open" or event.get("elevator_stopped") is not True:
            return Decision(False, True, "door-open and stopped evidence required")
        return Decision(True, False, "fresh target-floor event accepted")
