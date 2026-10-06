"""Fresh camera evidence for target-floor exit, with optional door confirmation."""
import math
import re


def floor_id(value):
    text = str(value).strip().upper()
    match = re.fullmatch(r"F?([0-9]{1,2}|B[0-9]{1,2})", text)
    if not match:
        return "UNKNOWN"
    value = match.group(1)
    return "B" + str(int(value[1:])) if value.startswith("B") else "F" + str(int(value))


def fresh_stamp(stamp, now, max_age):
    return (type(stamp) in (int, float) and math.isfinite(stamp)
            and 0 <= now - stamp <= max_age)


def reader_observation(state, wall_now, max_age=.8):
    """Use live frames, never the reader's historical TARGET_FLOOR_DETECTED event."""
    if not isinstance(state, dict):
        return {"valid": False}
    stream, seq = state.get("stream_id"), state.get("frame_seq")
    score, margin = state.get("score"), state.get("margin")
    valid = (not state.get("error") and bool(state.get("roi"))
             and isinstance(stream, str) and 0 < len(stream) <= 64
             and type(seq) is int and seq > 0
             and fresh_stamp(state.get("updated"), wall_now, max_age)
             and type(score) in (int, float) and math.isfinite(score) and score >= .70
             and type(margin) in (int, float) and math.isfinite(margin) and margin >= .08
             and floor_id(state.get("floor")) != "UNKNOWN")
    return {"valid": bool(valid), "floor": floor_id(state.get("floor")),
            "captured_at": state.get("updated"), "stream_id": stream, "frame_seq": seq}


class ElevatorArrivalGate:
    def __init__(self, target, required_frames=5, max_age=.8, settle_sec=.5, hold_sec=30.,
                 require_door_confirmation=False):
        if type(required_frames) is not int or required_frames < 2:
            raise ValueError("required_frames must be at least two")
        if not all(math.isfinite(x) and x > 0 for x in (max_age, hold_sec)):
            raise ValueError("freshness and hold time must be positive")
        if not math.isfinite(settle_sec) or settle_sec < 0:
            raise ValueError("settle_sec must be non-negative")
        self.target = floor_id(target)
        if self.target == "UNKNOWN":
            raise ValueError("invalid target floor")
        self.required_frames, self.max_age = required_frames, max_age
        self.settle_sec, self.hold_sec = settle_sec, hold_sec
        if type(require_door_confirmation) is not bool:
            raise ValueError("require_door_confirmation must be a boolean")
        self.require_door_confirmation = require_door_confirmation
        self.looking = False
        self.cutoff = float("inf")
        self.count, self.floor_seen = 0, None
        self.stream, self.seq, self.capture = None, 0, 0.
        self.door_seen, self.door_stamp = None, 0.
        self.door_ok = False
        self.epoch = 0
        self.arrival = None

    def _invalidate_door(self):
        self.door_ok = False
        self.epoch += 1
        self.arrival = None

    def observe_door(self, state, now, wall_now):
        if not self.require_door_confirmation:
            return
        if self.door_seen is not None and now - self.door_seen > self.max_age:
            self._invalidate_door()
        if (not isinstance(state, dict) or state.get("source") != "sensor"
                or not fresh_stamp(state.get("stamp"), wall_now, self.max_age)):
            self._invalidate_door()
            return
        # Replaying one sensor sample must not refresh the receipt clock.
        if state["stamp"] <= self.door_stamp:
            return
        self.door_stamp, self.door_seen = state["stamp"], now
        self.door_ok = (state.get("door_state") == "open" and state.get("stopped") is True
                        and state.get("exit_clear") is True)
        if not self.door_ok:
            self._invalidate_door()

    def door_safe(self, now, wall_now):
        if not self.require_door_confirmation:
            return True
        if (self.door_seen is None or now - self.door_seen > self.max_age
                or not fresh_stamp(self.door_stamp, wall_now, self.max_age)):
            if self.door_ok:
                self._invalidate_door()
            return False
        return self.door_ok

    def begin_looking(self, wall_now):
        self.looking = True
        self.cutoff = wall_now + self.settle_sec
        self.count, self.floor_seen = 0, None
        self.stream, self.seq, self.capture = None, 0, 0.
        self.arrival = None

    def observe_floor(self, state, now, wall_now):
        if not self.looking:
            return
        if (not isinstance(state, dict) or state.get("valid") is not True
                or not fresh_stamp(state.get("captured_at"), wall_now, self.max_age)
                or state["captured_at"] < self.cutoff
                or floor_id(state.get("floor")) != self.target
                or not isinstance(state.get("stream_id"), str) or not state["stream_id"]
                or type(state.get("frame_seq")) is not int or state["frame_seq"] <= 0):
            self.count, self.floor_seen, self.arrival = 0, None, None
            return
        if state["stream_id"] != self.stream:
            self.count, self.seq, self.capture, self.arrival = 0, 0, 0., None
            self.stream = state["stream_id"]
        if state["frame_seq"] <= self.seq or state["captured_at"] <= self.capture:
            return
        if self.floor_seen is not None and now - self.floor_seen > self.max_age:
            self.count = 0
            self.arrival = None
        self.seq, self.capture, self.floor_seen = state["frame_seq"], state["captured_at"], now
        self.count += 1

    def confirmed(self, now, wall_now):
        ok = (self.looking and self.count >= self.required_frames and self.floor_seen is not None
              and now - self.floor_seen <= self.max_age
              and fresh_stamp(self.capture, wall_now, self.max_age)
              and self.door_safe(now, wall_now))
        if ok:
            self.arrival = (self.epoch, now)
        return bool(ok)

    def freeze_for_front_view(self, now, wall_now):
        if not self.confirmed(now, wall_now):
            return False
        self.looking = False
        return True

    def can_exit(self, now, wall_now):
        # Retain floor evidence briefly while turning forward and navigating out.
        # When enabled, door confirmation must remain valid throughout this window.
        return (self.door_safe(now, wall_now) and self.arrival is not None
                and self.arrival[0] == self.epoch and now - self.arrival[1] <= self.hold_sec)
