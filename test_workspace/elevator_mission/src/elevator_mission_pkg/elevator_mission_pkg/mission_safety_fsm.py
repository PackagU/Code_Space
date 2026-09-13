"""Pure, hardware-free safety state machine for staged mission integration.

This module deliberately performs no ROS I/O.  A ROS adapter may publish
``drive_inhibit_reason`` on ``/mission/drive_inhibit`` and dispatch the
listed subsystem action only after the corresponding state is entered.
"""

from dataclasses import asdict, dataclass
from enum import Enum
import math


class Mode(str, Enum):
    DRIVE_ONLY = "DRIVE_ONLY"
    ARM_BENCH = "ARM_BENCH"
    LIFT_BENCH = "LIFT_BENCH"
    MISSION_DRY_RUN = "MISSION_DRY_RUN"
    FULL_MISSION = "FULL_MISSION"


class State(str, Enum):
    BOOT_SAFE = "BOOT_SAFE"
    PREFLIGHT = "PREFLIGHT"
    IDLE_STOPPED = "IDLE_STOPPED"
    NAVIGATING = "NAVIGATING"
    STOPPING = "STOPPING"
    VERIFY_STOP = "VERIFY_STOP"
    ARM_STOW = "ARM_STOW"
    ARM_READY = "ARM_READY"
    ARM_PRESS = "ARM_PRESS"
    LIFT_JOG = "LIFT_JOG"
    VERIFY_LIFT_RESULT = "VERIFY_LIFT_RESULT"
    MANUAL_ELEVATOR_TRANSFER = "MANUAL_ELEVATOR_TRANSFER"
    SELECT_MAP = "SELECT_MAP"
    LOCALIZE = "LOCALIZE"
    FAULT = "FAULT"
    CANCELLED = "CANCELLED"
    ESTOP = "ESTOP"
    WAIT_OPERATOR = "WAIT_OPERATOR"


class Subsystem(str, Enum):
    DRIVE = "drive"
    ARM = "arm"
    LIFT = "lift"
    TRANSFER = "manual_transfer"


@dataclass(frozen=True)
class Request:
    request_id: str
    subsystem: Subsystem
    target: str
    started_at: float
    deadline: float
    session_id: str
    simulated: bool = False
    manual: bool = False


SEQUENCES = {
    Subsystem.DRIVE: (State.NAVIGATING, State.STOPPING, State.VERIFY_STOP, State.IDLE_STOPPED),
    Subsystem.ARM: (
        State.VERIFY_STOP,
        State.ARM_STOW,
        State.ARM_READY,
        State.ARM_PRESS,
        State.ARM_STOW,
        State.IDLE_STOPPED,
    ),
    Subsystem.LIFT: (
        State.VERIFY_STOP,
        State.LIFT_JOG,
        State.VERIFY_LIFT_RESULT,
        State.IDLE_STOPPED,
    ),
    Subsystem.TRANSFER: (
        State.STOPPING,
        State.MANUAL_ELEVATOR_TRANSFER,
        State.SELECT_MAP,
        State.LOCALIZE,
        State.IDLE_STOPPED,
    ),
}

ALLOWED = {
    Mode.DRIVE_ONLY: {Subsystem.DRIVE, Subsystem.TRANSFER},
    Mode.ARM_BENCH: {Subsystem.ARM},
    Mode.LIFT_BENCH: {Subsystem.LIFT},
    Mode.MISSION_DRY_RUN: set(Subsystem),
}

NEXT_ACTION = {
    State.BOOT_SAFE: "run preflight; no hardware action",
    State.PREFLIGHT: "verify fresh stopped feedback",
    State.IDLE_STOPPED: "wait for an explicit request",
    State.NAVIGATING: "track Nav2 action result",
    State.STOPPING: "cancel goal and assert gate stop",
    State.VERIFY_STOP: "require fresh continuous stopped feedback",
    State.ARM_STOW: "dispatch/verify explicit arm stow step",
    State.ARM_READY: "verify arm readiness",
    State.ARM_PRESS: "dispatch/verify requested arm cycle",
    State.LIFT_JOG: "dispatch bounded pulse jog",
    State.VERIFY_LIFT_RESULT: "record pulses; movement remains unverified",
    State.MANUAL_ELEVATOR_TRANSFER: "wait for explicit operator transfer event",
    State.SELECT_MAP: "select target-floor map and invalidate old goals",
    State.LOCALIZE: "require fresh localization success",
    State.FAULT: "manual inspection and explicit recovery",
    State.CANCELLED: "verify physical stop, then explicit recovery",
    State.ESTOP: "keep stopped; physical reset and explicit recovery",
    State.WAIT_OPERATOR: "operator action required",
}


class MissionSafetyFSM:
    """Request-scoped FSM; stale completions and implicit recovery fail closed."""

    def __init__(self, session_id, current_floor="", current_map=""):
        if not str(session_id).strip():
            raise ValueError("session_id is required")
        self.session_id = str(session_id)
        self.mode = Mode.DRIVE_ONLY
        self.state = State.BOOT_SAFE
        self.current_floor = str(current_floor)
        self.current_map = str(current_map)
        self.active_request = None
        self._sequence = ()
        self._index = -1
        self.blocked_reason = "startup is motionless until preflight"
        self.result_reason = "not run"

    def preflight(self, stopped_verified):
        if self.state not in {State.BOOT_SAFE, State.PREFLIGHT, State.WAIT_OPERATOR}:
            raise RuntimeError("preflight is only valid before operation")
        self.state = State.PREFLIGHT
        if not stopped_verified:
            self.state = State.WAIT_OPERATOR
            self.blocked_reason = "fresh physical stop feedback required"
            return False
        self.state = State.IDLE_STOPPED
        self.blocked_reason = ""
        self.result_reason = "preflight passed; no motion commanded"
        return True

    def set_mode(self, mode):
        mode = Mode(mode)
        if self.state != State.IDLE_STOPPED or self.active_request is not None:
            raise RuntimeError("mode change requires IDLE_STOPPED with no active request")
        if mode == Mode.FULL_MISSION:
            raise RuntimeError("FULL_MISSION is disabled until physical gates pass")
        self.mode = mode

    def begin(
        self,
        request_id,
        subsystem,
        target,
        now,
        timeout_sec,
        *,
        simulated=False,
        manual=False,
    ):
        subsystem = Subsystem(subsystem)
        if self.state != State.IDLE_STOPPED or self.active_request is not None:
            raise RuntimeError("another request or recovery is active")
        if subsystem not in ALLOWED[self.mode]:
            raise RuntimeError(f"{subsystem.value} is not allowed in {self.mode.value}")
        if self.mode == Mode.MISSION_DRY_RUN and not simulated:
            raise RuntimeError("MISSION_DRY_RUN accepts simulated requests only")
        if subsystem == Subsystem.TRANSFER and not manual:
            raise RuntimeError("floor transfer requires an explicit manual event")
        if not str(request_id).strip() or not str(target).strip():
            raise ValueError("request_id and target are required")
        if not all(math.isfinite(v) for v in (now, timeout_sec)) or timeout_sec <= 0.0:
            raise ValueError("time values must be finite and timeout_sec positive")

        self.active_request = Request(
            request_id=str(request_id),
            subsystem=subsystem,
            target=str(target),
            started_at=float(now),
            deadline=float(now + timeout_sec),
            session_id=self.session_id,
            simulated=bool(simulated),
            manual=bool(manual),
        )
        self._sequence = SEQUENCES[subsystem]
        self._index = 0
        self.state = self._sequence[0]
        self.blocked_reason = NEXT_ACTION[self.state]
        self.result_reason = "running"
        return self.status(now)

    def advance(
        self,
        request_id,
        session_id,
        now,
        *,
        success,
        reason,
        stopped_verified=False,
        floor=None,
        map_id=None,
    ):
        self._validate_result(request_id, session_id, now)
        if not success:
            return self.fail(f"{self.state.value}: {reason}")
        if self.state in {State.STOPPING, State.VERIFY_STOP} and not stopped_verified:
            self.blocked_reason = "fresh physical stop feedback required"
            return False
        if self.state == State.SELECT_MAP:
            if not str(floor or "").strip() or not str(map_id or "").strip():
                raise RuntimeError("SELECT_MAP completion requires floor and map_id")
            self.current_floor = str(floor)
            self.current_map = str(map_id)

        self._index += 1
        self.state = self._sequence[self._index]
        self.result_reason = str(reason)
        if self.state == State.IDLE_STOPPED:
            self.active_request = None
            self._sequence = ()
            self._index = -1
            self.blocked_reason = ""
        else:
            self.blocked_reason = NEXT_ACTION[self.state]
        return True

    def tick(self, now):
        if self.active_request is not None and now > self.active_request.deadline:
            self.fail(f"deadline exceeded in {self.state.value}")
            return False
        return True

    def cancel(self, reason="operator cancel"):
        self.active_request = None
        self._sequence = ()
        self._index = -1
        self.state = State.CANCELLED
        self.result_reason = str(reason)
        self.blocked_reason = "cancelled; physical stop and explicit recovery required"

    def estop(self, reason="E-Stop asserted"):
        self.active_request = None
        self._sequence = ()
        self._index = -1
        self.state = State.ESTOP
        self.result_reason = str(reason)
        self.blocked_reason = "E-Stop reset, physical inspection, and explicit recovery required"

    def fail(self, reason):
        self.active_request = None
        self._sequence = ()
        self._index = -1
        self.state = State.FAULT
        self.result_reason = str(reason)
        self.blocked_reason = "fault; no automatic resume/stow/lift release"
        return False

    def recover(self, *, operator_ack, stopped_verified):
        if self.state not in {State.FAULT, State.CANCELLED, State.ESTOP, State.WAIT_OPERATOR}:
            raise RuntimeError("recovery is not active")
        if not operator_ack or not stopped_verified:
            raise RuntimeError("operator acknowledgement and fresh stop feedback are required")
        self.state = State.IDLE_STOPPED
        self.blocked_reason = ""
        self.result_reason = "explicit recovery completed"

    @property
    def drive_inhibit_reason(self):
        if self.mode in {Mode.ARM_BENCH, Mode.LIFT_BENCH, Mode.MISSION_DRY_RUN}:
            return f"mode={self.mode.value}"
        if self.state not in {State.IDLE_STOPPED, State.NAVIGATING}:
            request_id = self.active_request.request_id if self.active_request else "none"
            return f"state={self.state.value};request_id={request_id}"
        return ""

    def status(self, now):
        elapsed = 0.0
        request = None
        active_subsystem = ""
        if self.active_request is not None:
            elapsed = max(0.0, float(now) - self.active_request.started_at)
            request = asdict(self.active_request)
            request["subsystem"] = self.active_request.subsystem.value
            active_subsystem = self.active_request.subsystem.value
        return {
            "mode": self.mode.value,
            "state": self.state.value,
            "current_floor": self.current_floor,
            "map": self.current_map,
            "active_request": request,
            "active_subsystem": active_subsystem,
            "blocked_reason": self.blocked_reason,
            "elapsed_sec": elapsed,
            "next_action": NEXT_ACTION[self.state],
            "result_reason": self.result_reason,
            "drive_inhibit_reason": self.drive_inhibit_reason,
        }

    def _validate_result(self, request_id, session_id, now):
        if self.active_request is None:
            raise RuntimeError("late result rejected: no active request")
        if session_id != self.session_id:
            raise RuntimeError("stale result rejected: session mismatch")
        if request_id != self.active_request.request_id:
            raise RuntimeError("late result rejected: request_id mismatch")
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if now > self.active_request.deadline:
            self.fail(f"deadline exceeded in {self.state.value}")
            raise RuntimeError("result rejected after deadline")
