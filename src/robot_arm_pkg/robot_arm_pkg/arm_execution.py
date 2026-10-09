"""Controller-response-gated robot-arm execution contract with no ROS dependency.

Time passing is never treated as physical completion. A command completes only after
all four PRAD responses match the commanded PWM values; whether PRAD is a physical
encoder or target echo remains unverified. ``cancelled``
means the request lifecycle ended; the protocol cannot independently prove zero
mechanical motion, so ``physical_stop_verified`` remains false.
"""

from __future__ import annotations

from robot_arm_pkg import servo_protocol as sp
from robot_arm_pkg.arm_sequence import parse_arm_command


class ArmExecutionContract:
    """Deterministic state machine driven by ``submit`` and monotonic ``tick`` calls."""

    def __init__(
        self,
        driver=None,
        *,
        simulation_mode=False,
        require_homed=True,
        feedback_timeout_ms=1000,
        position_tolerance_pwm=30,
        stow_verified=False,
        camera_views=None,
        view_duration_ms=2000,
    ):
        if driver is not None and simulation_mode:
            raise ValueError("simulation_mode cannot use a physical serial driver")
        if feedback_timeout_ms <= 0:
            raise ValueError("feedback_timeout_ms must be positive")
        if position_tolerance_pwm < 0:
            raise ValueError("position_tolerance_pwm must be non-negative")
        self.driver = driver
        self.simulation_mode = bool(simulation_mode)
        self.require_homed = bool(require_homed)
        self.feedback_timeout_ms = float(feedback_timeout_ms)
        self.position_tolerance_pwm = int(position_tolerance_pwm)
        self.stow_verified = bool(stow_verified)
        self.camera_views = camera_views or {}
        self.view_duration_ms = view_duration_ms
        self.current_view = ""
        if self.camera_views:
            from robot_arm_pkg.camera_views import validate_views
            self.camera_views, self.view_duration_ms = validate_views(
                dict(self.camera_views, duration_ms=view_duration_ms)
            )
        self.state = "idle"
        self.phase = "unhomed"
        self.homed = False
        self.homing_basis = "none"
        self.active = None
        self.last_positions = None
        self._seen_request_ids = set()
        self._cycle = ()
        self._poses = None
        self._step_index = 0
        self._pose_due_ms = 0.0
        self._feedback_deadline_ms = 0.0

    @property
    def hardware_connected(self):
        return self.driver is not None

    def snapshot(self, event="snapshot", *, error="", request=None, completion_basis="none"):
        command = request if request is not None else self.active
        result = {
            "event": event,
            "state": self.state,
            "phase": self.phase,
            "request_id": command.get("request_id", "") if command else "",
            "action": command.get("action", "") if command else "",
            "hardware_connected": self.hardware_connected,
            "simulation_mode": self.simulation_mode,
            "homed": self.homed,
            "homing_basis": self.homing_basis,
            "stow_verified": self.stow_verified,
            "current_view": self.current_view,
            "feedback_semantics": "controller_position_response; encoder_vs_echo_unverified",
            "completion_basis": completion_basis,
            "physical_stop_verified": False,
            "error": error,
        }
        if command:
            for key in ("target", "button", "press_cycle", "target_request_id", "view"):
                if key in command:
                    result[key] = command[key]
        if self.last_positions is not None:
            result["measured_pwm"] = dict(self.last_positions)
        if self.state == "busy" and self._cycle:
            pose_name, _, _ = self._cycle[self._step_index]
            result["step_index"] = self._step_index
            result["pose"] = pose_name
            result["target_pwm"] = dict(self._poses[pose_name])
        return result

    def submit(self, command, now_ms):
        if not isinstance(command, dict):
            command = parse_arm_command(command)
        else:
            import json

            command = parse_arm_command(json.dumps(command))
        request_id = command["request_id"]
        if request_id in self._seen_request_ids:
            return self.snapshot("rejected", error="duplicate_request_id", request=command)
        self._seen_request_ids.add(request_id)

        if command["action"] == "cancel":
            return self._cancel(command)
        if self.state == "busy":
            return self.snapshot("rejected", error="busy", request=command)

        self.active = command
        if command["action"] in ("press", "view", "move") and self.require_homed and not self.homed:
            return self._fail("home_not_measured")
        if not self.hardware_connected and not self.simulation_mode:
            return self._fail("hardware_unavailable")

        if command["action"] == "read":
            try:
                self.last_positions = (dict(self.last_positions or sp.HOME) if self.simulation_mode
                                       else dict(self.driver.read_positions()))
            except Exception as exc:
                return self._fail("feedback_read_failed:" + type(exc).__name__)
            self.active = None
            return self.snapshot("completed", request=command, completion_basis=(
                "simulation_timing" if self.simulation_mode else "controller_position_response"))

        if command["action"] == "view":
            name = command["view"]
            if name not in self.camera_views:
                return self._fail("camera_view_not_configured")
            self._cycle = ((name, self.view_duration_ms, True),)
            self._poses = self.camera_views
            self.phase = "camera_moving"
        elif command["action"] == "move":
            self._cycle = (("manual_target", command["duration_ms"], True),)
            self._poses = {"manual_target": command["pose"]}
            self.phase = "manual_moving"
        elif command["action"] in ("home", "stow"):
            self._cycle = ((sp.HOME_POSE, sp.HOMING_DURATION_MS, True),)
            self._poses = {sp.HOME_POSE: sp.HOME}
            self.phase = "stowing"
        else:
            cycle_id = command["press_cycle"]
            self._cycle = sp.get_cycle(1 if "press_pose" in command else cycle_id)
            self._poses = (sp.calibrated_press_poses(command["press_pose"])
                           if "press_pose" in command else sp.get_poses(cycle_id))
            self.phase = "pressing"
        self.state = "busy"
        self.current_view = ""
        self._step_index = 0
        return self._start_current_step(float(now_ms), "started")

    def _cancel(self, command):
        if self.state != "busy" or self.active is None:
            return self.snapshot("rejected", error="no_active_request", request=command)
        target = command.get("target_request_id", "")
        if target and target != self.active["request_id"]:
            return self.snapshot("rejected", error="target_request_mismatch", request=command)
        cancelled_request = self.active
        if self.driver is not None:
            try:
                self.driver.stop_all()
            except Exception as exc:  # noqa: BLE001
                return self._fail(f"cancel_stop_failed:{type(exc).__name__}")
        self.state = "cancelled"
        self.phase = "stopped_unverified"
        self.homed = False
        self.current_view = ""
        self.homing_basis = "none"
        self.active = None
        return self.snapshot(
            "cancelled",
            error="mechanical_stop_not_independently_verified",
            request=cancelled_request,
        )

    def _fail(self, error):
        failed_request = self.active
        stop_error = ""
        if self.driver is not None:
            try:
                self.driver.stop_all()
            except Exception as exc:  # noqa: BLE001
                stop_error = f";stop_failed:{type(exc).__name__}"
        self.state = "failed"
        self.phase = "stopped_unverified"
        self.homed = False
        self.current_view = ""
        self.homing_basis = "none"
        self.active = None
        return self.snapshot("failed", error=error + stop_error, request=failed_request)

    def _start_current_step(self, now_ms, event="step_started"):
        pose_name, duration_ms, send = self._cycle[self._step_index]
        if send and self.driver is not None:
            try:
                self.driver.send_pose(pose_name, duration_ms, self._poses)
            except Exception as exc:  # noqa: BLE001
                return self._fail(f"serial_send_failed:{type(exc).__name__}")
        self._pose_due_ms = now_ms + duration_ms
        self._feedback_deadline_ms = self._pose_due_ms + self.feedback_timeout_ms
        return self.snapshot(event)

    def tick(self, now_ms):
        if self.state != "busy" or self.active is None:
            return None
        now_ms = float(now_ms)
        if now_ms < self._pose_due_ms:
            return None
        pose_name, _, send = self._cycle[self._step_index]
        if not self.simulation_mode:
            try:
                positions = self.driver.read_positions()
            except Exception as exc:  # noqa: BLE001
                return self._fail(f"feedback_read_failed:{type(exc).__name__}")
            self.last_positions = dict(positions)
            if not sp.positions_reached(
                self.last_positions,
                self._poses[pose_name],
                self.position_tolerance_pwm,
            ):
                if now_ms >= self._feedback_deadline_ms:
                    return self._fail("feedback_timeout")
                return None
        return self._advance(now_ms)

    def _advance(self, now_ms):
        if self._step_index + 1 < len(self._cycle):
            self._step_index += 1
            return self._start_current_step(now_ms)
        completed_request = self.active
        final_pose = self._cycle[-1][0]
        self.current_view = completed_request.get("view", "")
        if final_pose == sp.HOME_POSE:
            self.homed = True
            self.homing_basis = (
                "simulation_timing" if self.simulation_mode else "controller_position_response"
            )
        self.state = "simulated_complete" if self.simulation_mode else "completed"
        self.phase = self.current_view or ("stow" if final_pose == sp.HOME_POSE else "complete")
        self.active = None
        basis = "simulation_timing" if self.simulation_mode else "controller_position_response"
        return self.snapshot("completed", request=completed_request, completion_basis=basis)
