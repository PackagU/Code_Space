"""Arm views and continuously guarded elevator transfer behaviors."""
import json
import time
import uuid

from std_msgs.msg import String

from elevator_mission_pkg.behaviors import Behavior, RUNNING, SUCCESS, FAILURE, SwitchFloor


class ArmCommand(Behavior):
    def __init__(self, node, action, **fields):
        super().__init__(node)
        self.command = dict(fields, action=action, request_id="camera-" + uuid.uuid4().hex)
        self.name = "Arm:" + fields.get("view", action)
        self.status, self.sent = None, False
        self.pub = node.create_publisher(String, "/packagu_arm/command", 10)
        self.sub = node.create_subscription(String, "/packagu_arm/status", self.on_status, 10)

    def initialise(self):
        super().initialise()
        self.started = time.monotonic()

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            if isinstance(status, dict) and status.get("request_id") == self.command["request_id"]:
                self.status = status
        except (ValueError, TypeError):
            pass

    def cancel(self):
        if self.sent:
            msg = String()
            msg.data = json.dumps({"request_id": "cancel-" + uuid.uuid4().hex,
                                   "action": "cancel", "target_request_id": self.command["request_id"]})
            self.pub.publish(msg)

    def update(self):
        self.node.set_drive_inhibit("arm_moving")
        if time.monotonic() - self.started > 30:
            self.cancel()
            return FAILURE
        if not self.sent:
            if self.node.count_subscribers("/packagu_arm/command") == 0:
                return RUNNING
            msg = String()
            msg.data = json.dumps(self.command)
            self.pub.publish(msg)
            self.sent = True
        s = self.status or {}
        if s.get("event") in ("failed", "rejected", "cancelled"):
            self.node.get_logger().error(self.name + ": " + s.get("error", "arm failure"))
            return FAILURE
        if s.get("event") == "completed":
            if (s.get("completion_basis") != "controller_position_response"
                    or s.get("simulation_mode") is not False
                    or s.get("action") != self.command["action"]):
                return FAILURE
            if self.command["action"] == "view" and s.get("current_view") != self.command["view"]:
                return FAILURE
            return SUCCESS
        return RUNNING


class WaitDoorOpen(Behavior):
    name = "WaitDoorOpen:stationary:clear"

    def initialise(self):
        super().initialise()
        self.started = time.monotonic()

    def update(self):
        self.node.set_drive_inhibit("elevator_wait")
        if self.node.arrival_gate.door_safe(time.monotonic(), time.time()):
            return SUCCESS
        return FAILURE if time.monotonic() - self.started > 180 else RUNNING


class GuardedNavigate(Behavior):
    def __init__(self, node, navigation, exit_car=False):
        super().__init__(node)
        self.navigation, self.exit_car = navigation, exit_car
        self.name = ("ExitElevator:" if exit_car else "BoardElevator:") + navigation.name
        self.began = False

    def cancel(self):
        self.navigation.cancel()

    def update(self):
        gate = self.node.arrival_gate
        safe = (gate.can_exit(time.monotonic(), time.time()) if self.exit_car
                else gate.door_safe(time.monotonic(), time.time()))
        if not safe:
            self.node.set_drive_inhibit("elevator_transfer_blocked")
            if self.began:
                self.cancel()
                return FAILURE
            return RUNNING
        self.node.set_drive_inhibit("")
        self.began = True
        result = self.navigation.tick()
        if result == SUCCESS:
            self.node.set_drive_inhibit("elevator_transfer_complete")
            if self.exit_car:
                gate.arrival, gate.looking = None, False
        return result


class PrepareCameraExit(Behavior):
    """Read -> switch map -> face forward -> exit; recheck if authorization expires."""
    def __init__(self, node, floor, inside_point, navigation):
        super().__init__(node)
        self.floor, self.inside_point, self.navigation = floor, inside_point, navigation
        self.name = "CameraVerifiedExit:" + floor
        self.child = None
        self.phase = "look"

    def initialise(self):
        super().initialise()
        self.started = time.monotonic()
        self._look()

    def _look(self):
        self.phase = "look"
        self.child = ArmCommand(self.node, "view", view="floor_view")

    def cancel(self):
        if self.child is not None and hasattr(self.child, "cancel"):
            self.child.cancel()

    def update(self):
        now, wall = time.monotonic(), time.time()
        gate = self.node.arrival_gate
        if now - self.started > 360:
            self.node.set_drive_inhibit("target_floor_timeout")
            self.cancel()
            return FAILURE
        if self.phase != "exit":
            self.node.set_drive_inhibit("elevator_target_floor_wait")
        if self.phase == "wait":
            if not gate.confirmed(now, wall):
                return RUNNING
            self.phase = "map"
            self.child = SwitchFloor(self.node, self.floor, self.inside_point)
            return RUNNING
        if self.phase == "exit" and not self.child.began and not gate.can_exit(now, wall):
            self._look()
            return RUNNING
        result = self.child.tick()
        if result != SUCCESS:
            return result
        if self.phase == "look":
            gate.begin_looking(time.time())
            self.phase = "wait"
            self.child = None
        elif self.phase == "map":
            if not gate.freeze_for_front_view(now, wall):
                self._look()
            else:
                self.phase = "front"
                self.child = ArmCommand(self.node, "view", view="front_view")
        elif self.phase == "front":
            if not gate.can_exit(now, wall):
                self._look()
            else:
                self.phase = "exit"
                self.child = GuardedNavigate(self.node, self.navigation, exit_car=True)
        elif self.phase == "exit":
            return SUCCESS
        return RUNNING
