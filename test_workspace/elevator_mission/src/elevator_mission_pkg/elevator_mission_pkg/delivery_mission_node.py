"""Delivery mission node.

Loads a mission from ``delivery_missions.yaml`` and a point registry from
``kku_nav_points.yaml``, then ticks through mission behaviors. Navigation
legs use an orthogonal router so Nav2 receives corridor waypoint goals
instead of one diagonal start-to-finish goal.
"""

from __future__ import annotations

import sys
import json
import time
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node

# Allow running both as ``ros2 run`` (installed) and from source via
# ``python3 -m`` during local testing.
try:
    from elevator_mission_pkg.behaviors import (
        SUCCESS, FAILURE, RUNNING,
        NavigateRoute, CallElevator, WaitElevatorArrived,
        SwitchFloor, Relocalize, WaitForAck,
    )
    from elevator_mission_pkg.mission_model import load_mission
except ImportError:  # pragma: no cover - fallback for unbuilt source tree
    HERE = Path(__file__).resolve().parent
    sys.path.insert(0, str(HERE.parent))
    from elevator_mission_pkg.behaviors import (  # noqa: F401
        SUCCESS, FAILURE, RUNNING,
        NavigateRoute, CallElevator, WaitElevatorArrived,
        SwitchFloor, Relocalize, WaitForAck,
    )
    from elevator_mission_pkg.mission_model import load_mission


def _helper_script(name):
    source_path = Path(__file__).resolve().parents[3] / "scripts" / name
    if source_path.exists():
        return source_path
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("elevator_mission_pkg")) / "scripts" / name


def _load_point_registry(path):
    # Import lazily so the registry script's path-insert trick is unnecessary
    # when ``point_registry`` is installed alongside ``elevator_mission_pkg``.
    import importlib.util

    registry_path = _helper_script("point_registry.py")
    if not registry_path.exists():
        raise FileNotFoundError(
            f"point_registry.py not found at {registry_path}. "
            "Pass --ros-args -p points_yaml:=<abs path> and ensure scripts/ is reachable."
        )
    spec = importlib.util.spec_from_file_location("point_registry", registry_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PointRegistry.from_file(path)


def _load_orthogonal_router(registry):
    registry_path = _helper_script("orthogonal_router.py")
    if not registry_path.exists():
        raise FileNotFoundError(
            f"orthogonal_router.py not found at {registry_path}. "
            "Ensure test_workspace/elevator_mission/scripts/ is reachable."
        )
    import importlib.util

    spec = importlib.util.spec_from_file_location("orthogonal_router", registry_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.OrthogonalRouter(registry)


class DeliveryMissionNode(Node):
    def __init__(self):
        super().__init__("delivery_mission_node")
        self.declare_parameter("mission_id", "parcel_to_208")
        self.declare_parameter("dry_run_nav2", False)
        default_root = str(Path(__file__).resolve().parents[3])
        self.declare_parameter("workspace_root", default_root)
        self.declare_parameter("points_yaml", "")
        self.declare_parameter("missions_yaml", "")
        self.declare_parameter("tick_period_sec", 0.5)
        self.declare_parameter("camera_elevator_mode", False)
        self.declare_parameter("require_door_confirmation", False)
        self.declare_parameter("floor_observation_topic", "/elevator/vision_floor")
        self.declare_parameter("door_state_topic", "/elevator/door_state")
        self.declare_parameter("call_press_cycle", 1)
        self.declare_parameter("destination_press_cycle", 2)
        self.declare_parameter("vision_button_press", True)
        self.declare_parameter("camera_web_url", "http://127.0.0.1:8091")
        self.declare_parameter("routing_mode", "orthogonal")
        self.declare_parameter("observed_target_floor", "")
        self.declare_parameter('field_map_guard', False)
        self.declare_parameter('field_project_root', '')
        self.declare_parameter('field_pins', '')
        self.declare_parameter('field_registry', '')

        mission_id = self.get_parameter("mission_id").value
        dry_run = bool(self.get_parameter("dry_run_nav2").value)
        root = Path(self.get_parameter("workspace_root").value or default_root)
        if not (root / "config").is_dir():
            from ament_index_python.packages import get_package_share_directory
            root = Path(get_package_share_directory("elevator_mission_pkg"))
        points_yaml = self.get_parameter("points_yaml").value or str(root / "config" / "kku_nav_points.yaml")
        missions_yaml = self.get_parameter("missions_yaml").value or str(root / "config" / "delivery_missions.yaml")

        self.get_logger().info(
            f"loading mission={mission_id} dry_run_nav2={dry_run} "
            f"points={points_yaml} missions={missions_yaml}"
        )

        self.mission = load_mission(missions_yaml, mission_id)
        self.registry = _load_point_registry(points_yaml)
        self.router = _load_orthogonal_router(self.registry)
        self.dry_run = dry_run
        self.camera_mode = bool(self.get_parameter("camera_elevator_mode").value)
        if self.camera_mode:
            from elevator_mission_pkg.behaviors import NAV2_AVAILABLE
            if not NAV2_AVAILABLE:
                raise RuntimeError("camera mission requires nav2_msgs and rclpy.action")
            if dry_run:
                raise ValueError("camera_elevator_mode requires real Nav2 action results")
            from elevator_mission_pkg.elevator_camera import ElevatorArrivalGate
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, DurabilityPolicy
            self.arrival_gate = ElevatorArrivalGate(
                self.get_parameter('observed_target_floor').value or self.mission.target_floor,
                require_door_confirmation=bool(self.get_parameter("require_door_confirmation").value))
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._inhibit_pub = self.create_publisher(String, "/mission/drive_inhibit", qos)
            self._camera_state_pub = self.create_publisher(String, "/elevator/camera_state", 10)
            self.create_subscription(String, self.get_parameter("floor_observation_topic").value,
                                     self._on_floor_observation, 10)
            self.create_subscription(String, '/delivery_mission/stop', self._on_stop, 10)
            if self.arrival_gate.require_door_confirmation:
                self.create_subscription(String, self.get_parameter("door_state_topic").value,
                                         self._on_door_state, 10)
            self.set_drive_inhibit("camera_mission_startup")

        self.sequence = self._build_sequence()
        self.index = 0
        self.get_logger().info(
            "mission steps: " + ", ".join(b.name for b in self.sequence)
        )

        period = float(self.get_parameter("tick_period_sec").value)
        self.create_timer(period, self._tick)
        self._done = False

    def _build_sequence(self):
        m = self.mission
        start_pose_id = self.registry.initial_pose_id(m.start_floor)
        if self.camera_mode:
            from elevator_mission_pkg.camera_behaviors import (
                ArmCommand, WaitDoorOpen, GuardedNavigate, PrepareCameraExit, VisionButtonPress, WaitCameraWebsite,
            )
            direction = "UP" if int(m.target_floor.lstrip("F")) > int(m.start_floor.lstrip("F")) else "DOWN"
            boarding = self._navigate_route(
                m.start_floor, m.elevator_entry_point, m.elevator_inside_point)
            boarding_steps = ([WaitDoorOpen(self), GuardedNavigate(self, boarding)]
                              if self.arrival_gate.require_door_confirmation else [boarding])
            def press(target, button, cycle):
                if bool(self.get_parameter('vision_button_press').value):
                    return VisionButtonPress(self, target, button, cycle, self.arrival_gate.target)
                return ArmCommand(self, 'press', target=target, button=button, press_cycle=cycle)
            return [
                WaitCameraWebsite(self),
                ArmCommand(self, "home"),
                ArmCommand(self, "view", view="front_view"),
                self._navigate_route(m.start_floor, start_pose_id, m.pickup_point),
                WaitForAck(self, m.mock_load_event),
                self._navigate_route(m.start_floor, m.pickup_point, m.elevator_entry_point),
                press('call', direction, int(self.get_parameter('call_press_cycle').value)),
                ArmCommand(self, "view", view="front_view"),
                *boarding_steps,
                press('destination', self.arrival_gate.target, int(self.get_parameter('destination_press_cycle').value)),
                PrepareCameraExit(self, m.target_floor, m.elevator_inside_point,
                                  self._navigate_single_goal(m.target_floor, m.elevator_exit_point)),
                self._navigate_route(m.target_floor, m.elevator_exit_point, m.destination_point),
                WaitForAck(self, m.mock_delivery_event),
            ]
        return [
            self._navigate_route(m.start_floor, start_pose_id, m.pickup_point),
            WaitForAck(self, m.mock_load_event),
            self._navigate_route(m.start_floor, m.pickup_point, m.elevator_entry_point),
            CallElevator(self, m.start_floor),
            WaitElevatorArrived(self, m.start_floor, "open"),
            self._navigate_route(m.start_floor, m.elevator_entry_point, m.elevator_inside_point),
            CallElevator(self, m.target_floor),
            SwitchFloor(self, m.target_floor, m.elevator_inside_point),
            WaitElevatorArrived(self, m.target_floor, "open"),
            Relocalize(self, self.registry.get(m.target_floor, m.elevator_exit_point)),
            self._navigate_single_goal(m.target_floor, m.elevator_exit_point),
            self._navigate_route(m.target_floor, m.elevator_exit_point, m.destination_point),
            WaitForAck(self, m.mock_delivery_event),
        ]

    def _navigate_route(self, floor, start_point_id, goal_point_id):
        mode = self.get_parameter('routing_mode').value
        if mode not in ('nav2', 'orthogonal'):
            raise ValueError('routing_mode must be nav2 or orthogonal')
        route = ([self.registry.get(floor, goal_point_id)] if mode == 'nav2'
                 else self.router.route(floor, start_point_id, goal_point_id))
        return NavigateRoute(
            self,
            route,
            route_name=f"{floor}:{start_point_id}->{goal_point_id}",
            dry_run=self.dry_run,
        )

    def _navigate_single_goal(self, floor, goal_point_id):
        goal = self.registry.get(floor, goal_point_id)
        return NavigateRoute(
            self,
            [goal],
            route_name=f"{floor}:{goal_point_id}",
            dry_run=self.dry_run,
        )

    def check_navigation_goal(self, point):
        if not bool(self.get_parameter('field_map_guard').value):
            return True
        root = Path(self.get_parameter('field_project_root').value)
        name = point.point_id
        if name in ('elevator_entry', 'elevator_inside', 'elevator_exit'):
            name = point.floor.lower() + '_' + name
        try:
            result = subprocess.run([sys.executable, str(root/'scripts/field_map_guard.py'),
                'check', '--stage', 'goal', '--waypoint', name,
                '--pins', self.get_parameter('field_pins').value,
                '--registry', self.get_parameter('field_registry').value],
                cwd=root, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return True
            self.get_logger().error('field map identity check failed: ' + (result.stdout+result.stderr)[-1200:])
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error('field map identity check unavailable: ' + str(exc))
        self.set_drive_inhibit('field_map_identity_failed')
        return False

    def _tick(self):
        if self.camera_mode:
            self._publish_camera_state()
        if self._done:
            return
        if self.index >= len(self.sequence):
            self.get_logger().info("MISSION COMPLETE")
            self._done = True
            if self.camera_mode:
                self.set_drive_inhibit("mission_complete")
            return
        behavior = self.sequence[self.index]
        if self.camera_mode:
            from elevator_mission_pkg.camera_behaviors import GuardedNavigate, PrepareCameraExit
            if isinstance(behavior, NavigateRoute):
                self.set_drive_inhibit("")
            elif not isinstance(behavior, (GuardedNavigate, PrepareCameraExit)):
                self.set_drive_inhibit("mission_stationary_step")
        status = behavior.tick()
        if status == SUCCESS:
            self.get_logger().info(f"[{self.index + 1}/{len(self.sequence)}] {behavior.name} SUCCESS")
            self.index += 1
        elif status == FAILURE:
            self.get_logger().error(f"[{self.index + 1}/{len(self.sequence)}] {behavior.name} FAILURE — aborting mission")
            self._done = True
            if self.camera_mode:
                self.set_drive_inhibit("mission_failed")

    def set_drive_inhibit(self, reason):
        from std_msgs.msg import String
        msg = String()
        msg.data = reason
        self._inhibit_pub.publish(msg)

    def _on_floor_observation(self, msg):
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            state = {}
        self.arrival_gate.observe_floor(state, time.monotonic(), time.time())

    def _on_stop(self, msg):
        self.shutdown()
        self._done = True
        self._publish_camera_state()

    def _on_door_state(self, msg):
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            state = {}
        self.arrival_gate.observe_door(state, time.monotonic(), time.time())

    def _publish_camera_state(self):
        from std_msgs.msg import String
        gate = self.arrival_gate
        now, wall = time.monotonic(), time.time()
        ready = gate.confirmed(now, wall) if gate.looking else gate.can_exit(now, wall)
        msg = String()
        msg.data = json.dumps({"current_floor": self.mission.target_floor if ready else "UNKNOWN",
                               "observed_floor": gate.target if ready else "UNKNOWN",
                               "door_state": "camera_confirmed" if ready else "unknown",
                               "source": "camera_verified", "stamp": wall,
                               "target_floor": gate.target, "mission_id": self.mission.mission_id,
                               "map_floor": self.mission.target_floor,
                               "mission_active": not self._done,
                               'confirmed_frames': gate.count,
                               'required_frames': gate.required_frames,
                               "step": self.sequence[self.index].name if self.index < len(self.sequence) else 'complete',
                               "exit_condition_met": bool(ready)})
        self._camera_state_pub.publish(msg)

    def shutdown(self):
        if self.camera_mode:
            self._done = True
            if not self.context.ok():
                return
            self.set_drive_inhibit("mission_stopped")
            if self.index < len(self.sequence) and hasattr(self.sequence[self.index], "cancel"):
                self.sequence[self.index].cancel()
            self._publish_camera_state()


def main():
    from rclpy.signals import SignalHandlerOptions
    # Keep DDS alive long enough to publish stop/cancel before destroying the context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DeliveryMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
