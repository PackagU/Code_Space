"""Delivery mission node.

Loads a mission from ``delivery_missions.yaml`` and a point registry from
``kku_nav_points.yaml``, then ticks through mission behaviors. Navigation
legs use an orthogonal router so Nav2 receives corridor waypoint goals
instead of one diagonal start-to-finish goal.
"""

from __future__ import annotations

import sys
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


def _load_point_registry(path):
    # Import lazily so the registry script's path-insert trick is unnecessary
    # when ``point_registry`` is installed alongside ``elevator_mission_pkg``.
    import importlib.util

    registry_path = Path(__file__).resolve().parents[3] / "scripts" / "point_registry.py"
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
    registry_path = Path(__file__).resolve().parents[3] / "scripts" / "orthogonal_router.py"
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

        mission_id = self.get_parameter("mission_id").value
        dry_run = bool(self.get_parameter("dry_run_nav2").value)
        root = Path(self.get_parameter("workspace_root").value)
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
        route = self.router.route(floor, start_point_id, goal_point_id)
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

    def _tick(self):
        if self._done:
            return
        if self.index >= len(self.sequence):
            self.get_logger().info("MISSION COMPLETE")
            self._done = True
            return
        behavior = self.sequence[self.index]
        status = behavior.tick()
        if status == SUCCESS:
            self.get_logger().info(f"[{self.index + 1}/{len(self.sequence)}] {behavior.name} SUCCESS")
            self.index += 1
        elif status == FAILURE:
            self.get_logger().error(f"[{self.index + 1}/{len(self.sequence)}] {behavior.name} FAILURE — aborting mission")
            self._done = True


def main():
    rclpy.init()
    node = DeliveryMissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
