"""Mission behaviors for the elevator delivery PoC.

Each behavior exposes ``initialise`` (called once when entered) and
``update`` (called on each tick). ``update`` returns ``RUNNING``,
``SUCCESS``, or ``FAILURE``. The sequencer in ``delivery_mission_node``
advances on ``SUCCESS`` and aborts on ``FAILURE``.

This is a minimal py_trees-shaped API so the node can later swap to
py_trees / py_trees_ros without changing the sequencer.
"""

from __future__ import annotations

import json
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False


RUNNING = "RUNNING"
SUCCESS = "SUCCESS"
FAILURE = "FAILURE"


class Behavior:
    name = "Behavior"

    def __init__(self, node):
        self.node = node
        self._initialised = False

    def initialise(self):
        self._initialised = True

    def update(self):
        return SUCCESS

    def tick(self):
        if not self._initialised:
            self.initialise()
        return self.update()


class NavigateToPoint(Behavior):
    def __init__(self, node, point, dry_run=False):
        super().__init__(node)
        self.point = point
        self.dry_run = dry_run or not NAV2_AVAILABLE
        self.name = f"NavigateToPoint:{point.floor}:{point.point_id}"
        self._goal_future = None
        self._result_future = None
        self._client = None
        self._sent = False
        self._dry_start = None

    def initialise(self):
        super().initialise()
        self._sent = False
        self._goal_future = None
        self._result_future = None
        self._dry_start = time.monotonic()
        if self.dry_run:
            self.node.get_logger().info(f"[dry-run] {self.name}")
            return
        if self._client is None:
            self._client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")

    def update(self):
        if self.dry_run:
            if time.monotonic() - self._dry_start >= 0.2:
                return SUCCESS
            return RUNNING

        if not self._sent:
            if not self._client.wait_for_server(timeout_sec=0.1):
                return RUNNING
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = self.point.x
            goal.pose.pose.position.y = self.point.y
            yaw = self.point.yaw_rad
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self._goal_future = self._client.send_goal_async(goal)
            self._sent = True
            return RUNNING

        if self._goal_future is not None and self._goal_future.done():
            handle = self._goal_future.result()
            if not handle.accepted:
                self.node.get_logger().error(f"{self.name} goal rejected")
                return FAILURE
            self._result_future = handle.get_result_async()
            self._goal_future = None
            return RUNNING

        if self._result_future is not None and self._result_future.done():
            return SUCCESS

        return RUNNING


class NavigateRoute(Behavior):
    def __init__(self, node, route, route_name, dry_run=False):
        super().__init__(node)
        self.route = list(route)
        self.route_name = route_name
        self.dry_run = dry_run or not NAV2_AVAILABLE
        self.name = f"NavigateRoute:{route_name}"
        self._goal_future = None
        self._result_future = None
        self._client = None
        self._sent = False
        self._index = 0
        self._dry_start = None

    def initialise(self):
        super().initialise()
        self._goal_future = None
        self._result_future = None
        self._sent = False
        self._index = 0
        self._dry_start = time.monotonic()
        route_ids = " -> ".join(point.point_id for point in self.route) or "(already there)"
        if self.dry_run:
            self.node.get_logger().info(f"[dry-run] {self.name}: {route_ids}")
            return
        if self._client is None:
            self._client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.node.get_logger().info(f"{self.name}: {route_ids}")

    def update(self):
        if not self.route:
            return SUCCESS

        if self.dry_run:
            if time.monotonic() - self._dry_start >= 0.2:
                return SUCCESS
            return RUNNING

        if self._index >= len(self.route):
            return SUCCESS

        if not self._sent:
            if not self._client.wait_for_server(timeout_sec=0.1):
                return RUNNING
            goal = self._make_goal(self.route[self._index])
            self._goal_future = self._client.send_goal_async(goal)
            self._sent = True
            return RUNNING

        if self._goal_future is not None and self._goal_future.done():
            handle = self._goal_future.result()
            if not handle.accepted:
                self.node.get_logger().error(f"{self.name} waypoint rejected")
                return FAILURE
            self._result_future = handle.get_result_async()
            self._goal_future = None
            return RUNNING

        if self._result_future is not None and self._result_future.done():
            self._index += 1
            self._result_future = None
            self._sent = False
            return SUCCESS if self._index >= len(self.route) else RUNNING

        return RUNNING

    def _make_goal(self, point):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = point.x
        goal.pose.pose.position.y = point.y
        yaw = point.yaw_rad
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal


class CallElevator(Behavior):
    def __init__(self, node, target_floor):
        super().__init__(node)
        self.target_floor = target_floor
        self.name = f"CallElevator:{target_floor}"
        self._publisher = None
        self._sent = False

    def initialise(self):
        super().initialise()
        self._sent = False
        if self._publisher is None:
            self._publisher = self.node.create_publisher(String, "/elevator/call", 10)

    def update(self):
        if not self._sent:
            msg = String()
            msg.data = self.target_floor
            self._publisher.publish(msg)
            self._sent = True
            self.node.get_logger().info(f"{self.name} published")
        return SUCCESS


class WaitElevatorArrived(Behavior):
    def __init__(self, node, floor, door_state="open"):
        super().__init__(node)
        self.floor = floor
        self.door_state = door_state
        self.name = f"WaitElevatorArrived:{floor}:{door_state}"
        self._last_state = None
        self._sub = None

    def initialise(self):
        super().initialise()
        self._last_state = None
        if self._sub is None:
            self._sub = self.node.create_subscription(
                String, "/elevator/state", self._on_state, 10
            )

    def _on_state(self, msg):
        try:
            self._last_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self._last_state = None

    def update(self):
        s = self._last_state
        if s is None:
            return RUNNING
        if s.get("current_floor") == self.floor and s.get("door_state") == self.door_state:
            return SUCCESS
        return RUNNING


class SwitchFloor(Behavior):
    def __init__(self, node, target_floor, spawn_point_id):
        super().__init__(node)
        self.target_floor = target_floor
        self.spawn_point_id = spawn_point_id
        self.name = f"SwitchFloor:{target_floor}:{spawn_point_id}"
        self._param_client = None
        self._request_client = None
        self._status_sub = None
        self._last_status = None
        self._phase = "set_params"
        self._param_future = None
        self._request_future = None

    def initialise(self):
        super().initialise()
        self._phase = "set_params"
        self._param_future = None
        self._request_future = None
        self._last_status = None
        if self._param_client is None:
            from rcl_interfaces.srv import SetParameters
            self._param_client = self.node.create_client(
                SetParameters, "/floor_orchestrator_node/set_parameters"
            )
        if self._request_client is None:
            self._request_client = self.node.create_client(
                Trigger, "/floor_orchestrator/request_switch"
            )
        if self._status_sub is None:
            self._status_sub = self.node.create_subscription(
                String, "/floor_orchestrator/status", self._on_status, 10
            )

    def _on_status(self, msg):
        try:
            self._last_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self._last_status = None

    def update(self):
        from rcl_interfaces.srv import SetParameters

        if self._phase == "set_params":
            if not self._param_client.wait_for_service(timeout_sec=0.1):
                return RUNNING
            req = SetParameters.Request()
            req.parameters = [
                Parameter("target_floor", value=self.target_floor).to_parameter_msg(),
                Parameter("spawn_point_id", value=self.spawn_point_id).to_parameter_msg(),
            ]
            self._param_future = self._param_client.call_async(req)
            self._phase = "wait_params"
            return RUNNING

        if self._phase == "wait_params":
            if not self._param_future.done():
                return RUNNING
            self._phase = "request"
            return RUNNING

        if self._phase == "request":
            if not self._request_client.wait_for_service(timeout_sec=0.1):
                return RUNNING
            self._request_future = self._request_client.call_async(Trigger.Request())
            self._phase = "wait_request"
            return RUNNING

        if self._phase == "wait_request":
            if not self._request_future.done():
                return RUNNING
            res = self._request_future.result()
            if not res.success:
                self.node.get_logger().error(f"{self.name} request rejected: {res.message}")
                return FAILURE
            self.node.get_logger().warning(
                f"{self.name} WAITING_FOR_USER_ACK target={self.target_floor}. "
                "Relaunch Gazebo/Nav2 then call /floor_orchestrator/ack."
            )
            self._phase = "wait_ack"
            return RUNNING

        if self._phase == "wait_ack":
            s = self._last_status
            if s is None:
                return RUNNING
            if s.get("current_floor") == self.target_floor and not s.get("pending", True):
                self.node.get_logger().info(f"{self.name} acknowledged")
                return SUCCESS
            return RUNNING

        return FAILURE


class Relocalize(Behavior):
    def __init__(self, node, point):
        super().__init__(node)
        self.point = point
        self.name = f"Relocalize:{point.floor}:{point.point_id}"
        self._publisher = None
        self._sent = False

    def initialise(self):
        super().initialise()
        self._sent = False
        if self._publisher is None:
            self._publisher = self.node.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 10
            )

    def update(self):
        if not self._sent:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.pose.position.x = self.point.x
            msg.pose.pose.position.y = self.point.y
            yaw = self.point.yaw_rad
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            cov = [0.0] * 36
            cov[0] = 0.25
            cov[7] = 0.25
            cov[35] = 0.06853891909122467
            msg.pose.covariance = cov
            self._publisher.publish(msg)
            self._sent = True
            self.node.get_logger().info(f"{self.name} initialpose published")
        return SUCCESS


class WaitForAck(Behavior):
    """MVP: immediate success. Logs the event name for operator visibility."""

    def __init__(self, node, event_name):
        super().__init__(node)
        self.event_name = event_name
        self.name = f"WaitForAck:{event_name}"

    def update(self):
        self.node.get_logger().info(f"{self.name} (mock ack)")
        return SUCCESS
