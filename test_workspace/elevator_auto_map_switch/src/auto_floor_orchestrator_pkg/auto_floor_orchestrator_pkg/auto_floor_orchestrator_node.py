"""Automatic floor orchestrator.

Drop-in replacement for the legacy manual ``floor_orchestrator_node``:
same node name (legacy SwitchFloor hardcodes
``/floor_orchestrator_node/set_parameters``), same services and status topic,
but instead of waiting for a human ``/floor_orchestrator/ack`` it watches
``/elevator/state`` and, on arrival at the target floor with the door open:

  1. calls Nav2 ``/map_server/load_map`` with the target floor's map yaml
  2. clears the global/local costmaps (service names are parameters)
  3. publishes ``/initialpose`` at the requested spawn point
  4. publishes status with current_floor=target, pending=false,
     map_loaded=true, phase=ready  ->  legacy SwitchFloor resumes the mission

``pending`` never turns false before a successful map load. On failure the
status carries phase=failed and the error text, and the mission stays blocked.

Never run this node together with the legacy manual orchestrator: they share
the node name and services on purpose.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from nav2_msgs.srv import LoadMap, ClearEntireCostmap
    NAV2_SRV_AVAILABLE = True
except ImportError:  # dry-run still works without nav2_msgs installed
    NAV2_SRV_AVAILABLE = False

from auto_floor_orchestrator_pkg.auto_switch_core import AutoSwitchCore, LOAD_MAP
from auto_floor_orchestrator_pkg.floor_map_registry import FloorMapRegistry

# parents: [0]=auto_floor_orchestrator_pkg(inner) [1]=auto_floor_orchestrator_pkg(pkg)
#          [2]=src [3]=elevator_auto_map_switch
# resolve() follows colcon --symlink-install links back to this source tree.
DEFAULT_FLOOR_MAPS_YAML = Path(__file__).resolve().parents[3] / "config" / "floor_maps.yaml"


class AutoFloorOrchestratorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("floor_orchestrator_node", **kwargs)
        self.declare_parameter("current_floor", "F1")
        self.declare_parameter("target_floor", "F2")
        self.declare_parameter("spawn_point_id", "elevator_inside")
        self.declare_parameter("dry_run_map_load", True)
        self.declare_parameter("floor_maps_yaml", "")
        self.declare_parameter("workspace_root", "")
        self.declare_parameter("elevator_state_topic", "/elevator/state")
        self.declare_parameter("arrival_door_state", "open")
        self.declare_parameter("load_map_service", "/map_server/load_map")
        self.declare_parameter(
            "costmap_clear_services",
            [
                "/global_costmap/clear_entirely_global_costmap",
                "/local_costmap/clear_entirely_local_costmap",
            ],
        )
        self.declare_parameter("publish_initialpose", True)
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("status_period_sec", 0.5)
        self.declare_parameter("service_wait_timeout_sec", 10.0)
        self.declare_parameter("load_map_timeout_sec", 15.0)

        maps_yaml = self.get_parameter("floor_maps_yaml").value or str(DEFAULT_FLOOR_MAPS_YAML)
        workspace_root = self.get_parameter("workspace_root").value or None
        self.registry = FloorMapRegistry.from_file(maps_yaml, workspace_root=workspace_root)
        self.get_logger().info(
            f"floor maps: {maps_yaml} (floors: {self.registry.floor_ids()}, "
            f"workspace_root: {self.registry.workspace_root})"
        )

        self.core = AutoSwitchCore(
            current_floor=self.get_parameter("current_floor").value,
            arrival_door_state=self.get_parameter("arrival_door_state").value,
        )

        self._map_yaml = ""
        self._initialpose_sent = False
        self._effect = None  # in-flight async Nav2 step, see _advance_effect()
        self._load_client = None
        self._clear_clients = {}

        self.status_pub = self.create_publisher(String, "/floor_orchestrator/status", 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("initialpose_topic").value, 10
        )
        self.create_subscription(
            String, self.get_parameter("elevator_state_topic").value,
            self._on_elevator_state, 10,
        )
        self.create_service(
            Trigger, "/floor_orchestrator/request_switch", self._on_request_switch
        )
        self.create_service(Trigger, "/floor_orchestrator/ack", self._on_ack)

        period = float(self.get_parameter("status_period_sec").value)
        self.create_timer(period, self._publish_status)
        self.create_timer(0.2, self._advance_effect)

    # ------------------------------------------------------------------ services

    def _on_request_switch(self, request, response):
        del request
        target = str(self.get_parameter("target_floor").value)
        spawn = str(self.get_parameter("spawn_point_id").value)

        try:
            self.registry.get(target)
        except KeyError as exc:
            response.success = False
            response.message = f"rejected: {exc}"
            self.get_logger().error(response.message)
            return response

        accepted, reason = self.core.request_switch(target, spawn)
        response.success = accepted
        if not accepted:
            response.message = f"rejected: {reason}"
            self.get_logger().error(response.message)
            return response

        self._map_yaml = ""
        self._initialpose_sent = False
        response.message = (
            f"auto floor switch armed: target={target}, spawn={spawn}. "
            "Waiting for /elevator/state arrival; no manual ack needed."
        )
        self.get_logger().info(response.message)
        self._publish_status()
        return response

    def _on_ack(self, request, response):
        del request
        phase = self.core.phase
        response.success = phase == "ready"
        response.message = (
            f"auto mode: manual ack is not used (phase={phase}). "
            "The switch completes automatically on elevator arrival."
        )
        return response

    # ------------------------------------------------------------------ elevator

    def _on_elevator_state(self, msg):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"ignoring malformed elevator state: {msg.data!r}")
            return
        action = self.core.on_elevator_state(state)
        if action != LOAD_MAP:
            return
        self.get_logger().info(
            f"elevator arrived at {self.core.target_floor} with door "
            f"{self.get_parameter('arrival_door_state').value} — starting map switch"
        )
        self._start_map_load()
        self._publish_status()

    # ------------------------------------------------------------------ map load

    def _start_map_load(self):
        try:
            self._map_yaml = self.registry.resolve_map_yaml(self.core.target_floor)
        except FileNotFoundError as exc:
            self._fail(str(exc))
            return

        if bool(self.get_parameter("dry_run_map_load").value):
            self.get_logger().info(f"[dry-run] would load map: {self._map_yaml}")
            self.core.on_map_load_success()
            self.get_logger().info("[dry-run] skipping costmap clear services")
            self._finalize()
            return

        if not NAV2_SRV_AVAILABLE:
            self._fail("nav2_msgs not available but dry_run_map_load is false")
            return

        service = self.get_parameter("load_map_service").value
        if self._load_client is None or self._load_client.srv_name != service:
            self._load_client = self.create_client(LoadMap, service)
        wait_timeout = float(self.get_parameter("service_wait_timeout_sec").value)
        self._effect = {
            "step": "load_wait_service",
            "deadline": time.monotonic() + wait_timeout,
        }

    def _advance_effect(self):
        """Drives the async Nav2 service chain; no-op while idle."""
        effect = self._effect
        if effect is None:
            return
        now = time.monotonic()
        step = effect["step"]

        if step == "load_wait_service":
            if self._load_client.service_is_ready():
                request = LoadMap.Request()
                request.map_url = self._map_yaml
                load_timeout = float(self.get_parameter("load_map_timeout_sec").value)
                effect.update(
                    step="load_wait_response",
                    future=self._load_client.call_async(request),
                    deadline=now + load_timeout,
                )
            elif now > effect["deadline"]:
                self._effect = None
                self._fail(
                    f"load_map service unavailable: {self._load_client.srv_name} "
                    "(check `ros2 service list | grep load_map` and the "
                    "load_map_service parameter)"
                )
            return

        if step == "load_wait_response":
            future = effect["future"]
            if future.done():
                result = future.result()
                if result is not None and result.result == LoadMap.Response.RESULT_SUCCESS:
                    self.get_logger().info(f"map loaded: {self._map_yaml}")
                    self.core.on_map_load_success()
                    self._publish_status()
                    self._start_costmap_clear(effect)
                else:
                    code = None if result is None else result.result
                    self._effect = None
                    self._fail(f"load_map failed (result={code}) for {self._map_yaml}")
            elif now > effect["deadline"]:
                self._effect = None
                self._fail(f"load_map response timeout for {self._map_yaml}")
            return

        if step == "clear_wait_service":
            client = effect["clear_client"]
            if client.service_is_ready():
                effect.update(
                    step="clear_wait_response",
                    future=client.call_async(ClearEntireCostmap.Request()),
                    deadline=now + float(self.get_parameter("service_wait_timeout_sec").value),
                )
            elif now > effect["deadline"]:
                self.get_logger().warning(
                    f"costmap clear service unavailable, skipping: {client.srv_name}"
                )
                self._start_costmap_clear(effect)
            return

        if step == "clear_wait_response":
            future = effect["future"]
            if future.done():
                self.get_logger().info(f"costmap cleared: {effect['clear_client'].srv_name}")
                self._start_costmap_clear(effect)
            elif now > effect["deadline"]:
                self.get_logger().warning(
                    f"costmap clear timed out, skipping: {effect['clear_client'].srv_name}"
                )
                self._start_costmap_clear(effect)
            return

    def _start_costmap_clear(self, effect):
        """Pops the next clear service into the effect chain; finalizes when done.

        Clear failures are warnings, not mission blockers: the map itself is
        already loaded, and a stale costmap recovers as fresh sensor data
        arrives. Map load failures, by contrast, are fatal (handled above).
        """
        remaining = effect.get("remaining_clears")
        if remaining is None:
            remaining = [str(s) for s in self.get_parameter("costmap_clear_services").value]
        if not remaining:
            self._effect = None
            self._finalize()
            return
        service = remaining[0]
        if service not in self._clear_clients:
            self._clear_clients[service] = self.create_client(ClearEntireCostmap, service)
        wait_timeout = float(self.get_parameter("service_wait_timeout_sec").value)
        effect.update(
            step="clear_wait_service",
            clear_client=self._clear_clients[service],
            remaining_clears=remaining[1:],
            deadline=time.monotonic() + wait_timeout,
            future=None,
        )

    # ------------------------------------------------------------------ finalize

    def _finalize(self):
        if bool(self.get_parameter("publish_initialpose").value):
            self._publish_initialpose()
        self.core.on_finalize_done()
        self.get_logger().info(
            f"floor switch READY: current_floor={self.core.current_floor}, "
            f"map={self._map_yaml}, initialpose_sent={self._initialpose_sent}"
        )
        self._publish_status()

    def _publish_initialpose(self):
        floor = self.core.target_floor
        spawn = self.core.spawn_point_id
        try:
            point = self.registry.get_point(floor, spawn)
        except KeyError:
            fallback = self.registry.default_spawn_point_id
            self.get_logger().warning(
                f"spawn point '{spawn}' not in floor_maps.yaml for {floor}, "
                f"falling back to '{fallback}'"
            )
            try:
                point = self.registry.get_point(floor, fallback)
            except KeyError as exc:
                self.get_logger().error(f"initialpose skipped: {exc}")
                return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.registry.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = point.x
        msg.pose.pose.position.y = point.y
        yaw = point.yaw_rad
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.06853891909122467
        msg.pose.covariance = covariance
        self.initialpose_pub.publish(msg)
        self._initialpose_sent = True
        self.get_logger().info(
            f"initialpose published: {floor}:{point.point_id} ({point.x}, {point.y})"
        )

    # ------------------------------------------------------------------ status

    def _fail(self, error):
        self.core.on_map_load_failure(error)
        self.get_logger().error(f"floor switch FAILED: {error}")
        self._publish_status()

    def _publish_status(self):
        payload = self.core.status()
        payload["map_yaml"] = self._map_yaml
        payload["initialpose_sent"] = self._initialpose_sent
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = AutoFloorOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl-C already shuts the context down in Humble; a second
        # rclpy.shutdown() would raise "rcl_shutdown already called".
        rclpy.try_shutdown()
