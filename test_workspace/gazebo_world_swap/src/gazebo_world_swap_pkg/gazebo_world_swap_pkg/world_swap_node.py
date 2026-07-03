"""ROS node that swaps Gazebo floor geometry after Nav2 map switch readiness."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import subprocess
import time

from gazebo_world_swap_pkg.swap_trigger import WorldSwapTrigger
from gazebo_world_swap_pkg.world_model import (
    building_model_name,
    extract_building_spawn_xml,
    extract_world_model_names,
    normalize_floor,
    world_path_for_floor,
)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from ament_index_python.packages import get_package_share_directory
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from geometry_msgs.msg import Pose
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:  # Allows host-side tests of pure helper functions.
    rclpy = None
    Node = object
    ROS_AVAILABLE = False


DEFAULT_RESTART_COMMAND_TEMPLATE = (
    "ros2 launch common_pkg gazebo.launch.py "
    "floor:={floor} spawn_point:=elevator_inside use_sim_time:=true"
)


def format_restart_command(template, floor):
    return shlex.split(str(template).format(floor=normalize_floor(floor)))


def status_json(state, method, source_floor="", target_floor="", detail=""):
    return json.dumps(
        {
            "detail": str(detail),
            "method": str(method),
            "source_floor": str(source_floor),
            "state": str(state),
            "target_floor": str(target_floor),
        },
        sort_keys=True,
    )


if ROS_AVAILABLE:

    class GazeboWorldSwapNode(Node):
        def __init__(self):
            super().__init__("gazebo_world_swap_node")
            self.declare_parameter("method", "model_swap")
            self.declare_parameter("initial_floor", "F1")
            self.declare_parameter("world_dir", "")
            self.declare_parameter("status_topic", "/floor_orchestrator/status")
            self.declare_parameter("delete_entity_service", "/delete_entity")
            self.declare_parameter("spawn_entity_service", "/spawn_entity")
            self.declare_parameter("service_wait_timeout_sec", 10.0)
            self.declare_parameter(
                "restart_command_template", DEFAULT_RESTART_COMMAND_TEMPLATE
            )

            self.method = str(self.get_parameter("method").value)
            self.world_dir = self._resolve_world_dir()
            self.trigger = WorldSwapTrigger(
                initial_floor=self.get_parameter("initial_floor").value
            )
            self._effect = None
            self._restart_process = None

            status_qos = QoSProfile(depth=10)
            status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.status_pub = self.create_publisher(
                String, "/gazebo_world_swap/status", status_qos
            )
            self.create_subscription(
                String,
                self.get_parameter("status_topic").value,
                self._on_orchestrator_status,
                10,
            )
            self.delete_client = self.create_client(
                DeleteEntity, self.get_parameter("delete_entity_service").value
            )
            self.spawn_client = self.create_client(
                SpawnEntity, self.get_parameter("spawn_entity_service").value
            )
            self.create_timer(0.2, self._advance_effect)
            self.get_logger().info(
                f"world swap ready: method={self.method}, "
                f"initial_floor={self.trigger.last_floor}, world_dir={self.world_dir}"
            )

        def _resolve_world_dir(self) -> Path:
            configured = str(self.get_parameter("world_dir").value)
            if configured:
                return Path(configured)
            return Path(get_package_share_directory("common_pkg")) / "worlds"

        def _on_orchestrator_status(self, msg):
            request = self.trigger.observe(msg.data)
            if request is None:
                return
            if self._effect is not None:
                self._publish(
                    "busy",
                    request,
                    f"swap already in progress for {self._effect['request'].target_floor}",
                )
                return
            if self.method == "model_swap":
                self._start_model_swap(request)
            elif self.method == "restart":
                self._restart_gazebo(request)
            else:
                self._publish("failed", request, f"unknown method: {self.method}")

        def _start_model_swap(self, request):
            try:
                source_model = building_model_name(request.source_floor)
                source_world = world_path_for_floor(
                    request.source_floor, self.world_dir
                )
                target_world = world_path_for_floor(request.target_floor, self.world_dir)
                # 건물뿐 아니라 층 전용 소품(parcel_box, obstacle_* 등)도 함께
                # 지우고/생성한다 — 건물만 갈아끼우면 소품이 다른 층에 잔류한다.
                delete_names = extract_world_model_names(source_world)
                if source_model not in delete_names:
                    delete_names.insert(0, source_model)
                spawn_items = [
                    (name, extract_building_spawn_xml(target_world, name))
                    for name in extract_world_model_names(target_world)
                ]
            except Exception as exc:
                self._publish("failed", request, exc)
                return

            timeout = float(self.get_parameter("service_wait_timeout_sec").value)
            self._effect = {
                "step": "delete_wait_service",
                "request": request,
                "source_model": source_model,
                "delete_names": delete_names,
                "current_delete": "",
                "spawn_items": spawn_items,
                "current_spawn": "",
                "deadline": time.monotonic() + timeout,
            }
            self._publish(
                "deleting_source",
                request,
                f"delete {', '.join(delete_names)}, then spawn {request.target_model}",
            )

        def _advance_effect(self):
            effect = self._effect
            if effect is None:
                return
            now = time.monotonic()
            step = effect["step"]
            request = effect["request"]

            if step == "delete_wait_service":
                if not effect["delete_names"]:
                    self._start_spawn(effect)
                    return
                if self.delete_client.service_is_ready():
                    req = DeleteEntity.Request()
                    req.name = effect["delete_names"].pop(0)
                    effect["current_delete"] = req.name
                    effect.update(
                        step="delete_wait_response",
                        future=self.delete_client.call_async(req),
                    )
                elif now > effect["deadline"]:
                    self._fail_effect("delete_entity service unavailable")
                return

            if step == "delete_wait_response":
                future = effect["future"]
                if future.done():
                    result = future.result()
                    if result is None or not result.success:
                        detail = "delete_entity failed"
                        if result is not None:
                            detail = f"{detail}: {result.status_message}"
                        # 건물 삭제 실패는 치명(전환 불가). 소품은 이미 없을 수
                        # 있으므로(수동 삭제 등) 경고만 남기고 계속 진행.
                        if effect["current_delete"] == effect["source_model"]:
                            self._fail_effect(detail)
                            return
                        self.get_logger().warning(
                            f"prop {effect['current_delete']}: {detail}"
                        )
                    if effect["delete_names"]:
                        timeout = float(
                            self.get_parameter("service_wait_timeout_sec").value
                        )
                        effect.update(
                            step="delete_wait_service",
                            deadline=time.monotonic() + timeout,
                        )
                    else:
                        self._start_spawn(effect)
                return

            if step == "spawn_wait_service":
                if not effect["spawn_items"]:
                    self._finish_swap(effect)
                    return
                if self.spawn_client.service_is_ready():
                    name, xml = effect["spawn_items"].pop(0)
                    effect["current_spawn"] = name
                    req = SpawnEntity.Request()
                    req.name = name
                    req.xml = xml
                    req.robot_namespace = ""
                    req.initial_pose = Pose()
                    req.reference_frame = "world"
                    effect.update(
                        step="spawn_wait_response",
                        future=self.spawn_client.call_async(req),
                    )
                elif now > effect["deadline"]:
                    self._fail_effect("spawn_entity service unavailable")
                return

            if step == "spawn_wait_response":
                future = effect["future"]
                if future.done():
                    result = future.result()
                    if result is None or not result.success:
                        detail = "spawn_entity failed"
                        if result is not None:
                            detail = f"{detail}: {result.status_message}"
                        # 건물 스폰 실패는 치명. 소품은 경고 후 계속.
                        if effect["current_spawn"] == request.target_model:
                            self._fail_effect(detail)
                            return
                        self.get_logger().warning(
                            f"prop {effect['current_spawn']}: {detail}"
                        )
                    if effect["spawn_items"]:
                        timeout = float(
                            self.get_parameter("service_wait_timeout_sec").value
                        )
                        effect.update(
                            step="spawn_wait_service",
                            deadline=time.monotonic() + timeout,
                        )
                    else:
                        self._finish_swap(effect)

        def _start_spawn(self, effect):
            timeout = float(self.get_parameter("service_wait_timeout_sec").value)
            effect.update(
                step="spawn_wait_service",
                deadline=time.monotonic() + timeout,
                future=None,
            )
            names = ", ".join(name for name, _ in effect["spawn_items"])
            self._publish(
                "spawning_target",
                effect["request"],
                f"spawn {names or effect['request'].target_model}",
            )

        def _finish_swap(self, effect):
            request = effect["request"]
            self._effect = None
            self._publish(
                "swapped",
                request,
                f"spawned {request.target_model}",
            )

        def _fail_effect(self, detail):
            request = self._effect["request"]
            self._effect = None
            self._publish("failed", request, detail)

        def _restart_gazebo(self, request):
            self._publish("restarting", request, "terminating Gazebo processes")
            for pattern in ("gzserver", "gzclient"):
                subprocess.run(["pkill", "-f", pattern], check=False)
            cmd = format_restart_command(
                self.get_parameter("restart_command_template").value,
                request.target_floor,
            )
            self._restart_process = subprocess.Popen(
                cmd,
                cwd=os.getcwd(),
                start_new_session=True,
            )
            self._publish("restarted", request, " ".join(cmd))

        def _publish(self, state, request, detail):
            msg = String()
            msg.data = status_json(
                state=state,
                method=self.method,
                source_floor=request.source_floor,
                target_floor=request.target_floor,
                detail=detail,
            )
            self.status_pub.publish(msg)
            if state == "failed":
                self.get_logger().error(msg.data)
            else:
                self.get_logger().info(msg.data)


else:

    class GazeboWorldSwapNode:
        def __init__(self):
            raise RuntimeError("ROS dependencies are required to run GazeboWorldSwapNode")


def main():
    if not ROS_AVAILABLE:
        raise RuntimeError("ROS dependencies are required to run world_swap_node")
    rclpy.init()
    node = GazeboWorldSwapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
