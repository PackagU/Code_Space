#!/usr/bin/env python3
"""L2 ROS smoke test: in-process, dry-run, no Gazebo/Nav2 needed.

Replays the exact call order legacy SwitchFloor uses:
  1. /floor_orchestrator_node/set_parameters  (target_floor, spawn_point_id)
  2. /floor_orchestrator/request_switch       (std_srvs/Trigger)
  3. waits for /floor_orchestrator/status with current_floor=target, pending=false

and additionally asserts the new auto contract: map_loaded=true, phase=ready,
/initialpose published at the spawn point AFTER the (dry-run) map load.

Run (isolated domain recommended):
  ROS_DOMAIN_ID=89 python3 test_workspace/elevator_auto_map_switch/scripts/test_orchestrator_ros_smoke.py
"""
import json
import time
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "auto_floor_orchestrator_pkg"))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from auto_floor_orchestrator_pkg.auto_floor_orchestrator_node import AutoFloorOrchestratorNode


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    rclpy.init()
    orchestrator = AutoFloorOrchestratorNode(
        parameter_overrides=[
            Parameter("status_period_sec", value=0.1),
            Parameter("dry_run_map_load", value=True),
        ]
    )
    harness = Node("smoke_harness")
    executor = SingleThreadedExecutor()
    executor.add_node(orchestrator)
    executor.add_node(harness)

    status_box = {}
    pose_box = []
    harness.create_subscription(
        String, "/floor_orchestrator/status",
        lambda m: status_box.update(json.loads(m.data)), 10)
    harness.create_subscription(
        PoseWithCovarianceStamped, "/initialpose",
        lambda m: pose_box.append(m), 10)
    elevator_pub = harness.create_publisher(String, "/elevator/state", 10)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

    def spin_until(predicate, seconds, what):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError(f"timeout waiting for {what} (last status: {status_box})")

    def publish_elevator(payload):
        msg = String()
        msg.data = json.dumps(payload)
        elevator_pub.publish(msg)

    try:
        # --- 1. legacy contract: set target via node parameters service ---
        set_params = harness.create_client(
            SetParameters, "/floor_orchestrator_node/set_parameters")
        require(set_params.wait_for_service(timeout_sec=5.0),
                "node must be named floor_orchestrator_node (legacy SwitchFloor hardcodes it)")
        req = SetParameters.Request()
        req.parameters = [
            Parameter("target_floor", value="F2").to_parameter_msg(),
            Parameter("spawn_point_id", value="elevator_inside").to_parameter_msg(),
        ]
        future = set_params.call_async(req)
        spin_until(future.done, 5.0, "set_parameters response")
        require(all(r.successful for r in future.result().results),
                "set_parameters should succeed")

        # --- 2. legacy contract: Trigger request_switch ---
        request = harness.create_client(Trigger, "/floor_orchestrator/request_switch")
        require(request.wait_for_service(timeout_sec=5.0), "request_switch service missing")
        future = request.call_async(Trigger.Request())
        spin_until(future.done, 5.0, "request_switch response")
        require(future.result().success, f"request rejected: {future.result().message}")

        # --- 3. elevator still moving: must stay pending ---
        for _ in range(3):
            publish_elevator({"current_floor": "F1", "target_floor": "F2",
                              "door_state": "closed", "state": "MOVING"})
            spin_for(0.15)
        require(status_box.get("pending") is True, f"should stay pending: {status_box}")
        require(status_box.get("phase") == "waiting_elevator", f"unexpected: {status_box}")
        require(not pose_box, "initialpose must NOT be published before map load")

        # --- 4. arrival with door open: auto switch, no human ack ---
        publish_elevator({"current_floor": "F2", "target_floor": "F2",
                          "door_state": "open", "state": "ARRIVED_OPEN"})
        spin_until(lambda: status_box.get("phase") == "ready", 5.0, "phase=ready")

        require(status_box.get("current_floor") == "F2", f"bad floor: {status_box}")
        require(status_box.get("pending") is False, f"pending should clear: {status_box}")
        require(status_box.get("map_loaded") is True, f"map_loaded missing: {status_box}")
        require(str(status_box.get("map_yaml", "")).endswith("kku_f2.yaml"),
                f"status should expose the resolved map yaml: {status_box}")
        require(status_box.get("initialpose_sent") is True, f"pose flag missing: {status_box}")

        # legacy SwitchFloor gate, verbatim semantics
        require(status_box.get("current_floor") == "F2"
                and not status_box.get("pending", True),
                "legacy SwitchFloor gate should open without any manual ack")

        # --- 5. initialpose published at spawn point (robot is INSIDE the elevator) ---
        spin_until(lambda: pose_box, 2.0, "initialpose message")
        pose = pose_box[-1]
        require(pose.header.frame_id == "map", "initialpose frame must be map")
        require(abs(pose.pose.pose.position.x) < 1e-6
                and abs(pose.pose.pose.position.y) < 1e-6,
                "initialpose must be elevator_inside (0,0), not elevator_exit")

        # --- 6. ack stub stays harmless for operators with old habits ---
        ack = harness.create_client(Trigger, "/floor_orchestrator/ack")
        require(ack.wait_for_service(timeout_sec=5.0), "ack stub service missing")
        future = ack.call_async(Trigger.Request())
        spin_until(future.done, 5.0, "ack response")
        require("auto" in future.result().message.lower(),
                "ack stub should explain that auto mode needs no ack")

        print("PASS orchestrator ros smoke")
    finally:
        executor.shutdown()
        orchestrator.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
