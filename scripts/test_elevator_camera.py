#!/usr/bin/env python3
"""Offline evidence tests: wrong floors, replay, loss, doors and real arm responses."""
import json
from pathlib import Path
import sys
import unittest
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/robot_arm_pkg"))
sys.path.insert(0, str(ROOT / "test_workspace/elevator_mission/src/elevator_mission_pkg"))
from robot_arm_pkg.arm_execution import ArmExecutionContract
from robot_arm_pkg.camera_views import load_views, validate_views
from robot_arm_pkg import servo_protocol as sp
from elevator_mission_pkg.elevator_camera import ElevatorArrivalGate, reader_observation


def door(t, **changes):
    return dict({"source": "sensor", "stamp": 1000 + t, "door_state": "open",
                 "stopped": True, "exit_clear": True}, **changes)


def frame(t, seq, floor="F3", stream="camera-a", **changes):
    return dict({"valid": True, "floor": floor, "captured_at": 1000 + t,
                 "stream_id": stream, "frame_seq": seq}, **changes)


class FakeArm:
    def __init__(self):
        self.positions = dict(sp.HOME)
        self.sent, self.stopped = [], False

    def send_pose(self, name, duration, poses):
        self.sent.append(sp.pose_command(name, duration, poses))

    def read_positions(self):
        return self.positions

    def stop_all(self):
        self.stopped = True


class Tests(unittest.TestCase):
    def test_field_map_guard_rejection_prevents_nav2_goal(self):
        _, behaviors = self.ros_behaviors()
        node = types.SimpleNamespace(check_navigation_goal=lambda point: False)
        nav = behaviors.NavigateRoute(node, [object()], 'field')
        nav._client = types.SimpleNamespace(wait_for_server=lambda **kw: True,
            send_goal_async=lambda goal: self.fail('goal sent despite failed map identity'))
        self.assertEqual(nav.update(), behaviors.FAILURE)

    def ros_behaviors(self):
        # Minimal message/action interfaces; the actual behavior implementations run below.
        for name in ("geometry_msgs.msg", "rclpy.parameter", "std_msgs.msg", "std_srvs.srv",
                     "nav2_msgs.action", "rclpy.action"):
            parent = name.split(".")[0]
            if parent not in sys.modules:
                sys.modules[parent] = types.ModuleType(parent)
            if name not in sys.modules:
                sys.modules[name] = types.ModuleType(name)
        class Message:
            data = ""
        sys.modules["std_msgs.msg"].String = Message
        sys.modules["geometry_msgs.msg"].PoseWithCovarianceStamped = object
        sys.modules["rclpy.parameter"].Parameter = object
        sys.modules["std_srvs.srv"].Trigger = object
        sys.modules["nav2_msgs.action"].NavigateToPose = object
        sys.modules["rclpy.action"].ActionClient = object
        from elevator_mission_pkg import camera_behaviors, behaviors
        return camera_behaviors, behaviors

    def test_nav2_abort_is_failure_and_pending_goal_can_be_cancelled(self):
        _, behaviors = self.ros_behaviors()
        node = types.SimpleNamespace(get_logger=lambda: types.SimpleNamespace(info=lambda x: None))
        nav = behaviors.NavigateRoute(node, [object()], "test")
        nav._sent = True
        nav._result_future = types.SimpleNamespace(done=lambda: True,
                                                 result=lambda: types.SimpleNamespace(status=6))
        self.assertEqual(nav.update(), behaviors.FAILURE)
        callbacks, cancelled = [], []
        nav._goal_future = types.SimpleNamespace(add_done_callback=callbacks.append)
        nav.cancel()
        callbacks[0](types.SimpleNamespace(result=lambda: types.SimpleNamespace(
            accepted=True, cancel_goal_async=lambda: cancelled.append(True))))
        self.assertEqual(cancelled, [True])

    def test_cancel_between_waypoints_targets_the_new_pending_goal(self):
        _, behaviors = self.ros_behaviors()
        node = types.SimpleNamespace(get_logger=lambda: types.SimpleNamespace(info=lambda x: None))
        nav = behaviors.NavigateRoute(node, [object(), object()], "test")
        old, new, callbacks = [], [], []
        nav._sent = True
        nav._goal_handle = types.SimpleNamespace(cancel_goal_async=lambda: old.append(True))
        nav._result_future = types.SimpleNamespace(done=lambda: True,
                                                 result=lambda: types.SimpleNamespace(status=4))
        self.assertEqual(nav.update(), behaviors.RUNNING)
        self.assertIsNone(nav._goal_handle)
        nav._goal_future = types.SimpleNamespace(add_done_callback=callbacks.append)
        nav.cancel()
        callbacks[0](types.SimpleNamespace(result=lambda: types.SimpleNamespace(
            accepted=True, cancel_goal_async=lambda: new.append(True))))
        self.assertEqual(old, [])
        self.assertEqual(new, [True])

    def test_complete_mission_exit_waits_for_floor_map_and_front_view(self):
        cb, behaviors = self.ros_behaviors()
        class Node:
            def __init__(self):
                self.arrival_gate = ElevatorArrivalGate("F3")
                self.callbacks, self.commands, self.inhibits = [], [], []
            def create_publisher(self, *args):
                return types.SimpleNamespace(publish=lambda m: self.commands.append(json.loads(m.data)))
            def create_subscription(self, _kind, _topic, callback, _depth):
                self.callbacks.append(callback)
            def count_subscribers(self, topic):
                return 1
            def set_drive_inhibit(self, reason):
                self.inhibits.append(reason)
            def get_logger(self):
                return types.SimpleNamespace(error=lambda m: None)
            def complete(self):
                command = self.commands[-1]
                status = dict(command, event="completed", completion_basis="controller_position_response",
                              simulation_mode=False, current_view=command.get("view", ""))
                for callback in self.callbacks:
                    callback(types.SimpleNamespace(data=json.dumps(status)))
        class Nav:
            name = "nav-exit"
            calls = 0
            def tick(self):
                self.calls += 1
                return behaviors.SUCCESS
            def cancel(self):
                pass
        map_done = [False]
        class Map:
            def __init__(self, *args):
                pass
            def tick(self):
                return behaviors.SUCCESS if map_done[0] else behaviors.RUNNING
        node, nav, clock = Node(), Nav(), [0.]
        with patch.object(cb, "SwitchFloor", Map), patch.object(cb.time, "monotonic", lambda: clock[0]), \
                patch.object(cb.time, "time", lambda: 1000+clock[0]):
            exit_step = cb.PrepareCameraExit(node, "F3", "inside", nav)
            self.assertEqual(exit_step.tick(), behaviors.RUNNING)
            self.assertEqual(node.commands[-1]["view"], "floor_view")
            node.complete()
            clock[0] = .1
            exit_step.tick()
            for i in range(5):
                clock[0] = .7+i*.1
                node.arrival_gate.observe_floor(frame(clock[0], i+1, floor="F2"),
                                               clock[0], 1000+clock[0])
                exit_step.tick()
            self.assertEqual(exit_step.phase, "wait")
            self.assertEqual(nav.calls, 0)
            for i in range(5):
                clock[0] = 1.2+i*.1
                node.arrival_gate.observe_floor(frame(clock[0], i+6), clock[0], 1000+clock[0])
                exit_step.tick()
            self.assertEqual(exit_step.phase, "map")
            exit_step.tick()
            self.assertEqual(nav.calls, 0)
            map_done[0] = True
            exit_step.tick()
            self.assertEqual(exit_step.phase, "front")
            exit_step.tick()
            self.assertEqual(node.commands[-1]["view"], "front_view")
            self.assertEqual(nav.calls, 0)
            node.complete()
            exit_step.tick()
            self.assertEqual(exit_step.phase, "exit")
            self.assertEqual(exit_step.tick(), behaviors.SUCCESS)
            self.assertEqual(nav.calls, 1)

    def test_door_closes_during_exit_cancels_and_inhibits_navigation(self):
        cb, behaviors = self.ros_behaviors()
        gate = self.gate()
        self.arrive(gate)
        gate.freeze_for_front_view(1.4, 1001.4)
        cancelled, inhibits = [], []
        nav = types.SimpleNamespace(name="exit", tick=lambda: behaviors.RUNNING,
                                    cancel=lambda: cancelled.append(True))
        node = types.SimpleNamespace(arrival_gate=gate, set_drive_inhibit=inhibits.append)
        guarded = cb.GuardedNavigate(node, nav, exit_car=True)
        clock = [1.4]
        with patch.object(cb.time, "monotonic", lambda: clock[0]), \
                patch.object(cb.time, "time", lambda: 1000+clock[0]):
            self.assertEqual(guarded.tick(), behaviors.RUNNING)
            gate.observe_door(door(1.5, door_state="closed"), 1.5, 1001.5)
            clock[0] = 1.5
            self.assertEqual(guarded.tick(), behaviors.FAILURE)
        self.assertEqual(cancelled, [True])
        self.assertEqual(inhibits[-1], "elevator_transfer_blocked")

    def gate(self):
        gate = ElevatorArrivalGate("F3", require_door_confirmation=True)
        gate.begin_looking(1000)
        return gate

    def test_default_camera_only_mode_needs_no_door_input(self):
        gate = ElevatorArrivalGate("F3")
        gate.begin_looking(1000)
        for i in range(10):
            gate.observe_floor(frame(1+i*.1, i+1, floor="F2"), 1+i*.1, 1001+i*.1)
        self.assertFalse(gate.confirmed(1.9, 1001.9))
        for i in range(5):
            gate.observe_floor(frame(2+i*.1, i+11), 2+i*.1, 1002+i*.1)
        self.assertTrue(gate.freeze_for_front_view(2.4, 1002.4))
        self.assertTrue(gate.can_exit(4.4, 1004.4))
        self.assertFalse(gate.can_exit(33, 1033))

    def observe(self, gate, t, seq, **changes):
        gate.observe_door(door(t), t, 1000+t)
        gate.observe_floor(frame(t, seq, **changes), t, 1000+t)

    def arrive(self, gate):
        for i in range(5):
            self.observe(gate, 1+i*.1, i+1)
        self.assertTrue(gate.confirmed(1.4, 1001.4))

    def test_user_camera_poses_require_feedback(self):
        views, duration = load_views(ROOT / "src/robot_arm_pkg/config/camera_views.json")
        self.assertEqual(views["front_view"], {"000":1500,"001":1500,"002":2000,"003":1500})
        self.assertEqual(views["floor_view"], {"000":1500,"001":1480,"002":1670,"003":1500})
        arm = FakeArm()
        contract = ArmExecutionContract(arm, camera_views=views, view_duration_ms=duration)
        contract.submit({"request_id":"home", "action":"home"}, 0)
        contract.tick(sp.HOMING_DURATION_MS)
        status = contract.submit({"request_id":"front", "action":"view", "view":"front_view"}, 3000)
        self.assertEqual(status["event"], "started")
        self.assertIn("#002P2000", arm.sent[-1])
        self.assertIsNone(contract.tick(5000))  # elapsed time with old home response is insufficient
        arm.positions = dict(views["front_view"])
        self.assertEqual(contract.tick(5001)["current_view"], "front_view")
        contract.submit({"request_id":"floor", "action":"view", "view":"floor_view"}, 6000)
        arm.positions = dict(views["floor_view"])
        self.assertEqual(contract.tick(8000)["current_view"], "floor_view")
        bad = dict(views, duration_ms=duration)
        bad["floor_view"] = dict(views["floor_view"], **{"002":9000})
        with self.assertRaises(ValueError):
            validate_views(bad)

    def test_wrong_floor_and_pre_pose_frames_never_exit(self):
        gate = self.gate()
        for i in range(20):
            self.observe(gate, i*.1, i+1, floor="F2")
        self.assertFalse(gate.confirmed(1.9, 1001.9))
        for i in range(5):
            self.observe(gate, 2+i*.1, i+21)
        self.assertTrue(gate.freeze_for_front_view(2.4, 1002.4))
        self.assertTrue(gate.can_exit(2.4, 1002.4))
        before = self.gate()
        for i in range(5):
            self.observe(before, i*.05, i+1)
        self.assertEqual(before.count, 0)

    def test_replayed_frame_does_not_count_and_freshness_expires(self):
        gate = self.gate()
        for i in range(10):
            gate.observe_door(door(1+i*.05), 1+i*.05, 1001+i*.05)
            gate.observe_floor(frame(1, 1), 1+i*.05, 1001+i*.05)
        self.assertEqual(gate.count, 1)
        self.assertFalse(gate.confirmed(1.45, 1001.45))
        self.assertFalse(gate.confirmed(3, 1003))

    def test_reader_restart_requires_new_consecutive_frames(self):
        gate = self.gate()
        for i in range(4):
            self.observe(gate, 1+i*.1, i+1)
        self.observe(gate, 1.4, 1, stream="camera-b")
        self.assertEqual(gate.count, 1)
        self.assertFalse(gate.confirmed(1.4, 1001.4))

    def test_close_reopen_and_missing_door_revoke_latched_arrival(self):
        gate = self.gate()
        self.arrive(gate)
        self.assertTrue(gate.freeze_for_front_view(1.4, 1001.4))
        gate.observe_door(door(1.5, door_state="closing"), 1.5, 1001.5)
        self.assertFalse(gate.can_exit(1.5, 1001.5))
        gate.observe_door(door(1.6), 1.6, 1001.6)
        self.assertFalse(gate.can_exit(1.6, 1001.6))
        missing = self.gate()
        self.arrive(missing)
        missing.freeze_for_front_view(1.4, 1001.4)
        self.assertFalse(missing.can_exit(3, 1003))
        missing.observe_door(door(3.1), 3.1, 1003.1)
        self.assertFalse(missing.can_exit(3.1, 1003.1))

    def test_moving_obstructed_simulated_invalid_and_future_door(self):
        for change in ({"stopped":False}, {"exit_clear":False}, {"source":"simulator"},
                       {"stamp":float("nan")}, {"stamp":1009}, {"stopped":"true"}):
            gate = self.gate()
            self.arrive(gate)
            gate.observe_door(door(1.5, **change), 1.5, 1001.5)
            self.assertFalse(gate.freeze_for_front_view(1.5, 1001.5))

    def test_unknown_or_lost_camera_revokes_before_front_view(self):
        gate = self.gate()
        self.arrive(gate)
        gate.observe_floor({"valid":False}, 1.5, 1001.5)
        self.assertFalse(gate.freeze_for_front_view(1.5, 1001.5))

    def test_http_bridge_ignores_historical_event_and_stale_snapshot(self):
        state = {"floor":"3", "score":.9, "margin":.2, "error":None, "roi":[[0,0]],
                 "updated":1001, "frame_seq":1, "stream_id":"a",
                 "event":{"floor":"3", "timestamp":1001}}
        self.assertTrue(reader_observation(state, 1001.2)["valid"])
        self.assertFalse(reader_observation(state, 1003)["valid"])
        state["floor"] = "UNKNOWN"
        self.assertFalse(reader_observation(state, 1001.2)["valid"])
        self.assertFalse(reader_observation(None, 1001)["valid"])


if __name__ == "__main__":
    unittest.main()
