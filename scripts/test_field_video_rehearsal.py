#!/usr/bin/env python3
"""촬영 리허설 진행기 오프라인 시험 (ROS·장치 없이 가짜 연결로)."""

import json
import queue
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from field_video import backends, runner  # noqa: E402
from field_video import ros_straight_move  # noqa: E402

SCENARIO = HERE / "field_video" / "scenario.json"
REGISTRY = HERE.parent / "src" / "slam_pkg" / "maps" / "field" / "waypoints.json"


def make(answers=None, failures=None, ultrasonic=None, nav_restart=False, auto=True, **field_kwargs):
    scenario = runner.load_scenario(SCENARIO)
    tmp = tempfile.mkdtemp(prefix="fv_test_")
    out = []
    field = backends.FakeField(failures, **field_kwargs)
    web = backends.FakeWeb(failures)
    lift = backends.FakeLift(failures)
    ui = runner.ConsoleUI(answers=answers, auto=auto, out=out.append)
    log = runner.RunLog(tmp, "test")
    us = ultrasonic if ultrasonic is not None else backends.FakeUltrasonic()
    rehearsal = runner.Rehearsal(scenario, field, web, lift, ui, log, nav_restart=nav_restart,
                                 allow_preview_arm=True, sleep=lambda _s: None, ultrasonic=us)
    return rehearsal, scenario, field, web, lift, out


def run(rehearsal, scenario, **select):
    steps = scenario["steps"]
    return rehearsal.run(runner.select_steps(steps, **select), steps)


def statuses(rehearsal):
    return {r["id"]: r["status"] for r in rehearsal.results}


class FullRunTest(unittest.TestCase):
    def test_full_dry_run_is_one_take(self):
        rehearsal, scenario, field, web, lift, _ = make()
        meta = run(rehearsal, scenario)
        self.assertTrue(meta["verdict"].startswith("한 번에"), meta["verdict"])
        self.assertEqual(len(rehearsal.results), len(scenario["steps"]))
        self.assertEqual(set(statuses(rehearsal).values()), {"OK"})
        presses = [c for c in web.calls if c[0] == "mission"]
        self.assertEqual([c[1:] for c in presses],
                         [("hall", "up"), ("car", "up"), ("hall", "down"), ("car", "down")])
        self.assertEqual([c for c in lift.calls if c[0] == "move"], [("move", "up"), ("move", "down")])
        self.assertIn(("load_map", "/ros2_ws/maps/field/f2/pinned.yaml"), field.calls)
        self.assertIn(("load_map", "/ros2_ws/maps/field/f1/pinned.yaml"), field.calls)
        self.assertIn(("record_start", "rehearsal"), field.calls)
        self.assertEqual(field.calls[-1], ("record_stop",))
        summary = (rehearsal.log.dir / "summary.md").read_text(encoding="utf-8")
        self.assertIn("S26", summary)

    def test_motion_is_always_wrapped_by_cancel_resume_stop(self):
        rehearsal, scenario, field, *_ = make()
        run(rehearsal, scenario)
        calls = field.calls
        for index, call in enumerate(calls):
            if call[0] in ("goal", "teleop", "straight", "straight_until"):
                self.assertEqual(calls[index - 1], ("resume",), call)
                self.assertEqual(calls[index - 2], ("cancel",), call)
                self.assertEqual(calls[index + 1], ("stop",), call)
            if call[0] == "teleop":
                self.assertEqual(call[1], "released")

    def test_goal_never_sent_on_wrong_floor_map(self):
        rehearsal, scenario, field, *_ = make()
        run(rehearsal, scenario)
        loaded = "F1"
        for call in field.calls:
            if call[0] == "load_map":
                loaded = call[1].split("/")[-2].upper()
            if call[0] == "goal":
                self.assertEqual(call[2], loaded)


class FailurePathTest(unittest.TestCase):
    def test_goal_failure_is_retried_automatically_once(self):
        rehearsal, scenario, field, *_ = make(failures={"goal:f1_locker": 1})
        meta = run(rehearsal, scenario)
        self.assertEqual(meta["retries"], 0)
        locker = next(r for r in rehearsal.results if r["id"] == "S02")
        self.assertIn("자동 재시도 1회", locker["detail"])
        self.assertIn(("clear",), field.calls)

    def test_goal_failing_twice_shows_menu_and_retry_works(self):
        rehearsal, scenario, field, *_ = make(failures={"goal:f1_locker": 2}, answers=["", "", "r"])
        meta = run(rehearsal, scenario, only=["S02"])
        result = rehearsal.results[0]
        self.assertEqual((result["status"], result["attempts"]), ("OK", 2))
        self.assertEqual(meta["retries"], 1)
        self.assertFalse(meta["verdict"].startswith("한 번에"))

    def test_lamp_not_lit_retries_press(self):
        rehearsal, scenario, _, web, *_ = make(answers=["", "r", "r", ""])
        run(rehearsal, scenario, only=["S10"])
        self.assertEqual(rehearsal.results[0]["attempts"], 2)
        self.assertEqual(len([c for c in web.calls if c[0] == "press"]), 2)

    def test_arm_error_then_person_does_it_is_manual(self):
        rehearsal, scenario, _, web, *_ = make(failures={"press:4": 1}, answers=["", "s"])
        meta = run(rehearsal, scenario, only=["S10"])
        self.assertEqual(rehearsal.results[0]["status"], "MANUAL")
        self.assertEqual(meta["manual"], 1)
        self.assertIn(("stop",), web.calls)

    def test_button_not_recognized_times_out(self):
        rehearsal, scenario, *_ = make(answers=["", "s"])
        rehearsal.settings = dict(rehearsal.settings, button_ready_timeout_sec=0.05)
        rehearsal.web.fail.failures["ready:UP"] = 10 ** 6
        run(rehearsal, scenario, only=["S07"])
        self.assertEqual(rehearsal.results[0]["status"], "MANUAL")
        self.assertIn("not stably recognized", rehearsal.results[0]["detail"])

    def test_quit_stops_robot_and_still_writes_summary(self):
        rehearsal, scenario, field, *_ = make(answers=["", "q"], auto=False)
        meta = run(rehearsal, scenario, only=["S02"])
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("goal", [c[0] for c in field.calls])
        self.assertEqual(field.calls[-1], ("record_stop",))
        self.assertTrue((rehearsal.log.dir / "steps.csv").exists())

    def test_optional_teleop_can_be_skipped(self):
        rehearsal, scenario, field, *_ = make(answers=["", "n"])
        run(rehearsal, scenario, only=["S15"])
        self.assertEqual(rehearsal.results[0]["status"], "SKIPPED")
        self.assertNotIn("teleop", [c[0] for c in field.calls])

    def test_adjust_then_start_counts_adjust(self):
        rehearsal, scenario, field, *_ = make(answers=["t", ""])
        run(rehearsal, scenario, only=["S07"])
        self.assertEqual(rehearsal.results[0]["adjusts"], 1)
        self.assertIn(("teleop", "released"), field.calls)


class UltrasonicTest(unittest.TestCase):
    def test_parse_formats(self):
        cases = {"US distance_cm=23.4": 23.4, "DIST:12": 12.0, "ultrasonic 7.5": 7.5,
                 "STATUS moving=0 position_pulses=166400 move_size=166400": None,
                 "STATUS moving=0 position_pulses=0 us_cm=15": 15.0, "UP complete": None, "US distance_cm=0": None}
        for line, expected in cases.items():
            self.assertEqual(backends.parse_ultrasonic(line), expected, line)

    def test_freshness_uses_clock(self):
        now = [100.0]
        sensor = backends.Ultrasonic(stale_sec=0.5, clock=lambda: now[0])
        self.assertFalse(sensor.fresh())
        self.assertTrue(sensor.feed("US distance_cm=30"))
        self.assertTrue(sensor.fresh())
        now[0] += 0.6
        self.assertFalse(sensor.fresh())
        self.assertFalse(sensor.feed("UP start"))

    def test_approach_stops_at_threshold(self):
        rehearsal, scenario, field, *_ = make(ultrasonic=backends.FakeUltrasonic([40, 30, 20, 9, 7.5]))
        run(rehearsal, scenario, only=["S03"])
        result = rehearsal.results[0]
        self.assertEqual(result["status"], "OK")
        self.assertIn("7.5", result["detail"])
        self.assertIn(("straight_until", 0.6), field.calls)

    def test_stale_sensor_blocks_approach(self):
        rehearsal, scenario, field, *_ = make(ultrasonic=backends.FakeUltrasonic(stale=True))
        meta = run(rehearsal, scenario, only=["S03"])
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("straight_until", [c[0] for c in field.calls])

    def test_never_reaching_threshold_fails(self):
        rehearsal, scenario, field, *_ = make(ultrasonic=backends.FakeUltrasonic([50]), answers=["", "", "", "s"])
        run(rehearsal, scenario, only=["S03"])
        self.assertEqual(rehearsal.results[0]["status"], "MANUAL")
        self.assertIn("max distance", rehearsal.results[0]["detail"])

    def test_lift_uno_hands_ultrasonic_lines_to_sensor(self):
        sensor = backends.Ultrasonic()
        printed = []
        lift = backends.LiftUno("/dev/null", output=printed.append, line_hook=sensor.feed)
        lift._append("US distance_cm=11")
        lift._append("UP complete")
        self.assertEqual(sensor.reading()[0], 11.0)
        self.assertEqual([line for _, line in lift.lines], ["UP complete"])

    def test_status_line_with_distance_feeds_sensor_and_stays_for_lift(self):
        sensor = backends.Ultrasonic()
        lift = backends.LiftUno("/dev/null", output=lambda _l: None, line_hook=sensor.feed)
        lift._append("STATUS moving=0 position_pulses=0 reference=CONFIRMED move_size=166400 us_cm=14")
        self.assertEqual(sensor.reading()[0], 14.0)
        self.assertTrue(lift.lines[-1][1].startswith("STATUS"))


class FakeUnoConn:
    """POSITION_GUARD_V1 응답을 흉내 내는 가짜 시리얼."""

    def __init__(self, move_size=166400, complete=True):
        self.out = queue.Queue()
        self.position, self.reference, self.move_size, self.complete = None, "REQUIRED", move_size, complete
        self.written = []

    def write(self, data):
        command = data.decode().strip()
        self.written.append(command)
        if command == "s":
            pos = 0 if self.position is None else self.position
            self.out.put(f"STATUS moving=0 direction=NONE position_pulses={pos} reference={self.reference} "
                         f"move_size={self.move_size}")
        elif command == "b":
            self.position, self.reference = 0, "CONFIRMED"
            self.out.put("BOTTOM_REFERENCE_CONFIRMED position_pulses=0")
        elif command in ("u", "d"):
            word = "UP" if command == "u" else "DOWN"
            self.out.put(f"{word} start")
            if self.complete:
                self.position = self.move_size if command == "u" else 0
                self.out.put(f"{word} complete position_pulses={self.position}")
        elif command == "!":
            self.out.put("STOPPED")
        return len(data)

    def flush(self):
        pass

    def readline(self):
        try:
            return (self.out.get(timeout=0.05) + "\n").encode()
        except queue.Empty:
            return b""

    def close(self):
        pass


class LiftUnoTest(unittest.TestCase):
    def make(self, **conn_kwargs):
        lift = backends.LiftUno("/dev/null", output=lambda _l: None)
        lift.conn = FakeUnoConn(**conn_kwargs)
        threading.Thread(target=lift._reader, daemon=True).start()
        return lift

    def test_reference_up_down(self):
        lift = self.make()
        self.assertEqual(lift.status()["reference"], "REQUIRED")
        lift.set_reference("bottom")
        self.assertEqual(lift.move("up", 2.0)["position_pulses"], "166400")
        self.assertEqual(lift.move("down", 2.0)["position_pulses"], "0")
        lift.close()

    def test_missing_complete_sends_emergency_stop(self):
        lift = self.make(complete=False)
        lift.set_reference("bottom")
        with self.assertRaises(backends.BackendError):
            lift.move("up", 0.3)
        self.assertEqual(lift.conn.written[-1], "!")
        lift.close()


class ScenarioTest(unittest.TestCase):
    def test_scenario_matches_registered_waypoints_and_floors(self):
        scenario = runner.load_scenario(SCENARIO)
        registry = json.loads(REGISTRY.read_text(encoding="utf-8"))["waypoints"]
        floor = "F1"
        for step in scenario["steps"]:
            if step["type"] == "map":
                floor = step["floor"]
            if "waypoint" in step:
                self.assertIn(step["waypoint"], registry, step["id"])
                self.assertEqual(registry[step["waypoint"]]["floor"], floor, step["id"])

    def test_target_floor_is_4_going_up_and_1_going_down(self):
        scenario = runner.load_scenario(SCENARIO)
        car = [runner.BUTTON_LABELS[(s["stage"], s["direction"])] for s in scenario["steps"]
               if s["type"] == "button" and s["stage"] == "car"]
        self.assertEqual(car, ["4", "1"])

    def test_state_before(self):
        steps = runner.load_scenario(SCENARIO)["steps"]
        self.assertEqual(runner.state_before(steps, "S14"),
                         {"floor": "F2", "lift": "top", "waypoint": "f2_elevator_exit"})
        self.assertEqual(runner.state_before(steps, "S01"), {"floor": "F1", "lift": "bottom", "waypoint": None})

    def test_mid_run_start_sets_pose_and_loads_floor_map(self):
        rehearsal, scenario, field, _, lift, _ = make(loaded="F1")
        run(rehearsal, scenario, start="S14", end="S14")
        self.assertIn(("load_map", "/ros2_ws/maps/field/f2/pinned.yaml"), field.calls)
        self.assertIn(("pose", -11.375, -2.525, -0.829), field.calls)
        self.assertEqual(rehearsal.results[0]["status"], "OK")

    def test_only_lift_down_expects_fork_on_top(self):
        rehearsal, scenario, _, _, lift, _ = make()
        run(rehearsal, scenario, only=["S16"])
        self.assertIn(("ref", "top"), lift.calls)
        self.assertEqual(rehearsal.results[0]["status"], "OK")

    def test_nav2_start_is_retried_once_when_base_is_slow(self):
        rehearsal, scenario, field, *_ = make(nav_running=False)
        original = field.start_background
        calls = []

        def flaky(key, args, pattern, timeout, log_dir):
            calls.append(key)
            if len(calls) == 1:
                raise backends.BackendError("nav exited early")
            return original(key, args, pattern, timeout, log_dir)

        field.start_background = flaky
        run(rehearsal, scenario, only=["S02"])
        self.assertEqual(calls, ["nav_F1", "nav_F1"])
        self.assertEqual(rehearsal.results[0]["status"], "OK")

    def test_nav_restart_mode_restarts_instead_of_load_map(self):
        rehearsal, scenario, field, *_ = make(nav_restart=True)
        run(rehearsal, scenario, only=["S13"])
        names = [c[0] for c in field.calls]
        self.assertIn("nav_stop", names)
        self.assertNotIn("load_map", names)
        self.assertIn(("start", "nav_F2", "nav", "start", "F2"), field.calls)

    def test_bad_scenario_is_rejected(self):
        bad = {"steps": [{"id": "X", "type": "button", "stage": "car", "direction": "sideways", "title": "x"}]}
        path = Path(tempfile.mkdtemp()) / "bad.json"
        path.write_text(json.dumps(bad), encoding="utf-8")
        with self.assertRaises(ValueError):
            runner.load_scenario(path)


class StraightMoveTest(unittest.TestCase):
    def test_limits(self):
        with self.assertRaises(ValueError):
            ros_straight_move.validate(1.5, 0.05)
        with self.assertRaises(ValueError):
            ros_straight_move.validate(0.4, 0.2)
        ros_straight_move.validate(-0.4, 0.05)

    def test_speed_profile_stays_in_bounds(self):
        for elapsed in (0.0, 0.5, 3.0):
            for remaining in (0.001, 0.05, 0.5):
                v = ros_straight_move.plan_speed(elapsed, remaining, 0.05)
                self.assertGreaterEqual(v, ros_straight_move.MIN_SPEED_MPS)
                self.assertLessEqual(v, 0.05)


if __name__ == "__main__":
    unittest.main(verbosity=1)
