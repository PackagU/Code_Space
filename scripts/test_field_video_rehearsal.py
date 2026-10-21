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
from field_video import ros_map_switch, ros_nav_idle, ros_straight_move  # noqa: E402

SCENARIO = HERE / "field_video" / "scenario.json"
REGISTRY = HERE.parent / "src" / "slam_pkg" / "maps" / "field" / "waypoints.json"


def make(answers=None, failures=None, ultrasonic=None, nav_restart=False, auto=True, ros_arm=False, **field_kwargs):
    scenario = runner.load_scenario(SCENARIO)
    tmp = tempfile.mkdtemp(prefix="fv_test_")
    out = []
    field = backends.FakeField(failures, **field_kwargs)
    web = backends.FakeWeb(failures, ros_arm=ros_arm)
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
                self.assertEqual(calls[index - 2], ("idle",), call)
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
        self.assertEqual(meta["retries"], 1)  # 자동 재시도도 판정에 들어간다 (Codex #11)
        self.assertFalse(meta["verdict"].startswith("한 번에"))
        locker = next(r for r in rehearsal.results if r["id"] == "S02")
        self.assertIn("자동 재시도 1회", locker["detail"])
        self.assertIn(("clear",), field.calls)

    def test_goal_failing_twice_shows_menu_and_retry_works(self):
        rehearsal, scenario, field, *_ = make(failures={"goal:f1_locker": 2}, answers=["", "", "r"])
        meta = run(rehearsal, scenario, only=["S02"])
        result = rehearsal.results[0]
        self.assertEqual((result["status"], result["attempts"]), ("OK", 2))
        self.assertEqual(meta["retries"], 2)
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


class CodexReviewRegressionTest(unittest.TestCase):
    """2026-10-21 Codex 검토 지적 재발 방지."""

    def test_idle_check_failure_blocks_motion(self):  # 1
        rehearsal, scenario, field, *_ = make(failures={"idle": 1}, answers=["", "", "q"])
        run(rehearsal, scenario, only=["S02"])
        names = [c[0] for c in field.calls]
        self.assertNotIn("goal", names)
        first_idle = names.index("idle")
        self.assertNotIn("resume", names[:first_idle + 1])

    def test_prep_stops_and_checks_idle_before_map_or_pose(self):  # 2
        rehearsal, scenario, field, *_ = make(loaded="F1")
        run(rehearsal, scenario, start="S14", end="S14")
        names = [c[0] for c in field.calls]
        self.assertLess(names.index("stop"), names.index("load_map"))
        self.assertLess(names.index("idle"), names.index("load_map"))
        self.assertLess(names.index("idle"), names.index("pose"))

    def test_failed_teleop_blocks_auto_map_switch(self):  # 4
        rehearsal, scenario, field, *_ = make(teleop_rc=127, answers=["", "q"])
        meta = run(rehearsal, scenario, start="S12", end="S13")
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("load_map", [c[0] for c in field.calls])

    def test_operator_must_confirm_exit_marker(self):  # 4
        rehearsal, scenario, field, *_ = make(answers=["", "", "r", "r", ""])
        run(rehearsal, scenario, start="S12", end="S13")
        teleops = [c for c in field.calls if c[0] == "teleop"]
        self.assertEqual(len(teleops), 2)
        self.assertEqual(rehearsal.results[0]["attempts"], 2)

    def test_lift_failure_sends_bang(self):  # 5
        rehearsal, scenario, _, _, lift, _ = make(failures={"move:up": 1}, answers=["", "", "q"])
        run(rehearsal, scenario, only=["S04"])
        self.assertIn(("estop",), lift.calls)

    def test_ros_arm_with_nav_off_aborts_with_order(self):  # 8
        rehearsal, scenario, field, *_ = make(ros_arm=True, nav_running=False)
        meta = run(rehearsal, scenario)
        self.assertIn("--ros-arm", meta["verdict"])
        self.assertNotIn("start", [c[0] for c in field.calls])

    def test_nav_restart_with_ros_arm_is_refused(self):  # 8
        rehearsal, scenario, *_ = make(ros_arm=True, nav_restart=True)
        meta = run(rehearsal, scenario)
        self.assertIn("--nav-restart", meta["verdict"])

    def test_lift_reply_with_distance_is_kept(self):  # 10
        sensor = backends.Ultrasonic()
        lift = backends.LiftUno("/dev/null", output=lambda _l: None, line_hook=sensor.feed)
        for line in ("UP complete position_pulses=166400 distance_cm=14",
                     "BOTTOM_REFERENCE_CONFIRMED position_pulses=0 distance_cm=14"):
            lift._append(line)
        self.assertEqual([l for _, l in lift.lines][-2:],
                         ["UP complete position_pulses=166400 distance_cm=14",
                          "BOTTOM_REFERENCE_CONFIRMED position_pulses=0 distance_cm=14"])
        self.assertTrue(backends.is_ultrasonic_only("US distance_cm=3"))
        self.assertFalse(backends.is_ultrasonic_only("UP complete"))

    def test_unexpected_error_still_stops_and_writes_summary(self):
        rehearsal, scenario, field, *_ = make()

        def boom(*_a):
            raise ZeroDivisionError("bug")

        rehearsal.do_wait = boom
        meta = run(rehearsal, scenario, only=["S08"])
        self.assertIn("ZeroDivisionError", meta["verdict"])
        self.assertIn(("stop",), field.calls)
        self.assertTrue((rehearsal.log.dir / "summary.md").exists())

    def test_helper_pure_functions(self):
        def info(uuid):
            return type("I", (), {"goal_id": type("U", (), {"uuid": uuid})()})()

        class S:
            def __init__(self, uuid, status):
                self.status, self.goal_info = status, info(uuid)
        array = type("A", (), {"status_list": [S([1], 2), S([2], 5)]})()
        self.assertEqual(ros_nav_idle.unfinished(["01", "02"], array), ["01"])  # 01 still executing
        self.assertEqual(ros_nav_idle.unfinished(["01"], None), ["01"])  # no status = unconfirmed (2nd #1)
        canceling = type("A", (), {"status_list": [S([3], 3), S([4], 4)]})()
        self.assertEqual(ros_nav_idle.active_in(canceling), ["03"])  # already CANCELING is still waited for (3rd #1)
        self.assertEqual(ros_nav_idle.active_in(None), [])
        self.assertEqual(ros_nav_idle.foreign_publishers({"velocity_smoother", "packagu_field_web_ui"}),
                         ["packagu_field_web_ui"])
        self.assertTrue(ros_map_switch.same_size((10.0, 5.0), (10.1, 4.95)))

        def pose(x, y):
            return type("P", (), {"position": type("Q", (), {"x": x, "y": y})()})()
        grid = type("G", (), {"info": type("I", (), {"width": 4, "height": 2, "resolution": 0.05,
                                                     "origin": pose(-1.0, -2.0)})(),
                              "data": [0, 100, 0, 100, -1, 0, 100, 0]})()
        meta = type("M", (), {"size_x": 4, "size_y": 2, "resolution": 0.05, "origin": pose(-1.0, -2.0)})()
        fresh = [0, 254, 0, 254, 255, 0, 254, 0]
        stale = [0, 0, 254, 0, 255, 0, 0, 0]
        self.assertEqual(ros_map_switch.costmap_matches_map(meta, fresh, grid), (True, 1.0))
        self.assertEqual(ros_map_switch.costmap_matches_map(meta, stale, grid), (False, 0.0))
        other = type("M", (), {"size_x": 8, "size_y": 2, "resolution": 0.05, "origin": pose(-1.0, -2.0)})()
        self.assertEqual(ros_map_switch.costmap_matches_map(other, fresh, grid), (False, None))
        self.assertFalse(ros_map_switch.same_size((10.0, 5.0), (20.0, 5.0)))


class CodexSecondReviewTest(unittest.TestCase):
    """2026-10-21 Codex 2차 검토 지적 재발 방지."""

    def test_lift_only_run_stops_before_opening_lift(self):  # 2
        rehearsal, scenario, field, _, lift, _ = make()
        original = lift.open
        lift.open = lambda: (field.calls.append(("lift_open",)), original())
        run(rehearsal, scenario, only=["S04"])
        names = [c[0] for c in field.calls]
        self.assertLess(names.index("stop"), names.index("lift_open"))
        self.assertLess(names.index("idle"), names.index("lift_open"))

    def test_unconfirmed_lift_stop_is_retried_by_safe_stop(self):  # 3
        rehearsal, scenario, _, _, lift, _ = make(failures={"move:up": 1, "stopconfirm": 1}, answers=["", "", "q"])
        run(rehearsal, scenario, only=["S04"])
        self.assertIn(("stop_confirm",), lift.calls)

    def test_clear_failure_after_pose_fails_map_step(self):  # 5
        rehearsal, scenario, field, *_ = make(failures={"clear": 1}, answers=["q"])
        meta = run(rehearsal, scenario, only=["S13"])
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertEqual(rehearsal.results, [])

    def test_confirm_quit_aborts_before_map_switch(self):  # 8
        rehearsal, scenario, field, *_ = make(answers=["", "", "q"])
        meta = run(rehearsal, scenario, start="S12", end="S13")
        self.assertTrue(meta["verdict"].startswith("중단"))
        self.assertNotIn("load_map", [c[0] for c in field.calls])

    def test_confirm_other_input_asks_again(self):  # 8
        rehearsal, scenario, field, *_ = make(answers=["", "", "x", ""])
        run(rehearsal, scenario, only=["S12"])
        self.assertEqual(rehearsal.results[0]["status"], "OK")

    def test_nav_restart_refused_when_arm_node_runs_without_button_steps(self):  # 9
        rehearsal, scenario, field, *_ = make(nav_restart=True, arm_node=True)
        meta = run(rehearsal, scenario, only=["S13"])
        self.assertIn("--nav-restart", meta["verdict"])
        self.assertNotIn("nav_stop", [c[0] for c in field.calls])

    def test_cleanup_continues_when_record_stop_fails(self):  # 10
        rehearsal, scenario, field, _, lift, _ = make()

        def broken():
            raise RuntimeError("recorder hung")

        field.record_stop = broken
        meta = run(rehearsal, scenario, only=["S04"])
        self.assertIn(("close",), lift.calls)
        self.assertTrue((rehearsal.log.dir / "summary.md").exists())
        self.assertTrue(meta["verdict"].startswith("정리 실패"), meta["verdict"])  # 3차 #3
        self.assertEqual(len(meta["cleanup_errors"]), 1)

    def test_real_lift_keeps_moving_flag_when_stop_cannot_be_sent(self):  # 3
        lift = backends.LiftUno("/dev/null", output=lambda _l: None)
        lift.conn = FakeUnoConn(complete=False)
        threading.Thread(target=lift._reader, daemon=True).start()
        lift.set_reference("bottom")

        def failing_write(data):
            raise OSError("write failed")

        threading.Timer(0.2, lambda: (setattr(lift.conn, "write", failing_write),
                                      lift._append("SERIAL_ERROR device disconnected"))).start()
        with self.assertRaises(backends.BackendError):
            lift.move("up", 5.0)
        self.assertTrue(lift.moving)
        self.assertTrue(lift.stop_unconfirmed)
        lift.close()


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
        self.assertEqual(lift.conn.written[-2:], ["!", "s"])  # '!' 뒤 STATUS 로 정지 확인
        self.assertFalse(lift.moving)
        self.assertFalse(lift.stop_unconfirmed)
        lift.close()


class LiftSerialErrorTest(unittest.TestCase):
    def test_serial_error_during_move_sends_bang(self):  # 5
        lift = backends.LiftUno("/dev/null", output=lambda _l: None)
        lift.conn = FakeUnoConn(complete=False)
        threading.Thread(target=lift._reader, daemon=True).start()
        lift.set_reference("bottom")
        threading.Timer(0.2, lambda: lift._append("SERIAL_ERROR device disconnected")).start()
        with self.assertRaises(backends.BackendError):
            lift.move("up", 5.0)
        self.assertEqual(lift.conn.written[-2:], ["!", "s"])
        self.assertFalse(lift.moving)
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
