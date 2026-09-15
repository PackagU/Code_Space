import argparse
import json
import re
import tempfile
import threading
import unittest
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from unittest.mock import patch

import cv2
import numpy as np
import app
import arm
import calib
import synth
import vision

ROOT = Path(__file__).resolve().parent
SERVO = str(ROOT/'servo_test.py')


def args(data):
    return argparse.Namespace(data=data, source='synthetic', execute=False,
                              servo_source=SERVO, port='/dev/arm_servo')


def read_scene(labels, seed, templates):
    gray = cv2.cvtColor(synth.scene(labels, seed)[0], cv2.COLOR_BGR2GRAY)
    buttons = vision.reading_order(vision.find_buttons(gray))
    return [vision.read_button(gray, b, templates)[0] for b in buttons]


def enrolled_templates():
    templates = {}
    for seed in (1000, 1001, 1002):
        gray = cv2.cvtColor(synth.scene(vision.DIGITS, seed)[0], cv2.COLOR_BGR2GRAY)
        for b, label in zip(vision.reading_order(vision.find_buttons(gray)), vision.DIGITS):
            templates.setdefault(label, []).append(
                vision.prepare(vision.normalize(vision.extract_glyph(gray, b))))
    return templates


class VisionTests(unittest.TestCase):
    def test_arrows_without_enrollment(self):
        for seed in range(30):
            self.assertEqual(read_scene(('UP', 'DOWN'), seed, {}), ['UP', 'DOWN'], seed)
            self.assertEqual(read_scene(('DOWN', 'UP'), seed, {}), ['DOWN', 'UP'], seed)

    def test_digits_after_enrollment_any_layout(self):
        templates = enrolled_templates()
        for seed in range(40):
            order = tuple(np.random.default_rng(seed).permutation(vision.DIGITS))
            self.assertEqual(read_scene(order, seed, templates), list(order), seed)

    def test_digits_unknown_before_enrollment_and_blank_cap(self):
        self.assertEqual(read_scene(vision.DIGITS, 3, {}), ['UNKNOWN']*4)
        self.assertEqual(read_scene(('1', None), 3, enrolled_templates()), ['1', 'UNKNOWN'])

    def test_ignores_non_round_bright_things(self):
        frame, _ = synth.scene(('UP', 'DOWN'), 4)
        cv2.rectangle(frame, (250, 20), (600, 60), (240, 240, 240), -1)  # ceiling light strip
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.assertEqual(len(vision.find_buttons(gray)), 2)

    def test_gate_loss_switch_stale_and_recovery(self):
        gate = vision.Gate()
        for i in range(4):
            self.assertFalse(gate.update(('1', True), 1+i*.1))
        self.assertTrue(gate.update(('1', True), 1.4))
        self.assertFalse(gate.update(('1', False), 1.5))
        self.assertFalse(gate.update(None, 1.6))
        self.assertFalse(gate.update(('1', True), 3))
        self.assertEqual(gate.count, 1)


class CalibTests(unittest.TestCase):
    def points(self, pixels):
        # Ground truth: PWM linear in pixel position.
        def truth(u, v):
            return {'000': round(1500+u*.9), '001': round(1500+v*1.2), '002': round(1800-v*.5), '003': 1500}
        return truth, [{'px': list(p), 'radius': 80, 'press': truth(*p)} for p in pixels]

    def test_affine_recovers_linear_map_inside_only(self):
        truth, pts = self.points([(150, 120), (480, 120), (150, 360), (480, 360)])
        model = calib.fit(pts)
        self.assertEqual(model['kind'], 'affine')
        pose, reason = calib.predict(model, 300, 250, 82)
        self.assertIsNone(reason)
        for i in calib.IDS:
            self.assertLessEqual(abs(pose[i]-truth(300, 250)[i]), 1)
        self.assertIsNone(calib.predict(model, 630, 470, 80)[0])    # outside taught area
        self.assertIsNone(calib.predict(model, 300, 250, 120)[0])   # cap much bigger: board closer

    def test_line_and_single_point(self):
        truth, pts = self.points([(150, 120), (480, 120)])
        model = calib.fit(pts)
        self.assertEqual(model['kind'], 'line')
        self.assertEqual(calib.predict(model, 315, 120, 80)[0]['000'], truth(315, 120)['000'])
        self.assertIsNone(calib.predict(model, 315, 360, 80)[0])    # other row needs 3+ points
        single = calib.fit(pts[:1])
        self.assertEqual(calib.predict(single, 150, 125, 80)[0], pts[0]['press'])
        self.assertIsNone(calib.predict(single, 480, 120, 80)[0])
        self.assertIsNone(calib.fit([]))


class AppTests(unittest.TestCase):
    def build(self, data):
        state = app.App(args(data))
        for seed in (1000, 1001, 1002):
            state.process(synth.scene(vision.DIGITS, seed)[0])
            state.enroll('1234')
        return state

    def teach(self, state, point, pose):
        state.select(point)
        state.add_point(pose)

    def stable(self, state, frame):
        base = app.time.monotonic()
        for i in range(5):
            with patch('app.time.monotonic', return_value=base+i*.1):
                state.process(frame)

    def test_teach_find_press_swap_duplicate_stale_and_restore(self):
        with tempfile.TemporaryDirectory() as data:
            state = self.build(data)
            state.label_menu = {}                # pixel calibration path (fixed mapping tested below)
            self.teach(state, [.27, .25], '6')   # button 1 <- servo_test press
            self.teach(state, [.73, .25], '7')   # button 2 <- servo_test press2
            with self.assertRaises(ValueError):
                self.teach(state, [.73, .25], '6')   # same button twice
            with self.assertRaises(ValueError):
                state.select([.5, .5])               # not on a button
            state.set_mission('car', 'down')                # always floor 1
            self.assertEqual(state.target, '1')
            with self.assertRaises(ValueError):
                state.set_target('2')
            self.stable(state, synth.scene(('2', '1', '3', '4'), 1002)[0])
            press = state.ready['press']     # '1' now sits where press2 was taught (±few px)
            for i in arm.IDS:
                self.assertLessEqual(abs(press[i]-arm.protocol.POSES_2['press2'][i]), 10)
            with patch('arm.Session.press', side_effect=AssertionError('serial in dry-run')):
                preview = state.press(state.token)
            self.assertTrue(preview['dry_run'])
            self.assertEqual(preview['steps'][2]['payload'], '{%s}' % ''.join(
                '#%sP%04dT1000!' % (i, press[i]) for i in arm.IDS))
            with self.assertRaises(ValueError):
                state.press(None)
            self.stable(state, synth.scene(('3', '4', '1', '2'), 1002)[0])
            self.assertIsNotNone(state.ready)   # seen on bottom row, only a 2-point line taught
            self.assertIsNone(state.token)
            # Teach bottom-left with a jogged pose -> plane covers the board.
            state.commanded = {'000': 1600, '001': 2000, '002': 1700, '003': 1500}
            self.teach(state, [.27, .75], 'current')
            self.stable(state, synth.scene(('3', '4', '1', '2'), 1002)[0])
            self.assertIsNotNone(state.token)
            self.assertEqual(state.ready['press'], state.commanded)
            self.stable(state, synth.scene(('1', '1', '3', '4'), 1002)[0])
            self.assertIsNone(state.ready)      # duplicate target
            self.stable(state, synth.scene(vision.DIGITS, 1002)[0])
            state.updated -= 2
            self.assertIsNone(state.state()['ready'])
            restored = app.App(args(data))
            self.assertEqual(restored.state()['samples'], {'1': 3, '2': 3, '3': 3, '4': 3})
            self.assertEqual(restored.points, state.points)
            self.assertEqual(restored.model['kind'], 'affine')
            restored.delete_point(2)
            self.assertEqual(restored.model['kind'], 'line')

    def test_button_1_and_4_use_menu_poses_anywhere(self):
        with tempfile.TemporaryDirectory() as data:
            state = self.build(data)             # no calibration points at all
            for scene, mission, menu in (((('4', '3', '2', '1'), 7), ('car', 'down'), '6'),
                                         ((('1', '2', '4', '3'), 9), ('car', 'up'), '7')):
                state.set_mission(*mission)
                self.stable(state, synth.scene(*scene)[0])
                self.assertIsNotNone(state.token)
                self.assertEqual(state.ready['press'], state.source['menu_press'][menu])

    def test_arrow_fixed_pose_saved_by_jog(self):
        with tempfile.TemporaryDirectory() as data:
            state = app.App(args(data))          # dry-run: commanded starts at home
            state.goto('press_ready')
            state.jog('001', 50)
            state.save_arrow('DOWN')
            state.set_mission('hall', 'down')
            self.stable(state, synth.scene(('DOWN', 'UP'), 4)[0])
            self.assertEqual(state.ready['press'], state.commanded)
            self.assertIsNotNone(state.token)
            self.assertEqual(app.App(args(data)).arrow_poses, {'DOWN': state.commanded})

    def test_enroll_rejects_count_mismatch_and_keeps_three(self):
        with tempfile.TemporaryDirectory() as data:
            state = self.build(data)
            with self.assertRaises(ValueError):
                state.enroll('12')
            with self.assertRaises(ValueError):
                state.enroll('1123')
            state.process(synth.scene(vision.DIGITS, 5)[0])
            state.enroll('1234')
            self.assertEqual(len(list(Path(data).glob('template_1_*.png'))), 3)

    def test_arrow_target_without_enrollment(self):
        with tempfile.TemporaryDirectory() as data:
            state = app.App(args(data))
            state.process(synth.scene(('UP', 'DOWN'), 2)[0])
            self.teach(state, [.27, .5], '7')     # pixel points never drive arrows
            state.set_mission('hall', 'down')
            self.stable(state, synth.scene(('DOWN', 'UP'), 2)[0])
            self.assertIsNone(state.token)
            self.assertIn('미저장', state.ready['reason'])
            with self.assertRaises(ValueError):     # home's approach pose leaves PWM limits
                state.save_arrow('DOWN')

    def test_dry_run_jog_never_opens_serial(self):
        with tempfile.TemporaryDirectory() as data:
            state = app.App(args(data), driver_factory=lambda *a: self.fail('serial opened'))
            state.jog('001', 50)
            self.assertEqual(state.commanded['001'], arm.protocol.HOME['001']+50)
            with self.assertRaises(ValueError):
                state.jog('001', 37)
            state.commanded['002'] = 2590
            with self.assertRaises(ValueError):
                state.jog('002', 50)                 # PWM_LIMITS 900~2600
            state.goto('press_ready')
            self.assertEqual(state.commanded, arm.protocol.POSES_1['press_ready'])

    def test_http_cross_origin_busy_and_dry_request(self):
        with tempfile.TemporaryDirectory() as data:
            state = self.build(data)
            self.teach(state, [.73, .25], '7')
            state.set_mission('car', 'up')                  # always floor 4
            self.stable(state, synth.scene(('1', '4', '2', '3'), 1002)[0])
            server = app.ThreadingHTTPServer(('127.0.0.1', 0), app.handler(state))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            base = 'http://127.0.0.1:%d' % server.server_port
            def post(path, value, origin=None):
                headers = {'Content-Type': 'application/json'}
                if origin:
                    headers['Origin'] = origin
                return urlopen(Request(base+path, json.dumps(value).encode(), headers), timeout=3)
            try:
                with self.assertRaises(HTTPError):
                    post('/api/press', {'token': state.token}, 'http://other.example')
                result = json.loads(post('/api/press', {'token': state.token}).read())
                self.assertTrue(result['dry_run'])
                state.busy = True
                with self.assertRaises(HTTPError):
                    post('/api/jog', {'joint': '000', 'delta': 10})
                with self.assertRaises(HTTPError):
                    post('/api/mission', {'stage': 'car', 'direction': 'down'})
                post('/api/stop', {})
                self.assertTrue(state.cancel.is_set())
            finally:
                server.shutdown()
                server.server_close()
                thread.join()


class FakeDriver:
    def __init__(self, fail_after=None):
        self.moves, self.stopped, self.closed = [], False, False
        self.position, self.fail_after = dict(arm.protocol.HOME), fail_after

    def read_positions(self):
        if self.fail_after is not None and len(self.moves) > self.fail_after:
            return {sid: 900 for sid in arm.IDS}
        return dict(self.position)

    def send_pose(self, name, duration, poses):
        self.moves.append(name)
        self.position = dict(poses[name])

    def stop_all(self):
        self.stopped = True

    def close(self):
        self.closed = True


class NoWait:
    def is_set(self):
        return False

    def wait(self, seconds):
        return False


class ArmTests(unittest.TestCase):
    def test_servo_test_values_read_without_import(self):
        source = arm.load_source(SERVO)
        self.assertEqual(source['approach'], {'000': 0, '001': -300, '002': 100, '003': 0})
        with tempfile.TemporaryDirectory() as data:
            text = Path(SERVO).read_text(encoding='utf-8')
            file = Path(data)/'servo_test.py'
            self.assertIn('arm = ServoArm(PORT, BAUD)', text)  # would open serial if executed
            # Break menu 7's approach offset only (values themselves change in the field).
            broken = re.sub(r'("pre_press2": \{\s*"000": )(\d+)', lambda m: m.group(1)+str(int(m.group(2))+50), text)
            self.assertNotEqual(broken, text)
            file.write_text(broken, encoding='utf-8')
            with self.assertRaises(ValueError):
                arm.load_source(file)

    def test_calibrated_plan_equals_servo_test_menu_cycles(self):
        source = arm.load_source(SERVO)
        for cycle, press in ((1, 'press'), (2, 'press2')):
            poses = arm.protocol.get_poses(cycle)
            expected = [(ms, arm.protocol.pose_command(name, ms, poses) if send else None)
                        for name, ms, send in arm.protocol.get_cycle(cycle)]
            got = [(s['duration_ms'], s['payload']) for s in arm.plan(source, poses[press])]
            self.assertEqual(got, expected)
        with self.assertRaises(ValueError):  # approach adds +100 on 002 -> 2650 > 2600
            arm.plan(source, {'000': 1500, '001': 1900, '002': 2550, '003': 1500})

    def test_session_press_success_and_fault_stops_without_retreat(self):
        source = arm.load_source(SERVO)
        press = arm.protocol.POSES_1['press']
        driver = FakeDriver()
        arm.Session(source, 'FAKE', lambda *a: driver).press(press, NoWait(), lambda _: None)
        self.assertEqual(driver.moves, ['press_ready', 'pre_press', 'press', 'retreat', 'press_ready', 'home'])
        self.assertFalse(driver.stopped)
        driver = FakeDriver(fail_after=1)
        session = arm.Session(source, 'FAKE', lambda *a: driver)
        ticks = iter(range(100))
        with patch('arm.time.monotonic', side_effect=lambda: next(ticks)):
            with self.assertRaises(TimeoutError):
                session.press(press, NoWait(), lambda _: None)
        self.assertEqual(driver.moves, ['press_ready', 'pre_press'])
        self.assertTrue(driver.stopped and driver.closed)
        self.assertIsNone(session.driver)

    def test_save_jogged_pose_to_menu_7(self):
        with tempfile.TemporaryDirectory() as data, patch.dict(arm.protocol.POSES_2):
            servo, ref = Path(data)/'servo_test.py', Path(data)/'servo_reference.py'
            servo.write_bytes(Path(SERVO).read_bytes())
            ref.write_bytes(Path(arm.protocol.__file__).read_bytes())
            source = arm.load_source(servo)
            with self.assertRaises(ValueError):  # approach 002 +100 would exceed 2600
                arm.save_menu_press(servo, ref, source, '7', {'000': 1900, '001': 1900, '002': 2550, '003': 1500})
            self.assertEqual(servo.read_bytes(), Path(SERVO).read_bytes())
            press = {'000': 1850, '001': 1950, '002': 1760, '003': 1520}
            new = arm.save_menu_press(servo, ref, source, '7', press)
            self.assertEqual(new['menu_press']['7'], press)
            self.assertEqual(new['menu_press']['6'], source['menu_press']['6'])
            text = servo.read_text(encoding='utf-8')
            self.assertIn('"000": 1850,', text)
            self.assertIn('arm = ServoArm(PORT, BAUD)', text)
            self.assertNotIn('\r', text)
            self.assertEqual(len(list(Path(data).glob('*.bak_*'))), 2)
            self.assertEqual(arm.protocol.POSES_2['retreat2'], {'000': 1850, '001': 1650, '002': 1860, '003': 1520})

    def test_press_refuses_when_not_home(self):
        source = arm.load_source(SERVO)
        driver = FakeDriver()
        driver.position = dict(arm.protocol.POSES_1['press_ready'])
        with self.assertRaises(RuntimeError):
            arm.Session(source, 'FAKE', lambda *a: driver).press(
                arm.protocol.POSES_1['press'], NoWait(), lambda _: None)
        self.assertEqual(driver.moves, [])
        self.assertTrue(driver.stopped)


if __name__ == '__main__':
    unittest.main(verbosity=2)
