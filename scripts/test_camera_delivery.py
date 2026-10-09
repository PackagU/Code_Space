#!/usr/bin/env python3
"""Field coordinates, website selection and ROS arm ownership without physical devices."""
import json
import math
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/robot_arm_pkg'))
sys.path.insert(0, str(ROOT/'test_workspace/elevator_mission/src/elevator_mission_pkg'))
sys.path.insert(0, str(ROOT/'tools/button_arm_test'))
from prepare_camera_delivery import prepare
from robot_arm_pkg.arm_execution import ArmExecutionContract
from robot_arm_pkg.arm_sequence import parse_arm_command
from robot_arm_pkg import servo_protocol as sp
from elevator_mission_pkg.camera_web_client import CameraWebClient
from ros_arm import Session


class Tests(unittest.TestCase):
    def test_existing_field_coordinates_and_physical_floor_are_preserved(self):
        registry = json.loads((ROOT/'src/slam_pkg/maps/field/waypoints.json').read_text(encoding='utf-8'))
        pins = json.loads((ROOT/'src/slam_pkg/maps/field/map_pins.json').read_text(encoding='utf-8'))
        points, maps, missions, args = prepare(registry, pins)
        self.assertEqual(args['observed_target_floor'], 'F4')
        self.assertEqual(missions['missions']['field_camera_delivery']['target_floor'], 'F2')
        for name in ('f1_initial_test', 'f1_locker', 'f2_delivery_left_room4'):
            original = registry['waypoints'][name]
            point = points['floors'][original['floor']]['points'][name]
            self.assertEqual((point['x'], point['y']), (original['x'], original['y']))
            self.assertAlmostEqual(math.radians(point['yaw_deg']), original['yaw_rad'])
        for floor in ('F1', 'F2'):
            self.assertEqual(maps['floors'][floor]['map_yaml'], pins['floors'][floor]['yaml'])
            self.assertEqual(points['floors'][floor]['points']['elevator_inside'],
                             points['floors'][floor]['points'][floor.lower()+'_elevator_inside'])
        del registry['waypoints']['f2_elevator_inside']
        with self.assertRaises(KeyError):
            prepare(registry, pins)

    def test_website_http_target_and_invalid_detections(self):
        pose = dict(sp.POSES_1['press'])
        state = dict(error=None, target='4', token='fresh', captured_at=time.time(), ready=dict(label='4', press=pose))
        posts = []
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_GET(self):
                body = json.dumps(state).encode()
                self.send_response(200); self.end_headers(); self.wfile.write(body)
            def do_POST(self):
                posts.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
                self.send_response(200); self.end_headers(); self.wfile.write(b'{"ok":true}')
        with ThreadingHTTPServer(('127.0.0.1', 0), Handler) as server:
            thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
            try:
                web = CameraWebClient('http://127.0.0.1:%s' % server.server_port)
                web.select('4', 'F4')
                self.assertEqual(posts, [dict(label='4', floor_target='F4')])
                self.assertEqual(web.pose('4', state['captured_at']-.1), pose)
                for change in (dict(target='3'), dict(token=None), dict(error='camera lost'),
                               dict(captured_at=time.time()-2), dict(captured_at=time.time()+10),
                               dict(ready=dict(label='3', press=pose)),
                               dict(ready=dict(label='4', press=dict(pose, **{'001': True}))),
                               dict(ready=dict(label='4', press=dict(pose, **{'002': 9000})))):
                    old = dict(state); state.update(change)
                    self.assertIsNone(web.pose('4', 0), change)
                    state.clear(); state.update(old)
            finally:
                server.shutdown(); thread.join(timeout=2)

    def test_calibrated_press_uses_taught_pose_and_existing_ros_home(self):
        class Driver:
            positions = dict(sp.HOME)
            sent = []
            def send_pose(self, name, duration, poses):
                self.positions = dict(poses[name]); self.sent.append((name, dict(poses[name])))
            def read_positions(self): return dict(self.positions)
            def stop_all(self): pass
        driver = Driver()
        contract = ArmExecutionContract(driver=driver, require_homed=False)
        pose = {'000':1590,'001':1730,'002':1700,'003':1700}
        status = contract.submit(dict(request_id='taught-4', action='press', target='destination',
                                      button='4', press_cycle=2, press_pose=pose), 0)
        now = 0
        while status['event'] != 'completed':
            now += 2500
            status = contract.tick(now) or status
        self.assertIn(('press', pose), driver.sent)
        self.assertEqual(driver.sent[-1], ('home', sp.HOME))
        self.assertEqual(status['completion_basis'], 'controller_position_response')

    def test_invalid_dynamic_poses_are_rejected_before_sending(self):
        for pose in (dict(sp.HOME, **{'002':2600}), dict(sp.HOME, **{'001':True}), {'000':1500}):
            with self.assertRaises(ValueError):
                parse_arm_command(json.dumps(dict(request_id='bad', action='press', target='destination',
                                                  button='4', press_cycle=1, press_pose=pose)))
        for value in (0, 6000, True):
            with self.assertRaises(ValueError):
                parse_arm_command(json.dumps(dict(request_id='jog', action='move', pose=sp.HOME, duration_ms=value)))

    def session(self, response_changes=None):
        session = Session.__new__(Session)
        session.condition, session.io = threading.Condition(), threading.Lock()
        session.arm, session.mission, session.responses = {}, {}, {}
        session.String = types.SimpleNamespace
        session.node = types.SimpleNamespace(count_subscribers=lambda _: 1)
        sent = []
        def publish(msg):
            command = json.loads(msg.data); sent.append(command)
            status = dict(command, event='completed', completion_basis='controller_position_response',
                          simulation_mode=False, measured_pwm=dict(sp.HOME), current_view=command.get('view', ''))
            status.update(response_changes or {})
            session.on_arm(types.SimpleNamespace(data=json.dumps(status)))
        session.pub = types.SimpleNamespace(publish=publish)
        return session, sent

    def test_ros_website_never_opens_serial_and_blocks_manual_mission_interference(self):
        session, sent = self.session()
        self.assertEqual(session.call('view', view='floor_view'), sp.HOME)
        self.assertEqual(sent[0]['view'], 'floor_view')
        session.mission = dict(mission_active=True)
        with self.assertRaises(RuntimeError): session.call('home')
        self.assertEqual(len(sent), 1)
        session.mission = {}
        session.node.count_subscribers = lambda _: 2
        with self.assertRaises(RuntimeError): session.call('home')
        self.assertEqual(len(sent), 1)

    def test_ros_completion_requires_real_pose_feedback(self):
        for change in (dict(simulation_mode=True), dict(current_view='front_view'),
                       dict(completion_basis='timing'), dict(measured_pwm=None), dict(action='home')):
            session, _ = self.session(change)
            with self.assertRaises(RuntimeError): session.call('view', view='floor_view')


if __name__ == '__main__':
    unittest.main()
