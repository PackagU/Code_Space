#!/usr/bin/env python3
"""Real ROS/HTTP integration with synthetic frames and fake hardware only.

Never opens a camera, serial device or physical Nav2 server. Run in an isolated
ROS_DOMAIN_ID (the runner defaults to 229) on a development machine.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer
from unittest.mock import patch

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose

ROOT = Path(__file__).resolve().parents[1]
for folder in ('src/robot_arm_pkg', 'test_workspace/elevator_mission/src/elevator_mission_pkg', 'tools/button_arm_test'):
    sys.path.insert(0, str(ROOT/folder))
from prepare_camera_delivery import prepare
from robot_arm_pkg import servo_protocol as sp
from robot_arm_pkg.arm_sequence_node import ArmSequenceNode
from elevator_mission_pkg.delivery_mission_node import DeliveryMissionNode
from elevator_mission_pkg.floor_reader_bridge import FloorReaderBridge
from ros_arm import Session
import app
import synth


def main():
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('CAMERA_TEST_ROS_DOMAIN_ID', '229')
    commands, goals, switches, map_states = [], [], [], []
    nodes, threads = [], []
    stop = threading.Event()
    session = executor = server = mission = arm = None
    with tempfile.TemporaryDirectory() as folder:
        tmp = Path(folder)
        registry = json.loads((ROOT/'src/slam_pkg/maps/field/waypoints.json').read_text(encoding='utf-8'))
        pins = json.loads((ROOT/'src/slam_pkg/maps/field/map_pins.json').read_text(encoding='utf-8'))
        points, maps, missions, _ = prepare(registry, pins)
        for name, value in (('points', points), ('missions', missions)):
            (tmp/(name+'.yaml')).write_text(yaml.safe_dump(value), encoding='utf-8')
        state = app.App(argparse.Namespace(source='synthetic', data=str(tmp/'button'),
            floor_data=str(tmp/'floor'), execute=False, ros_arm=True,
            servo_source=str(ROOT/'tools/button_arm_test/servo_test.py'), port='unused'))
        state.floor.roi = [[0,0],[1,0],[1,1],[0,1]]
        state.arrow_poses['UP'] = dict(sp.POSES_1['press'])
        for seed in (1000,1001,1002):
            state.process(synth.scene(('1','2','3','4'), seed)[0]); state.enroll('1234')
        server = ThreadingHTTPServer(('127.0.0.1', 0), app.handler(state))
        url = 'http://127.0.0.1:%s' % server.server_port
        http_thread = threading.Thread(target=server.serve_forever, daemon=True); http_thread.start()
        rclpy.init(args=['--ros-args', '-p', 'camera_elevator_mode:=true', '-p', 'dry_run_nav2:=false',
            '-p', 'vision_button_press:=true', '-p', 'routing_mode:=nav2', '-p', 'observed_target_floor:=F4',
            '-p', 'mission_id:=field_camera_delivery', '-p', 'tick_period_sec:=0.05',
            '-p', 'workspace_root:='+str(ROOT/'test_workspace/elevator_mission'),
            '-p', 'points_yaml:='+str(tmp/'points.yaml'), '-p', 'missions_yaml:='+str(tmp/'missions.yaml'),
            '-p', 'camera_web_url:='+url, '-p', 'reader_url:='+url+'/floor/api/state',
            '-p', 'serial_port:=fake-test-only', '-p', 'enable_floor_trigger:=false',
            '-p', 'camera_pose_config:='+str(ROOT/'src/robot_arm_pkg/config/camera_views.json')])
        try:
            probe = Node('packagu_camera_test_domain_probe')
            time.sleep(1)
            existing = [name for name in probe.get_node_names() if name != probe.get_name()]
            probe.destroy_node()
            assert not existing, 'test domain is in use: ' + str(existing)
            class Driver:
                def __init__(self, *args, **kwargs): self.positions = dict(sp.HOME)
                def send_pose(self, name, duration, poses): self.positions = dict(poses[name])
                def read_positions(self): return dict(self.positions)
                def stop_all(self): pass
                def close(self): pass
            with patch.object(sp, 'SerialPoseDriver', Driver), \
                    patch.object(ArmSequenceNode, '_now_ms', staticmethod(lambda: time.monotonic()*50000)):
                arm = ArmSequenceNode(); nodes.append(arm)
                # Keep the accelerated clock on this test instance after leaving the patch.
                arm._now_ms = lambda: time.monotonic()*50000
            submit = arm._contract.submit
            def record(command, now):
                commands.append(json.loads(command) if isinstance(command, str) else dict(command))
                return submit(command, now)
            arm._contract.submit = record
            fake = Node('floor_orchestrator_node'); nodes.append(fake)
            fake.declare_parameter('target_floor', 'F2'); fake.declare_parameter('spawn_point_id', 'elevator_inside')
            status_pub = fake.create_publisher(String, '/floor_orchestrator/status', 10)
            active = [False]
            def switch(request, response):
                assert mission.arrival_gate.confirmed(time.monotonic(), time.time())
                assert fake.get_parameter('target_floor').value == 'F2'
                switches.append('F2'); active[0] = True
                response.success = True; return response
            fake.create_service(Trigger, '/floor_orchestrator/request_switch', switch)
            def status():
                msg = String(); msg.data = json.dumps(dict(current_floor='F2' if active[0] else 'F1', pending=False))
                status_pub.publish(msg)
            fake.create_timer(.1, status)
            def navigate(handle):
                assert arm._contract.current_view == 'front_view', 'drive before front-view completion'
                p = handle.request.pose.pose.position
                goals.append((p.x, p.y))
                handle.succeed(); return NavigateToPose.Result()
            action = ActionServer(fake, NavigateToPose, '/navigate_to_pose', execute_callback=navigate)
            mission = DeliveryMissionNode(); nodes.append(mission)
            bridge = FloorReaderBridge(); nodes.append(bridge)
            fake.create_subscription(String, '/elevator/camera_state', lambda msg: map_states.append(json.loads(msg.data)), 10)
            executor = MultiThreadedExecutor(num_threads=4)
            for node in nodes: executor.add_node(node)
            thread = threading.Thread(target=executor.spin, daemon=True); thread.start(); threads.append(thread)
            session = Session(state.source); state.session = session
            observed = ['3']
            def capture():
                while not stop.wait(.1):
                    if arm._contract.current_view == 'floor_view':
                        frame = np.zeros((480,640,3), np.uint8)
                        pattern = app.floor_app.pattern(observed[0]); frame[100:320,230:410] = pattern
                    else:
                        labels = ('UP','DOWN') if state.target in ('UP','DOWN') else ('1','2','3','4')
                        frame = synth.scene(labels, 1002)[0]
                    state.process(frame)
            capture_thread = threading.Thread(target=capture, daemon=True); capture_thread.start(); threads.append(capture_thread)
            deadline = time.monotonic()+25
            while not mission.arrival_gate.looking and not mission._done and time.monotonic() < deadline:
                time.sleep(.05)
            assert mission.arrival_gate.looking, 'mission failed before floor observation'
            with session.condition: session.mission = dict(mission_active=True)
            try: session.call('home'); raise AssertionError('manual arm accepted during mission')
            except RuntimeError: pass
            time.sleep(1.3)
            assert len(goals) == 3 and switches == [], 'wrong floor allowed map switch or exit'
            observed[0] = '4'
            deadline = time.monotonic()+15
            while not mission._done and time.monotonic() < deadline: time.sleep(.05)
            assert mission._done and mission.index == len(mission.sequence), (mission.index, state.state()['error'])
            expected = [points['floors'][floor]['points'][name] for floor,name in (
                ('F1','f1_locker'),('F1','elevator_entry'),('F1','elevator_inside'),
                ('F2','elevator_exit'),('F2','f2_delivery_left_room4'))]
            assert goals == [(p['x'],p['y']) for p in expected], goals
            assert switches == ['F2']
            presses = [c for c in commands if c['action'] == 'press']
            assert [c['button'] for c in presses] == ['UP','4'], presses
            assert presses[1]['press_pose'] == state.source['menu_press']['7']
            assert any(s['observed_floor']=='F4' and s['current_floor']=='F2' for s in map_states)
            # A real DDS round trip through the site's manual ROS client after completion.
            with session.condition: session.mission = dict(mission_active=False)
            assert session.read() == state.camera_views['front_view']
            assert session.view('floor_view', threading.Event()) == state.camera_views['floor_view']
            try:
                session.call('move', pose=dict(sp.HOME, **{'001': True}), duration_ms=200)
                raise AssertionError('invalid PWM accepted')
            except RuntimeError as exc:
                assert 'invalid_command' in str(exc), exc
            from rclpy.parameter import Parameter
            mission.set_parameters([Parameter('field_map_guard', value=True),
                Parameter('field_project_root', value=str(ROOT)),
                Parameter('field_pins', value=str(ROOT/'src/slam_pkg/maps/field/map_pins.json')),
                Parameter('field_registry', value=str(ROOT/'src/slam_pkg/maps/field/waypoints.json'))])
            with patch('subprocess.run') as check:
                check.return_value.returncode = 0
                assert mission.check_navigation_goal(mission.registry.get('F2','elevator_exit'))
                assert 'f2_elevator_exit' in check.call_args.args[0]
                check.return_value.returncode = 1
                check.return_value.stdout, check.return_value.stderr = 'wrong map', ''
                assert not mission.check_navigation_goal(mission.registry.get('F2','elevator_exit'))
            print('PASS ROS + shared HTTP + real button/floor recognition: wrong F3 holds; F4 -> map F2 -> front -> exit -> original destination')
        finally:
            stop.set()
            if session: session.close()
            if mission: mission.shutdown()
            if executor: executor.shutdown(timeout_sec=2)
            for thread in threads: thread.join(timeout=2)
            for node in reversed(nodes): node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            server.shutdown(); server.server_close(); http_thread.join(timeout=2)


if __name__ == '__main__':
    main()
