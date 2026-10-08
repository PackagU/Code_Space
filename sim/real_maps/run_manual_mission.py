#!/usr/bin/env python3
"""Operator rehearsal: Nav2 legs alternate with the real WASD teleop on a PTY.

Ground truth represents the simulated operator's visual feedback. It is never
fed into Nav2. Elevator travel is an explicit simulated floor selection.
"""
import argparse
import csv
import json
import math
import os
import pty
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, SetEntityState, GetEntityState
from lifecycle_msgs.srv import GetState
from run_scenarios import ROOT, HERE, LOG, MAPS, CONFIG, IDLE, TOPICS, Observer, Processes, guard, record_check, pose_values, lifecycle_start


def angle_error(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


class Mission:
    def __init__(self, name, goal_timeout, start_delay=10, spawn_pose=IDLE):
        self.directory = LOG/name
        if (self.directory/'bag').exists():
            raise RuntimeError('existing mission bag; choose a new --name')
        self.directory.mkdir(parents=True, exist_ok=True)
        self.node = Observer()
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.processes = Processes(self.directory)
        self.floor = 'F1'
        self.nav = None
        self.goal_timeout = goal_timeout
        self.start_delay = start_delay
        self.spawn_pose = spawn_pose
        self.stages = []
        self.stage_pub = self.node.create_publisher(String, '/sim/manual_stage', 10)
        self.key_pub = self.node.create_publisher(String, '/sim/manual_key', 10)
        self.payload_pub = self.node.create_publisher(String, '/sim/payload_state', 10)
        self.doors = json.loads((HERE/'generated/doors.json').read_text())
        self.waypoints = json.loads((MAPS/'waypoints.json').read_text())['waypoints']
        self.node.progress_path = self.directory/'progress.json'
        self.result = {'validation_scope': 'simulation', 'name': name, 'stages': self.stages,
                       'manual_controller': 'real keyboard_teleop.py PTY WASD; simulated operator uses ground truth',
                       'payload': 'manual handling marker; no arm actuation validation',
                       'floor_travel': 'explicit simulation teleport; no elevator hardware validation',
                       'floor_heading_method': 'inside-waypoint heading; preserve cabin-relative pose',
                       'door_world': 'F2 1.0m opening exception; original raw A/B world remains unchanged'}

    def service(self, srv, name, request):
        client = self.node.create_client(srv, name)
        if not self.node.wait(client.service_is_ready, 20):
            raise RuntimeError('service unavailable: '+name)
        future = client.call_async(request)
        if not self.node.wait(future.done, 20):
            raise RuntimeError('service timeout: '+name)
        response = future.result()
        self.node.destroy_client(client)
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError(name+': '+str(getattr(response, 'status_message', response)))
        return response

    def marker(self, name, mode):
        row = {'stage': name, 'mode': mode, 'floor': self.floor,
               'start_sim_s': self.node.get_clock().now().nanoseconds/1e9}
        self.stages.append(row)
        self.stage_pub.publish(String(data=json.dumps(row)))
        print('MISSION_STAGE '+json.dumps(row), flush=True)
        (self.directory/'stages.json').write_text(json.dumps(self.stages, indent=2))
        return row

    def stop(self):
        for _ in range(3):
            self.node.stop.publish(Bool(data=True))
            self.node.pause(.15)

    def resume(self):
        self.node.stop.publish(Bool(data=False))
        self.node.pause(.3)

    def bootstrap(self, start_nav=True):
        guard('F1', 'pre-nav', self.directory)
        self.processes.start('gazebo', ['gzserver', '--verbose', str(HERE/'generated/f1.world'),
                                      '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'])
        import yaml
        rsp = self.directory/'rsp.yaml'
        rsp.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
            'use_sim_time': True, 'robot_description': (HERE/'generated/robot.urdf').read_text()}}}))
        self.processes.start('rsp', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                                    '--ros-args', '--params-file', str(rsp)])
        spawn = self.processes.start('spawn', ['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity',
            'packagu_sim', '-file', str(HERE/'generated/robot.urdf'), '-x', str(self.spawn_pose[0]), '-y', str(self.spawn_pose[1]),
            '-z', '.005', '-Y', str(self.spawn_pose[2]), '-timeout', '60'])
        if not self.node.wait(lambda: self.node.truth is not None and self.node.scan is not None, 90):
            raise RuntimeError('ground truth/scan unavailable')
        if spawn.poll() not in (None, 0):
            raise RuntimeError('spawn failed')
        self.processes.start('bridge', [sys.executable, str(HERE/'opencr_shim.py'), '--ros-args',
            '--params-file', str(ROOT/'src/drive_pkg/config/drive_calib.yaml'),
            '-p', 'cmd_vel_topic:=/cmd_vel_safe', '-p', 'publish_tf:=false', '-p', 'use_sim_time:=true'])
        self.processes.start('gate', [sys.executable, str(ROOT/'src/drive_pkg/drive_pkg/nav_safety_gate.py'),
            '--ros-args', '--params-file', str(ROOT/'src/drive_pkg/config/nav_safety.yaml'),
            '-p', 'use_sim_time:=true'])
        self.node.pause(2)
        self.stop()
        self.load_boxes()
        self.spawn_doors()
        self.bag = self.directory/'bag'
        self.recorder = self.processes.start('record', ['ros2', 'bag', 'record', '--include-hidden-topics',
            '--max-cache-size', '262144', '--qos-profile-overrides-path',
            str(ROOT/'scripts/rosbag_qos_overrides.yaml'), '-o', str(self.bag), *TOPICS,
            '/sim/manual_stage', '/sim/manual_key', '/sim/payload_state'])
        record_check(self.recorder, self.bag, self.node, self.directory)
        self.node.measure = True
        if start_nav:
            self.start_nav(self.spawn_pose)

    def load_boxes(self):
        suffix = '_door_open' if self.floor == 'F2' else ''
        boxes = json.loads((HERE/f'generated/{self.floor.lower()}{suffix}_boxes.json').read_text())
        for positions in self.doors[self.floor]['leaves'].values():
            x,y,yaw = positions['open']
            boxes.append([x,y,self.doors[self.floor]['opening_m']/2,.025,yaw])
        self.node.boxes = np.array(boxes)

    def spawn_doors(self):
        for name in self.doors[self.floor]['leaves']:
            request = SpawnEntity.Request()
            request.name = name
            request.xml = (HERE/f'generated/{name}.sdf').read_text()
            self.service(SpawnEntity, '/spawn_entity', request)
        self.door_state('open')

    def door_state(self, mode):
        states = []
        for name, positions in self.doors[self.floor]['leaves'].items():
            x,y,yaw = positions[mode]
            request = SetEntityState.Request()
            request.state.name = name
            request.state.reference_frame = 'world'
            request.state.pose.position.x = x
            request.state.pose.position.y = y
            request.state.pose.orientation.z = math.sin(yaw/2)
            request.state.pose.orientation.w = math.cos(yaw/2)
            self.service(SetEntityState, '/gazebo/set_entity_state', request)
            query = GetEntityState.Request()
            query.name, query.reference_frame = name, 'world'
            actual = self.service(GetEntityState, '/gazebo/get_entity_state', query)
            observed = pose_values(actual.state.pose)
            if math.hypot(observed[0]-x, observed[1]-y) > .01:
                raise RuntimeError('door position mismatch: '+name)
            states.append({'leaf': name, 'mode': mode, 'pose': observed})
        (self.directory/f'doors_{self.floor}_{mode}_{time.time_ns()}.json').write_text(json.dumps(states, indent=2))

    def start_nav(self, point):
        for attempt in range(3):
            try:
                self._start_nav(point, attempt)
                return
            except RuntimeError as exc:
                if 'activation failed' not in str(exc) and 'initialization failed' not in str(exc):
                    raise
                self.stop()
                self.processes.stop_one(self.nav)
                self.result.setdefault('bootstrap_retries', []).append(str(exc))
                if attempt == 2:
                    raise
                self.node.pause(2)

    def _start_nav(self, point, attempt):
        if self.nav is not None and self.nav.poll() is None:
            raise RuntimeError('Nav2 already running')
        guard(self.floor, 'pre-nav', self.directory)
        self.node.amcl = None
        self.node.buffer.clear()
        map_yaml = MAPS/('f1/f1_manual_clean_v3.yaml' if self.floor == 'F1' else 'f2/f2_nav_unknown_v1.yaml')
        self.nav = self.processes.start('nav2_'+str(len(self.stages))+'_'+str(attempt), ['ros2', 'launch', 'nav2_bringup',
            'bringup_launch.py', f'map:={map_yaml}', f'params_file:={CONFIG/"nav2_params.yaml"}',
            'use_sim_time:=true', 'autostart:=false', 'use_composition:=False', 'use_respawn:=False'])
        lifecycle_start(self.node, 'localization')
        state = self.node.create_client(GetState, '/amcl/get_state')
        active = False
        for _ in range(40):
            if state.service_is_ready():
                future = state.call_async(GetState.Request())
                if self.node.wait(future.done, 2) and future.result().current_state.id == 3:
                    active = True
                    break
            self.node.pause(.5)
        self.node.destroy_client(state)
        if not active:
            raise RuntimeError('AMCL activation failed')
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.position.x, initial.pose.pose.position.y = point[:2]
        initial.pose.pose.orientation.z = math.sin(point[2]/2)
        initial.pose.pose.orientation.w = math.cos(point[2]/2)
        initial.pose.covariance[0] = initial.pose.covariance[7] = .25
        initial.pose.covariance[35] = math.radians(15)**2
        for _ in range(3):
            initial.header.stamp = self.node.get_clock().now().to_msg()
            self.node.initial.publish(initial)
            self.node.pause(.5)
        lifecycle_start(self.node, 'navigation')
        def ready_or_transport_failed():
            logfile = self.directory/('nav2_'+str(len(self.stages))+'_'+str(attempt)+'.log')
            return ((self.node.amcl is not None and self.node.client.server_is_ready()) or
                    ('failed to send response to' in logfile.read_text()))
        if not self.node.wait(ready_or_transport_failed, 90) or not (self.node.amcl is not None and self.node.client.server_is_ready()):
            raise RuntimeError('Nav2 initialization failed')
        self.node.pause(2)
        guard(self.floor, 'goal', self.directory)
        self.resume()

    def waypoint(self, name):
        point = self.waypoints[name]
        return point['x'], point['y'], point['yaw_rad']

    def autonomous(self, label, point):
        row = self.marker(label, 'Nav2')
        guard(self.floor, 'goal', self.directory)
        record_check(self.recorder, self.bag, self.node, self.directory)
        self.resume()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = point[:2]
        goal.pose.pose.orientation.z = math.sin(point[2]/2)
        goal.pose.pose.orientation.w = math.cos(point[2]/2)
        future = self.node.client.send_goal_async(goal)
        if not self.node.wait(future.done, 15) or not future.result().accepted:
            raise RuntimeError('goal rejected: '+label)
        handle = future.result()
        done = handle.get_result_async()
        deadline = self.node.get_clock().now().nanoseconds/1e9+self.goal_timeout
        self.node.wait(lambda: done.done() or self.node.get_clock().now().nanoseconds/1e9 > deadline,
                       self.goal_timeout*3)
        if not done.done():
            cancel = handle.cancel_goal_async()
            self.node.wait(cancel.done, 10)
            raise RuntimeError('goal timeout: '+label)
        row['action_status'] = done.result().status
        row['end_sim_s'] = self.node.get_clock().now().nanoseconds/1e9
        row['true_pose'] = pose_values(self.node.truth.pose.pose)
        if done.result().status != 4:
            raise RuntimeError('goal failed: '+label)
        self.stop()

    def manual(self, label, targets=(), payload=None):
        self.stop()
        if self.nav is not None:
            self.processes.stop_one(self.nav)
            if self.nav.poll() is None:
                raise RuntimeError('Nav2 still running before teleop')
        row = self.marker(label, 'manual WASD')
        row['nav2_process_stopped'] = True
        master, slave = pty.openpty()
        handle = open(self.directory/f'teleop_{len(self.stages)}.log', 'w')
        proc = subprocess.Popen([sys.executable, str(ROOT/'src/drive_pkg/drive_pkg/keyboard_teleop.py'),
            '--ros-args', '-p', 'max_linear_speed:=0.05', '-p', 'max_angular_speed:=0.20',
            '-p', 'use_sim_time:=true'], stdin=slave, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        os.close(slave)
        self.processes.items.append((proc, handle))
        self.node.pause(1)
        self.resume()
        key_counts = {}
        try:
            for tx,ty in targets:
                deadline = time.monotonic()+180
                while True:
                    if proc.poll() is not None:
                        raise RuntimeError('teleop exited unexpectedly')
                    x,y,yaw = pose_values(self.node.truth.pose.pose)
                    if math.hypot(tx-x,ty-y) < .06:
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError('manual target timeout')
                    if self.node.samples and self.node.samples[-1][4] <= .015:
                        raise RuntimeError('manual clearance <= 0.015m; stopped')
                    error = angle_error(math.atan2(ty-y,tx-x), yaw)
                    key = ('a' if error > 0 else 'd') if abs(error) > .09 else 'w'
                    os.write(master, key.encode())
                    self.key_pub.publish(String(data=json.dumps({'stage': label, 'key': key})))
                    key_counts[key] = key_counts.get(key,0)+1
                    self.node.pause(.12)
                os.write(master, b'k')
                self.node.pause(.6)
            os.write(master, b'k')
            self.node.pause(.7)
            self.stop()
            if payload:
                # Manual approach and withdrawal through the actual WASD node.
                self.resume()
                for key in ('w', 's'):
                    end = self.node.get_clock().now().nanoseconds/1e9+1.2
                    while self.node.get_clock().now().nanoseconds/1e9 < end:
                        os.write(master, key.encode())
                        self.key_pub.publish(String(data=json.dumps({'stage': label, 'key': key})))
                        key_counts[key] = key_counts.get(key, 0)+1
                        self.node.pause(.12)
                    os.write(master, b'k')
                    self.node.pause(.7)
                self.stop()
                self.payload_pub.publish(String(data=payload))
                row['payload_marker'] = payload
                row['arm_validated'] = False
            row['key_counts'] = key_counts
            row['true_pose'] = pose_values(self.node.truth.pose.pose)
            row['end_sim_s'] = self.node.get_clock().now().nanoseconds/1e9
        finally:
            os.write(master, b'k\x03')
            self.processes.stop_one(proc)
            os.close(master)
            self.stop()

    def travel(self, destination):
        self.stop()
        if self.nav is not None and self.nav.poll() is None:
            raise RuntimeError('Nav2 must be stopped during elevator travel')
        self.marker('elevator_'+self.floor+'_to_'+destination, 'manual floor selection (simulated travel)')
        self.door_state('closed')
        self.node.measure = False
        # Preserve cabin-relative orientation, with the different map headings.
        yaw = pose_values(self.node.truth.pose.pose)[2]
        source_inside = self.waypoint('f2_elevator_inside' if self.floor == 'F2' else 'f1_elevator_inside')
        destination_inside = self.waypoint('f2_elevator_inside' if destination == 'F2' else 'f1_elevator_inside')
        heading = destination_inside[2]-source_inside[2]
        cabin_dx = self.node.truth.pose.pose.position.x-source_inside[0]
        cabin_dy = self.node.truth.pose.pose.position.y-source_inside[1]
        for name in [self.floor.lower()+'_real_building', *self.doors[self.floor]['leaves']]:
            request = DeleteEntity.Request()
            request.name = name
            self.service(DeleteEntity, '/delete_entity', request)
        self.floor = destination
        self.node.amcl = None
        self.node.buffer.clear()
        suffix = '_door_open' if destination == 'F2' else ''
        request = SpawnEntity.Request()
        request.name = destination.lower()+'_real_building'
        request.xml = '<sdf version="1.6">'+(HERE/f'generated/{destination.lower()}{suffix}_building.xml').read_text()+'</sdf>'
        self.service(SpawnEntity, '/spawn_entity', request)
        target = self.waypoint('f2_elevator_inside' if destination == 'F2' else 'f1_elevator_inside')
        request = SetEntityState.Request()
        request.state.name, request.state.reference_frame = 'packagu_sim', 'world'
        request.state.pose.position.x = target[0]+math.cos(heading)*cabin_dx-math.sin(heading)*cabin_dy
        request.state.pose.position.y = target[1]+math.sin(heading)*cabin_dx+math.cos(heading)*cabin_dy
        request.state.pose.position.z = .005
        request.state.pose.orientation.z = math.sin((yaw+heading)/2)
        request.state.pose.orientation.w = math.cos((yaw+heading)/2)
        self.service(SetEntityState, '/gazebo/set_entity_state', request)
        self.node.pause(1)
        self.load_boxes()
        self.spawn_doors()
        self.node.measure = True

    def continue_from_locker(self):
        self.resume_recording()
        self._after_locker()

    def resume_recording(self):
        if not self.node.wait(lambda: self.node.truth is not None and self.node.scan is not None, 15):
            raise RuntimeError('existing simulation ground truth/scan unavailable')
        self.load_boxes()
        self.bag = self.directory/'bag'
        self.recorder = self.processes.start('record', ['ros2', 'bag', 'record', '--include-hidden-topics',
            '--max-cache-size', '262144', '--qos-profile-overrides-path',
            str(ROOT/'scripts/rosbag_qos_overrides.yaml'), '-o', str(self.bag), *TOPICS,
            '/sim/manual_stage', '/sim/manual_key', '/sim/payload_state'])
        record_check(self.recorder, self.bag, self.node, self.directory)
        self.node.measure = True
        self.result['continuation_from_user_hold'] = True
        self.result['starting_pose'] = pose_values(self.node.truth.pose.pose)
        self.result['initial_to_locker_autonomous_success'] = False

    def run(self):
        self.bootstrap()
        self.stop()
        print('INITIAL_POSITION_READY '+json.dumps(pose_values(self.node.truth.pose.pose)), flush=True)
        self.node.pause(self.start_delay)
        self.autonomous('initial_to_locker', self.waypoint('f1_locker'))
        self._after_locker()

    def _after_locker(self):
        self.manual('package_pickup', payload='loaded_by_operator')
        self.start_nav(pose_values(self.node.truth.pose.pose))
        self.autonomous('locker_to_F1_elevator_front', self.waypoint('f1_elevator_entry'))
        self.door_state('open')
        self.manual('F1_elevator_entry', [self.waypoint('f1_elevator_inside')[:2]])
        self.travel('F2')
        self.manual('F2_elevator_exit', [self.waypoint('f2_elevator_entry')[:2]])
        self.start_nav(pose_values(self.node.truth.pose.pose))
        self.autonomous('F2_to_delivery', self.waypoint('f2_delivery_destination'))
        self.manual('package_unload', payload='unloaded_by_operator')
        self.start_nav(pose_values(self.node.truth.pose.pose))
        self.autonomous('delivery_to_F2_elevator_front', self.waypoint('f2_elevator_entry'))
        self.door_state('open')
        self.manual('F2_elevator_entry', [self.waypoint('f2_elevator_inside')[:2]])
        self.travel('F1')
        self.manual('F1_elevator_exit', [self.waypoint('f1_elevator_entry')[:2]])
        self.start_nav(pose_values(self.node.truth.pose.pose))
        self.autonomous('F1_return_to_initial', IDLE)
        self.result['completed'] = True

    def finish(self):
        self.node.measure = False
        self.stop()
        if hasattr(self, 'recorder'):
            self.processes.stop_one(self.recorder)
            self.result['bag_finalized'] = (self.bag/'metadata.yaml').exists()
            analysis = subprocess.run([sys.executable, str(ROOT/'scripts/analyze_nav_bag.py'), str(self.bag)],
                                      capture_output=True, text=True, timeout=90)
            (self.directory/'analysis.log').write_text(analysis.stdout+analysis.stderr)
            self.result['analysis_exit'] = analysis.returncode
            contract = subprocess.run([sys.executable, str(ROOT/'scripts/bag_contract.py'), 'inspect', str(self.bag),
                '--require', '/scan', '--require', '/odom', '--require', '/tf', '--require', '/tf_static',
                '--require', '/sim/ground_truth', '--require', '/amcl_pose', '--require', '/plan',
                '--require', '/navigate_to_pose/_action/status', '--require', '/sim/manual_key',
                '--require', '/sim/manual_stage', '--require', '/sim/payload_state'],
                capture_output=True, text=True, timeout=90)
            (self.directory/'bag_contract.json').write_text(contract.stdout+contract.stderr)
            self.result['bag_contract_exit'] = contract.returncode
        self.result.update(self.node.events)
        self.result.update(self.node.stats)
        self.result['truth_samples'] = len(self.node.samples)
        self.result['min_wall_m'] = min((s[4] for s in self.node.samples), default=None)
        for index, stage in enumerate(self.stages):
            end = self.stages[index+1]['start_sim_s'] if index+1 < len(self.stages) else float('inf')
            samples = [s for s in self.node.samples if stage['start_sim_s'] <= s[0] < end]
            stage['min_wall_m'] = min((s[4] for s in samples), default=None)
            stage['collision_samples'] = sum(s[4] <= 0 for s in samples)
        self.result['navigation_sequence_completed'] = self.result.get('completed', False)
        self.result['completed'] = (self.result['navigation_sequence_completed'] and
                                    self.result.get('bag_finalized', False) and
                                    self.result.get('analysis_exit') == 0 and
                                    self.result.get('bag_contract_exit') == 0)
        with open(self.directory/'clearance.csv', 'w', newline='') as output:
            writer = csv.writer(output)
            writer.writerow(['sim_t','true_x','true_y','true_yaw','footprint_wall_m','amcl_x','amcl_y','amcl_yaw'])
            writer.writerows(self.node.samples)
        self.processes.cleanup()
        (self.directory/'result.json').write_text(json.dumps(self.result, indent=2)+'\n')
        self.node.client.destroy()
        self.node.listener.unregister()
        self.node.destroy_node()
        print(json.dumps(self.result), flush=True)


def main():
    from isolation import assert_isolated
    assert_isolated()
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', default='M1_manual_roundtrip_r1')
    parser.add_argument('--goal-timeout', type=float, default=900)
    parser.add_argument('--start-delay', type=float, default=10)
    parser.add_argument('--continue-from-locker', action='store_true')
    args = parser.parse_args()
    if Path(args.name).name != args.name:
        raise SystemExit('invalid mission name')
    rclpy.init()
    mission = Mission(args.name, args.goal_timeout, args.start_delay)
    try:
        if args.continue_from_locker:
            mission.continue_from_locker()
        else:
            mission.run()
    except Exception as exc:
        mission.result['completed'] = False
        mission.result['error'] = str(exc)
        print('MISSION_ERROR '+str(exc), flush=True)
    finally:
        mission.finish()
        rclpy.shutdown()
    raise SystemExit(0 if mission.result.get('completed') else 1)


if __name__ == '__main__':
    main()
