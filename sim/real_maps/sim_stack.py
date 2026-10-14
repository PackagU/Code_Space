#!/usr/bin/env python3
"""Isolated real-map simulation stack and shared measurement code.

Gazebo Classic (real-map world) + robot (URDF) + OpenCR shim + actual nav_safety_gate + Nav2
(slam_pkg kku_navigation.launch.py, use_sim_time) with the 9/15 ordered lifecycle STARTUP
(discovery wait -> localization -> initial pose -> navigation). Command chain:
  Nav2 /cmd_vel -> nav_safety_gate -> /cmd_vel_safe -> OpenCR shim -> /sim/cmd_vel_drive -> diff drive
Ground truth (/sim/ground_truth, p3d 20 Hz, world frame) is used for evaluation only, never by Nav2.

Run inside the isolated container (sim_container.sh). Persistent mode (used by start_sim.sh):
  python3 sim/real_maps/sim_stack.py --name NAME --world f1 --floor F1 --params P0 --spawn f1_initial_test
All results are simulation evidence only (not physical validation).
"""
import argparse
import collections
import csv
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Log
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener

from clearance import minimum_clearance
from isolation import assert_isolated
from run_scenarios import Processes, guard, record_check, lifecycle_start, pose_values

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LOG = ROOT/'logs/real_map_sim'
MAPS = ROOT/'maps/field'            # read-only mount of src/slam_pkg/maps (pins use /ros2_ws/maps)
CONFIG = ROOT/'src/slam_pkg/config'
NAV_PARAMS = {'P0': 'nav2_params_pre_wallpush_20260915.yaml', 'P1': 'nav2_params.yaml',
              'v011': 'nav2_params_speed_v011.yaml', 'v012': 'nav2_params_speed_v012.yaml'}
WORLD = yaml.safe_load((HERE/'world_params.yaml').read_text())
NAV_MAPS = {'F1': MAPS/'f1/f1_manual_clean_v3.yaml', 'F2': MAPS/'f2/f2_nav_unknown_v1.yaml'}
BOXES = {'f1': {'F1': 'f1_boxes.json'}, 'f2': {'F2': 'f2_boxes.json'},
         'f2_door_open': {'F2': 'f2_door_open_boxes.json'},
         'building': {'F1': 'f1_boxes.json', 'F2': 'f2_flat_boxes.json' if WORLD.get('f2_cleanup', {}).get('enabled') else 'f2_door_open_boxes.json'},
         'building_dressed': {'F1': 'f1_boxes.json', 'F2': 'f2_flat_boxes.json' if WORLD.get('f2_cleanup', {}).get('enabled') else 'f2_door_open_boxes.json'}}
TWO_FLOOR = ('building', 'building_dressed')   # building_dressed = same collisions + visual-only dressing
CORNER, CORNER_RADIUS = (-12.95, -5.25), 3.0   # F2 user-designated wall-proximity corner
NAV_NODES = ['map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']
TOPICS = ['/clock', '/scan', '/odom', '/tf', '/tf_static', '/sim/ground_truth', '/amcl_pose', '/plan',
          '/global_costmap/costmap', '/global_costmap/costmap_updates', '/global_costmap/costmap_raw',
          '/local_costmap/costmap', '/local_costmap/costmap_updates', '/local_costmap/costmap_raw',
          '/cmd_vel', '/cmd_vel_safe', '/sim/cmd_vel_drive', '/sim/shim_stats', '/rosout', '/drive/ready',
          '/nav_safety/status', '/nav_safety/stopped', '/navigate_to_pose/_action/status', '/initialpose',
          '/map', '/elevator/state', '/floor_orchestrator/status', '/sim/elevator/command',
          '/sim/mission_stage', '/sim/payload_state', '/joint_states', '/sim/parcel/state', '/set_arm_trajectory',
          '/set_lift_trajectory', '/sim/ultrasonic_front']
LOG_EVENTS = {'collision ahead': 'collision_ahead', 'starting point in lethal space': 'lethal_start',
              'control loop missed': 'controller_missed', 'running spin': 'recovery_spin',
              'running backup': 'recovery_backup', 'running wait': 'recovery_wait',
              'running drive_on_heading': 'recovery_drive_on_heading',
              'received request to clear entirely': 'costmap_clear', 'failed to create plan': 'planner_fail',
              'goal was rejected': 'goal_rejected'}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def sha12(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:12]


def waypoints():
    return json.loads((MAPS/'waypoints.json').read_text())['waypoints']


def resolve_pose(value):
    """Waypoint name -> (x, y, yaw, floor); or (x, y, yaw[, floor]) tuple passthrough."""
    if isinstance(value, str):
        p = waypoints()[value]
        return (p['x'], p['y'], p['yaw_rad'], p['floor'])
    return tuple(value)


def quat_yaw(yaw):
    return math.sin(yaw/2), math.cos(yaw/2)


def provenance(run_dir, files):
    """Run-start provenance: code commit, command, full SHA256 of every input actually used, and
    snapshots of the robot URDF / world_params so later edits never replace what this run used."""
    def git(*args):
        try:
            return subprocess.run(['git', '-C', str(ROOT), *args], capture_output=True, text=True, timeout=10).stdout.strip()
        except (OSError, subprocess.TimeoutExpired):
            return None
    snap = run_dir/'snapshot'
    snap.mkdir(exist_ok=True)
    record = {'started': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'git_head': git('rev-parse', 'HEAD'),
              'git_branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
              'git_dirty': [line for line in (git('status', '--porcelain', '--untracked-files=no') or '').splitlines()],
              'command': os.environ.get('PACKAGU_SIM_CMD') or ' '.join(sys.argv),
              'container': os.environ.get('HOSTNAME'), 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
              'ros_localhost_only': os.environ.get('ROS_LOCALHOST_ONLY'), 'files': {}}
    diff = git('diff', 'HEAD') or ''
    if diff:
        (snap/'uncommitted.diff').write_text(diff+'\n')
    record['git_diff_sha256'] = hashlib.sha256(diff.encode()).hexdigest() if diff else None
    for key, path in files.items():
        path = Path(path)
        record['files'][key] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        if key in ('robot_urdf', 'world_params', 'floor_maps_real', 'nav2_params'):
            shutil.copyfile(path, snap/path.name)
    (run_dir/'provenance.json').write_text(json.dumps(record, indent=2)+'\n')
    return record


class Monitor(Node):
    """Observer for one simulation: truth (map frame, floor offset removed), AMCL, scan, logs, metrics."""

    def __init__(self, world, name='real_map_sim_monitor'):
        super().__init__(name)
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.world = world
        # Floor offsets exist only in the two-floor world; single-floor worlds sit at the map origin.
        self.offsets = {f: (WORLD['floors'][f]['world_offset'] if world in TWO_FLOOR else [0.0, 0.0, 0.0])
                        for f in BOXES[world]}
        self.boxes = {f: np.array(json.loads((HERE/'generated'/n).read_text())) for f, n in BOXES[world].items()}
        self.doors = json.loads((HERE/'generated/doors.json').read_text()) if world in TWO_FLOOR else {}
        self.extents = {}
        for f in self.offsets:
            cfg = yaml.safe_load(NAV_MAPS[f].read_text())
            from PIL import Image
            w, h = Image.open(NAV_MAPS[f].parent/cfg['image']).size
            ox, oy, _ = cfg['origin']
            self.extents[f] = (ox, oy, ox+w*cfg['resolution'], oy+h*cfg['resolution'])
        self.truth = self.amcl = self.scan = self.odom = None
        self.truth_hist = collections.deque(maxlen=20*7200)
        self.amcl_hist = []
        self.scan_stamps = collections.deque(maxlen=400)
        self.cmd = (0.0, 0.0)
        self.elevator = self.orch = None
        self.drive_ready = self.safety_ready = None
        self.joints = {}
        self.parcel = None
        self.measure = False
        self.samples = []
        self.events = collections.Counter()
        self.stats = {'rpm_blocks': 0, 'ready_drops': 0}
        self.warnings = collections.deque(maxlen=400)
        self.progress_path = None
        self.last_progress = 0.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 20)
        self.create_subscription(Log, '/rosout', self.on_log, 200)
        self.create_subscription(String, '/sim/shim_stats', lambda m: setattr(self, 'stats', json.loads(m.data)), 10)
        self.create_subscription(Twist, '/cmd_vel_safe', lambda m: setattr(self, 'cmd', (m.linear.x, m.angular.z)), 10)
        self.create_subscription(String, '/elevator/state', lambda m: setattr(self, 'elevator', json.loads(m.data)), 10)
        self.create_subscription(String, '/floor_orchestrator/status',
                                 lambda m: setattr(self, 'orch', json.loads(m.data)), 10)
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.joints.update(dict(zip(m.name, m.position))), 20)
        self.create_subscription(String, '/sim/parcel/state', lambda m: setattr(self, 'parcel', json.loads(m.data)), 10)
        self.create_subscription(Bool, '/drive/ready', lambda m: setattr(self, 'drive_ready', m.data), 10)
        self.create_subscription(Bool, '/nav_safety/ready', lambda m: setattr(self, 'safety_ready', m.data), 10)
        self.initialpose_t = None
        self.initialpose_log = []
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', latched)
        self.stop_pub = self.create_publisher(Bool, '/nav_safety/stop', 10)
        self.elevator_cmd = self.create_publisher(String, '/sim/elevator/command', 10)
        self.stage_pub = self.create_publisher(String, '/sim/mission_stage', 10)
        self.payload_pub = self.create_publisher(String, '/sim/payload_state', 10)
        self.direct_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    # ------------------------------------------------------------ frames
    def floor_of(self, x, y):
        for f, off in self.offsets.items():
            x0, y0, x1, y1 = self.extents[f]
            if x0-2 <= x-off[0] <= x1+2 and y0-2 <= y-off[1] <= y1+2:
                return f
        return min(self.offsets, key=lambda f: abs(x-self.offsets[f][0]))

    def truth_map(self, msg=None):
        msg = msg or self.truth
        if msg is None:
            return None
        p = msg.pose.pose
        x, y, yaw = pose_values(p)
        f = self.floor_of(x, y)
        off = self.offsets[f]
        return f, x-off[0], y-off[1], yaw

    def leaf_boxes(self, floor):
        if not self.doors:
            return []
        d = self.doors[floor]
        state = self.elevator or {}
        if state.get('current_floor') == floor and state.get('leaves'):
            poses = state['leaves'].values()
        else:
            poses = [p['closed'] for p in d['leaves'].values()]
        return [[x, y, d['leaf_size_m'][0], d['leaf_size_m'][1], yaw] for x, y, yaw in poses]

    def clearance(self, floor, x, y, yaw):
        boxes = self.boxes[floor]
        leaves = self.leaf_boxes(floor)
        if leaves:
            boxes = np.vstack([boxes, np.array(leaves)])
        footprint = Stack.footprint((self.parcel or {}).get('state') == 'attached') if getattr(self, 'arm_model', False) else None
        return minimum_clearance(boxes, x, y, yaw, footprint)

    # ------------------------------------------------------------ callbacks
    def on_truth(self, msg):
        self.truth = msg
        t = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        f, x, y, yaw = self.truth_map(msg)
        self.truth_hist.append((t, f, x, y, yaw))
        if not self.measure:
            return
        clr = self.clearance(f, x, y, yaw)
        est = pose_values(self.amcl.pose.pose) if self.amcl else (None, None, None)
        self.samples.append([t, f, x, y, yaw, clr, *est, *self.cmd])
        if self.progress_path and t-self.last_progress >= 30:
            self.last_progress = t
            progress = dict(sim_t=t, floor=f, true_pose=[x, y, yaw], wall_m=clr,
                            min_wall_m=min(s[5] for s in self.samples), **self.events, **self.stats)
            self.progress_path.write_text(json.dumps(progress))
            print('PROGRESS '+self.progress_path.parent.name+' '+json.dumps(progress), flush=True)

    def on_amcl(self, msg):
        self.amcl = msg
        c = msg.pose.covariance
        t = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.amcl_hist.append((t, *pose_values(msg.pose.pose), c[0], c[7], c[35]))

    def on_initialpose(self, msg):
        self.initialpose_t = self.now()
        self.initialpose_log.append((self.initialpose_t, *pose_values(msg.pose.pose)))

    def on_scan(self, msg):
        self.scan = msg
        valid = sum(1 for r in msg.ranges if msg.range_min < r < msg.range_max)
        self.scan_stamps.append((msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, time.monotonic(),
                                 len(msg.ranges), valid/max(1, len(msg.ranges))))

    def on_log(self, msg):
        text = msg.msg.lower()
        for key, name in LOG_EVENTS.items():
            if key in text:
                self.events[name] += 1
        if msg.level >= 30:
            self.warnings.append(f'{msg.stamp.sec}.{msg.stamp.nanosec:09d} [{msg.name}] {msg.msg}')

    # ------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def wait(self, predicate, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=.05)
        return bool(predicate())

    def pause(self, seconds):
        self.wait(lambda: False, seconds)

    def sim_pause(self, seconds, wall_cap=None):
        end = self.now()+seconds
        self.wait(lambda: self.now() >= end, wall_cap or seconds*20+10)

    def truth_at(self, t, floor=None):
        """Interpolated map-frame truth at sim time t (None across gaps > 0.15 s or floor changes)."""
        hist = self.truth_hist
        if not hist or t < hist[0][0] or t > hist[-1][0]:
            return None
        lo, hi = 0, len(hist)-1
        while hi-lo > 1:
            mid = (lo+hi)//2
            if hist[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = hist[lo], hist[hi]
        if b[0]-a[0] > .15 or a[1] != b[1] or (floor and a[1] != floor):
            return None
        r = 0 if b[0] == a[0] else (t-a[0])/(b[0]-a[0])
        return a[2]+r*(b[2]-a[2]), a[3]+r*(b[3]-a[3]), wrap(a[4]+r*wrap(b[4]-a[4]))

    def amcl_errors(self, since=None, floor=None):
        rows = []
        for t, x, y, yaw, cxx, cyy, cyaw in self.amcl_hist:
            if since is not None and t < since:
                continue
            truth = self.truth_at(t, floor)
            if truth is None:
                continue
            rows.append({'t': t, 'pos_err_m': math.hypot(x-truth[0], y-truth[1]),
                         'yaw_err_rad': abs(wrap(yaw-truth[2])), 'cov_xx': cxx, 'cov_yy': cyy, 'cov_yaw': cyaw})
        return rows

    def lifecycle_state(self, node_name, timeout=3.0):
        client = self.create_client(GetState, f'/{node_name}/get_state')
        try:
            if not self.wait(client.service_is_ready, timeout):
                return 'unavailable'
            future = client.call_async(GetState.Request())
            if not self.wait(future.done, timeout) or future.result() is None:
                return 'timeout'
            return future.result().current_state.label
        finally:
            self.destroy_client(client)

    def call_empty(self, service, timeout=5.0):
        client = self.create_client(Empty, service)
        try:
            if not self.wait(client.service_is_ready, timeout):
                return False
            future = client.call_async(Empty.Request())
            return self.wait(future.done, timeout)
        finally:
            self.destroy_client(client)

    def tf_pose(self, parent, child):
        try:
            t = self.buffer.lookup_transform(parent, child, Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


def segment_metrics(samples, goal=None, floor_corner=True):
    """Clearance + naturalness metrics over truth samples of one segment (single floor)."""
    out = {'truth_samples': len(samples)}
    if len(samples) < 2:
        return out
    arr = np.array([[s[0], s[2], s[3], s[4], s[5]] for s in samples], dtype=float)
    t, x, y, yaw, clr = arr.T
    out['min_wall_m'] = float(clr.min())
    floor = samples[0][1]
    if floor_corner and floor == 'F2':
        near = np.hypot(x-CORNER[0], y-CORNER[1]) <= CORNER_RADIUS
        out['corner_min_wall_m'] = float(clr[near].min()) if near.any() else None
    dt = np.diff(t)
    dt[dt <= 0] = 1e-3
    step = np.hypot(np.diff(x), np.diff(y))
    dyaw = np.array([wrap(v) for v in np.diff(yaw)])
    out['path_length_m'] = float(step.sum())
    straight = math.hypot(x[-1]-x[0], y[-1]-y[0])
    out['straight_distance_m'] = straight
    out['path_ratio'] = float(step.sum()/straight) if straight > .05 else None
    k = 10   # 0.5 s at 20 Hz
    kernel = np.ones(k)/k
    speed = np.convolve(step/dt, kernel, mode='same')
    rate = np.convolve(dyaw/dt, kernel, mode='same')
    rotating = (speed < .02) & (np.abs(rate) > .15)
    runs, angle, start = 0, 0.0, None
    for i, flag in enumerate(np.r_[rotating, False]):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if t[min(i, len(t)-1)]-t[start] >= .5:
                runs += 1
                angle += float(np.abs(dyaw[start:i]).sum())
            start = None
    out['in_place_rotations'] = runs
    out['in_place_rotation_deg'] = math.degrees(angle)
    signs = np.sign(rate[np.abs(rate) > .05])
    out['angular_sign_changes'] = int((np.diff(signs) != 0).sum()) if len(signs) > 1 else 0
    stopped = speed < .01
    restarts, start = 0, None
    for i, flag in enumerate(stopped):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if t[i]-t[start] >= 1.0 and start > 0 and speed[i:i+10].max(initial=0) > .03:
                restarts += 1
            start = None
    out['stop_restarts'] = restarts
    if goal is not None:
        out['arrival_pos_err_m'] = math.hypot(x[-1]-goal[0], y[-1]-goal[1])
        out['arrival_yaw_err_rad'] = abs(wrap(yaw[-1]-goal[2]))
    return out


class Stack:
    """One simulation run directory; owns or attaches to the simulation processes."""

    def __init__(self, name, world='f1', floor='F1', params='P0', spawn='f1_initial_test', initial=None,
                 lidar_noise=True, odom='encoder', record=True, attach=False, arm=False, speed=1.0, parcels=1,
                 gui=False, load='pallet', lift_stop=None, initial_elevator_floor=None, initial_load=False):
        if Path(name).name != name or not name:
            raise SystemExit('invalid run name')
        self.name, self.world, self.floor, self.params = name, world, floor, params
        self.nav_maps = dict(NAV_MAPS)
        self.floor_registry = HERE/'floor_maps_real.yaml'
        if world in TWO_FLOOR and WORLD.get('f2_cleanup', {}).get('enabled'):
            self.nav_maps['F2'] = HERE/'generated/f2_sim_flat.yaml'
            self.floor_registry = HERE/'generated/floor_maps_sim_flat.yaml'
        self.dir = LOG/name
        if not attach and (self.dir/'bag').exists():
            raise SystemExit(f'existing bag in {self.dir}; choose a new --name (runs are never overwritten)')
        self.dir.mkdir(parents=True, exist_ok=True)
        from sim_map_guard import pin_map
        self.sim_map_pins = {f: pin_map(path) for f, path in self.nav_maps.items() if path != NAV_MAPS[f]}
        if self.sim_map_pins:
            (self.dir/'sim_map_pins.json').write_text(json.dumps(self.sim_map_pins, indent=2))
        self.spawn = resolve_pose(spawn)
        self.initial = resolve_pose(initial) if initial is not None else self.spawn
        if arm:
            if odom != 'encoder' or not lidar_noise:
                raise SystemExit('--arm-sim uses robot_arm.urdf (encoder odom, LiDAR noise on) only')
            # The arm-tip camera needs a display to render (no Xvfb in the image): GUI runs and --load pallet runs
            # (camera-based pallet/button skills) use the X11-enabled container and robot_arm_cam.urdf.
            self.robot = HERE/'generated'/('robot_arm_cam.urdf' if gui or load == 'pallet' else 'robot_arm.urdf')
        elif odom == 'world':
            self.robot = HERE/'generated/robot_world.urdf'
        else:
            self.robot = HERE/'generated'/('robot.urdf' if lidar_noise else 'robot_nonoise.urdf')
        self.record, self.odom_profile, self.lidar_noise, self.arm = record, odom, lidar_noise, arm
        self.speed, self.parcels, self.gui, self.load = float(speed), int(parcels), gui, load
        self.initial_elevator_floor, self.initial_load = initial_elevator_floor, initial_load
        self.lift_stop = lift_stop or WORLD['lift']['carry']['mode']
        self.node = Monitor(world, 'real_map_sim_attach' if attach else 'real_map_sim_monitor')
        self.node.arm_model = arm
        self.node.progress_path = self.dir/'progress.json'
        self.nav_params_file = CONFIG/NAV_PARAMS[self.params]
        if arm:
            self.nav_params_file = self.dir/'nav2_sim_payload.yaml'
            profile = yaml.safe_load((CONFIG/NAV_PARAMS[self.params]).read_text())
            for costmap in ('local_costmap', 'global_costmap'):
                profile[costmap][costmap]['ros__parameters']['footprint'] = json.dumps(self.footprint(False))
                profile[costmap][costmap]['ros__parameters']['footprint_padding'] = .15
            self.nav_params_file.write_text(yaml.safe_dump(profile))
        self.processes = Processes(self.dir)
        self.nav = self.recorder = None
        self.bag = self.dir/'bag'
        self.result = {'name': name, 'validation_scope': 'simulation', 'world': world, 'floor': floor,
                       'params': params, 'odometry': odom, 'lidar_noise': lidar_noise, 'arm_sim': arm,
                       'sim_speed_target': 'max' if float(speed) == 0 else float(speed), 'parcels': int(parcels) if arm else 0,
                       'load': load if arm else None,
                       'lift_stop': (lift_stop or WORLD['lift']['carry']['mode']) if arm and load == 'pallet' else None,
                       'dressed_visual_only': world == 'building_dressed',
                       'provisional_dimensions': True, 'error': None}

    @staticmethod
    def footprint(loaded):
        fp = WORLD['robot']['footprint']
        rear = min(p[0] for p in fp)
        half = max(abs(p[1]) for p in fp)
        front = max(max(p[0] for p in fp), float(WORLD['lift']['fork_tip_x_m']))
        if loaded:
            from dressing import pallet_center_ahead
            pal = WORLD['parcel']['pallet']
            front = max(front, pallet_center_ahead(pal)+pal['size_m'][0]/2)
            half = max(half,pal['size_m'][1]/2)
        return [[front,half],[front,-half],[rear,-half],[rear,half]]

    def update_footprint(self, loaded):
        if not self.arm or getattr(self, '_footprint_loaded', None) == bool(loaded):
            return
        value = json.dumps(self.footprint(loaded))
        parameter = Parameter('footprint', value=value).to_parameter_msg()
        for node_name in ('local_costmap/local_costmap', 'global_costmap/global_costmap'):
            client = self.node.create_client(SetParameters, f'/{node_name}/set_parameters')
            try:
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError(f'{node_name} footprint service unavailable')
                future = client.call_async(SetParameters.Request(parameters=[parameter]))
                if not self.node.wait(future.done, 10):
                    raise RuntimeError(f'{node_name} footprint update timeout')
                result = future.result()
                if result is None or not all(r.successful for r in result.results):
                    raise RuntimeError(f'{node_name} footprint rejected')
            finally:
                self.node.destroy_client(client)
        self._footprint_loaded = bool(loaded)
        self.node.pause(.5)
        self.result.setdefault('payload_footprints',[]).append({'loaded': bool(loaded), 'outline': self.footprint(loaded)})

    @classmethod
    def attach(cls, name):
        ready = json.loads((LOG/name/'ready.json').read_text())
        stack = cls(name, ready['world'], ready['floor'], ready['params'], tuple(ready['spawn']),
                    attach=True, record=False)
        stack.attached = True
        return stack

    # ------------------------------------------------------------ bring-up
    def guard_map(self, stage):
        if self.floor in self.sim_map_pins:
            from sim_map_guard import guard as generated_guard
            generated_guard(self.sim_map_pins[self.floor], stage, self.dir)
        else:
            guard(self.floor, stage, self.dir)

    def config_hashes(self):
        files = {'nav2_params': self.nav_params_file, 'nav_safety': ROOT/'src/drive_pkg/config/nav_safety.yaml',
                 'drive_calib': ROOT/'src/drive_pkg/config/drive_calib.yaml', 'waypoints': MAPS/'waypoints.json',
                 'map_pins': MAPS/'map_pins.json', 'world_params': HERE/'world_params.yaml', 'robot_urdf': self.robot}
        files.update({f'map_{f}_yaml': self.nav_maps[f] for f in self.node.offsets})
        for f in self.node.offsets:
            image = yaml.safe_load(self.nav_maps[f].read_text())['image']
            files[f'map_{f}_pgm'] = self.nav_maps[f].parent/image
        files['world'] = HERE/f'generated/{self.world}.world'
        if self.world in TWO_FLOOR:
            files['floor_maps_real'] = self.floor_registry
        self.config_files = files
        return {k: sha12(v) for k, v in files.items()}

    def start(self):
        assert_isolated()
        node = self.node
        self.result['config_sha256_12'] = self.config_hashes()
        self.result['provenance'] = provenance(self.dir, self.config_files)
        self.guard_map('pre-nav')
        self.processes.start('gazebo', ['gzserver', '--verbose', str(HERE/f'generated/{self.world}.world'),
                                        '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'])
        rsp = self.dir/'rsp.yaml'
        rsp.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
            'use_sim_time': True, 'robot_description': self.robot.read_text()}}}))
        self.processes.start('rsp', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                                     '--ros-args', '--params-file', str(rsp)])
        x, y, yaw = self.spawn[:3]
        off = node.offsets[self.floor]
        spawn = self.processes.start('spawn', ['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'packagu_sim',
            '-file', str(self.robot), '-x', str(x+off[0]), '-y', str(y+off[1]), '-z', str(off[2]+.005),
            '-Y', str(yaw), '-timeout', '90'])
        node.wait(lambda: (node.truth is not None and node.scan is not None) or spawn.poll() not in (None, 0), 120)
        if spawn.poll() not in (None, 0):
            raise RuntimeError('robot spawn failed; see spawn.log')
        if self.gui:
            self.start_gui()
        if node.truth is None or node.scan is None:
            raise RuntimeError('GROUND_TRUTH_OR_SCAN_UNAVAILABLE: user confirmation required')
        node.pause(2)
        self.result['truth_spawn_pose'] = node.truth_map()
        self.processes.start('bridge', [sys.executable, str(HERE/'opencr_shim.py'), '--ros-args',
            '--params-file', str(ROOT/'src/drive_pkg/config/drive_calib.yaml'),
            '-p', 'cmd_vel_topic:=/cmd_vel_safe', '-p', 'publish_tf:=false', '-p', 'use_sim_time:=true'])
        gate = [sys.executable, str(ROOT/'src/drive_pkg/drive_pkg/nav_safety_gate.py'), '--ros-args',
                '--params-file', str(ROOT/'src/drive_pkg/config/nav_safety.yaml'), '-p', 'use_sim_time:=true']
        if self.params == 'v012':
            gate += ['-p', 'max_linear_speed:=0.13']
        self.processes.start('gate', gate)
        if self.world in TWO_FLOOR:
            self.elevator_process = self.processes.start('elevator', [sys.executable, str(HERE/'elevator_sim_node.py'),
                                              '--ros-args', '-p', 'use_sim_time:=true',
                                              '-p', f"indicators:={'true' if self.world == 'building_dressed' else 'false'}",
                                              '-p', f"button_contact:={'true' if self.arm and self.load == 'pallet' else 'false'}",
                                              '-p', f"require_arm_pause:={'true' if self.arm else 'false'}",
                                              '-p', f'initial_floor:={self.initial_elevator_floor or WORLD["elevator"]["initial_floor"]}'])
            self.processes.start('floor_reader', [sys.executable, str(HERE/'fake_floor_reader.py'),
                                                  '--ros-args', '-p', 'use_sim_time:=true'])
            self.processes.start('orchestrator', ['ros2', 'run', 'auto_floor_orchestrator_pkg',
                'auto_floor_orchestrator_node', '--ros-args', '-p', 'use_sim_time:=true',
                '-p', f'floor_maps_yaml:={self.floor_registry}', '-p', 'dry_run_map_load:=false',
                '-p', f'current_floor:={self.floor}', '-p', 'target_floor:=F2'])
            if self.arm:
                self.processes.start('parcel', [sys.executable, str(HERE/'parcel_mock.py'),
                                                '--ros-args', '-p', 'use_sim_time:=true',
                                                '-p', f'stack_count:={self.parcels}', '-p', f'carry_mode:={self.load}', '-p', f'initial_attached:={str(self.initial_load).lower()}'])
        self.start_nav()
        if self.speed != 1.0:
            self.set_speed(self.speed)
        self.result['readiness'] = self.readiness()
        if not self.result['readiness']['ready']:
            raise RuntimeError('readiness check failed: '+json.dumps(self.result['readiness']['failed']))
        if self.gui:
            self.follow_camera()
        return self.result['readiness']

    def start_gui(self):
        """gzclient + RViz in this (GUI-enabled) container; observation only."""
        self.processes.start('gzclient', ['gzclient'])
        self.processes.start('rviz', ['rviz2', '-d', str(HERE/'nav2_view.rviz'), '--ros-args', '-p', 'use_sim_time:=true'])
        if self.arm:
            self.processes.start('arm_camera_view', ['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                                                     '/arm_camera/image_raw'])

    def follow_camera(self):
        for _ in range(10):
            done = subprocess.run(['gz', 'camera', '-c', 'gzclient_camera', '-f', 'packagu_sim'],
                                  capture_output=True, text=True, timeout=20)
            if done.returncode == 0:
                return
            self.node.pause(2)

    def set_speed(self, target):
        """[시연용] Target real-time factor. Only real_time_update_rate changes; max_step_size stays 0.001 s, so the
        physics per simulated second is unchanged. Goal/elevator timeouts are in sim seconds already."""
        if target == 0.0:
            subprocess.run(['gz', 'physics', '-u', '0'], check=True, capture_output=True, timeout=20)
            sim0, wall0 = self.node.now(), time.monotonic()
            self.node.pause(2)
            self.result['sim_speed'] = {'target': 'max', 'max_step_size_s': .001,
                                       'update_rate_hz': 0, 'measured_rtf': (self.node.now()-sim0)/(time.monotonic()-wall0)}
            return
        rate, tuning = 1000.0*target*1.4, []
        for _ in range(5):
            subprocess.run(['gz', 'physics', '-u', str(int(rate))], capture_output=True, timeout=20)
            sim0, wall0 = self.node.now(), time.monotonic()
            self.node.pause(4)
            rtf = (self.node.now()-sim0)/(time.monotonic()-wall0)
            tuning.append({'update_rate_hz': int(rate), 'rtf': round(rtf, 3)})
            if abs(rtf-target) <= .08*target:
                break
            rate = min(rate*target/max(rtf, .05), 1000.0*target*3)
        self.result['sim_speed'] = {'target': target, 'max_step_size_s': 0.001, 'tuning': tuning}

    def start_recorder(self):
        if not self.record or self.recorder is not None:
            return
        self.recorder = self.processes.start('record', ['ros2', 'bag', 'record', '--include-hidden-topics',
            '--max-cache-size', '262144', '--qos-profile-overrides-path',
            str(ROOT/'scripts/rosbag_qos_overrides.yaml'), '-o', str(self.bag), *TOPICS])
        record_check(self.recorder, self.bag, self.node, self.dir)

    def start_nav(self):
        for attempt in range(3):
            try:
                self._start_nav(attempt)
                return
            except RuntimeError as exc:
                transient = any(k in str(exc) for k in ('activation failed', 'initialization failed', 'unavailable'))
                self.result.setdefault('bootstrap_retries', []).append(str(exc))
                if not transient or attempt == 2:
                    raise
                self.stop()
                self.processes.stop_one(self.nav)
                self.node.pause(3)

    def _start_nav(self, attempt):
        node = self.node
        node.amcl = None
        self.nav = self.processes.start(f'nav2_{attempt}', ['ros2', 'launch', 'slam_pkg', 'kku_navigation.launch.py',
            f'map:={self.nav_maps[self.floor]}', f'params_file:={self.nav_params_file}',
            'use_sim_time:=true', 'autostart:=false', 'rviz:=false'])
        self.start_recorder()
        lifecycle_start(node, 'localization')
        active = False
        for _ in range(40):
            if node.lifecycle_state('amcl', 2) == 'active':
                active = True
                break
            node.pause(.5)
        if not active:
            raise RuntimeError('AMCL lifecycle activation failed')
        self.publish_initial(self.initial)
        if not node.wait(lambda: node.amcl is not None and node.buffer.can_transform('map', 'base_footprint', Time()), 30):
            raise RuntimeError('AMCL/TF initialization failed')
        lifecycle_start(node, 'navigation')
        if not node.wait(lambda: node.client.server_is_ready(), 90):
            raise RuntimeError('Nav2 action server unavailable (initialization failed)')
        node.pause(2)
        self.guard_map('goal')

    def publish_initial(self, pose, repeats=3, xy_var=.25, yaw_sd=math.radians(15)):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = quat_yaw(pose[2])
        # Same operator uncertainty as field_nav_cli pose-set (9/15 runner).
        msg.pose.covariance[0] = msg.pose.covariance[7] = xy_var
        msg.pose.covariance[35] = yaw_sd**2
        for _ in range(repeats):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.node.initial.publish(msg)
            self.node.pause(.5)

    def readiness(self):
        node = self.node
        report, failed = {}, []
        n0 = len(node.scan_stamps)
        sim0, wall0 = node.now(), time.monotonic()
        node.pause(6)
        stamps = list(node.scan_stamps)[n0:]
        sim1, wall1 = node.now(), time.monotonic()
        report['rtf'] = (sim1-sim0)/(wall1-wall0)
        if len(stamps) >= 2 and stamps[-1][0] > stamps[0][0]:
            report['scan_rate_hz_sim'] = (len(stamps)-1)/(stamps[-1][0]-stamps[0][0])
            report['scan_points'] = stamps[-1][2]
        else:
            report['scan_rate_hz_sim'] = 0.0
        if not 6.5 <= report['scan_rate_hz_sim'] <= 8.7:
            failed.append('scan_rate')
        # A robot spawned outside the walls sees nothing (all inf) yet still drives; refuse that.
        report['scan_valid_fraction'] = min((s[3] for s in stamps), default=0.0)
        if report['scan_valid_fraction'] < .2:
            failed.append('scan_valid_fraction')
        for parent, child in (('map', 'odom'), ('odom', 'base_footprint'), ('map', 'base_footprint')):
            ok = node.buffer.can_transform(parent, child, Time())
            report[f'tf_{parent}_to_{child}'] = ok
            if not ok:
                failed.append(f'tf_{parent}_{child}')
        truth = node.truth_map()
        if node.amcl is not None and truth is not None:
            ax, ay, ayaw = pose_values(node.amcl.pose.pose)
            report['amcl_pos_err_m'] = math.hypot(ax-truth[1], ay-truth[2])
            report['amcl_yaw_err_rad'] = abs(wrap(ayaw-truth[3]))
        else:
            failed.append('amcl_pose')
        report['lifecycle'] = {n: node.lifecycle_state(n) for n in NAV_NODES}
        failed += [f'lifecycle_{n}' for n, s in report['lifecycle'].items() if s != 'active']
        report['drive_ready'] = node.drive_ready
        report['nav_safety_ready'] = node.safety_ready
        if node.drive_ready is not True:
            failed.append('drive_ready')
        report['failed'] = failed
        report['ready'] = not failed
        report['truth_map_pose'] = truth
        return report

    # ------------------------------------------------------------ motion
    def stop(self):
        for _ in range(3):
            self.node.stop_pub.publish(Bool(data=True))
            self.node.pause(.15)

    def resume(self):
        self.node.stop_pub.publish(Bool(data=False))
        self.node.pause(.3)

    def navigate(self, point, timeout):
        """Send one NavigateToPose goal in the current floor's map frame. Returns final status string."""
        node = self.node
        self.guard_map('goal')
        self.resume()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = point[:2]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quat_yaw(point[2])
        future = node.client.send_goal_async(goal)
        if not node.wait(future.done, 15):
            return 'NOT_ACCEPTED_TIMEOUT'
        handle = future.result()
        if not handle.accepted:
            return 'REJECTED'
        done = handle.get_result_async()
        deadline = node.now()+timeout
        node.wait(lambda: done.done() or node.now() > deadline, timeout*20+60)
        if not done.done():
            cancel = handle.cancel_goal_async()
            node.wait(cancel.done, 10)
            node.wait(done.done, 10)
            self.stop()
            return 'TIMEOUT'
        return {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(done.result().status, str(done.result().status))

    def begin(self, label):
        node = self.node
        node.measure = True
        return {'label': label, 'sample_index': len(node.samples), 'sim_start': node.now(),
                'wall_start': time.monotonic(), 'events': dict(node.events), 'stats': dict(node.stats)}

    def end(self, seg, status=None, goal=None):
        node = self.node
        samples = node.samples[seg['sample_index']:]
        row = {'stage': seg['label'], 'status': status, 'floor': samples[-1][1] if samples else self.floor}
        row['sim_s'] = node.now()-seg['sim_start']
        row['wall_s'] = time.monotonic()-seg['wall_start']
        row['rtf'] = row['sim_s']/row['wall_s'] if row['wall_s'] > 0 else None
        row.update(segment_metrics(samples, goal))
        for key in set(LOG_EVENTS.values()):
            row[key] = node.events.get(key, 0)-seg['events'].get(key, 0)
        row['recoveries'] = sum(row[k] for k in row if k.startswith('recovery_'))
        row['rpm_blocks'] = node.stats.get('rpm_blocks', 0)-seg['stats'].get('rpm_blocks', 0)
        row['ready_drops'] = node.stats.get('ready_drops', 0)-seg['stats'].get('ready_drops', 0)
        truth = node.truth_map()
        row['true_pose'] = list(truth) if truth else None
        row['amcl_pose'] = list(pose_values(node.amcl.pose.pose)) if node.amcl else None
        return row

    # ------------------------------------------------------------ bag validity
    def required_topics(self):
        topics = ['/scan', '/odom', '/tf', '/tf_static', '/sim/ground_truth', '/amcl_pose', '/plan',
                  '/cmd_vel_safe', '/rosout', '/navigate_to_pose/_action/status']
        if self.world in TWO_FLOOR:
            topics += ['/elevator/state', '/floor_orchestrator/status', '/map']
        if self.arm:
            topics += ['/joint_states', '/sim/parcel/state']
        return topics

    def check_bag(self):
        """metadata.yaml after a normal recorder stop; sim-only recovery (ros2 bag reindex) is recorded
        separately from the original failure. Then the bag contract with the required topics."""
        out = {'path': str(self.bag), 'original_metadata_present': (self.bag/'metadata.yaml').exists()}
        if not out['original_metadata_present']:
            end = time.monotonic()+20
            while not (self.bag/'metadata.yaml').exists() and time.monotonic() < end:
                time.sleep(.5)
            out['metadata_after_wait'] = (self.bag/'metadata.yaml').exists()
            if not out['metadata_after_wait'] and list(self.bag.glob('*.db3')):
                rec = subprocess.run(['ros2', 'bag', 'reindex', '-s', 'sqlite3', str(self.bag)], capture_output=True,
                                     text=True, timeout=300)
                (self.dir/'bag_reindex.log').write_text(rec.stdout+rec.stderr)
                out['recovery'] = {'method': 'ros2 bag reindex (simulation-only recovery)', 'exit': rec.returncode,
                                   'metadata_present': (self.bag/'metadata.yaml').exists()}
        required = self.required_topics()
        if (self.bag/'metadata.yaml').exists():
            args = [sys.executable, str(ROOT/'scripts/bag_contract.py'), 'inspect', str(self.bag)]
            for topic in required:
                args += ['--require', topic]
            contract = subprocess.run(args, capture_output=True, text=True, timeout=180)
            (self.dir/'bag_contract.json').write_text(contract.stdout+contract.stderr)
            out['contract_exit'] = contract.returncode
            out['required_topics'] = required
        if out['original_metadata_present'] or out.get('metadata_after_wait'):
            out['status'] = 'OK' if out.get('contract_exit') == 0 else 'RECORD_FAILED_CONTRACT'
        elif out.get('recovery', {}).get('metadata_present'):
            out['status'] = ('RECORD_FAILED_METADATA_RECOVERED' if out.get('contract_exit') == 0
                             else 'RECORD_FAILED_METADATA_RECOVERED_CONTRACT_FAILED')
        else:
            out['status'] = 'RECORD_FAILED_NO_METADATA'
        return out

    # ------------------------------------------------------------ teardown
    def finish(self, preserve_scene=False):
        node = self.node
        node.measure = False
        if getattr(self, 'attached', False):
            node.destroy_node()
            return self.result
        try:
            self.stop()
        except Exception:
            pass
        if preserve_scene:
            # Freeze physics only after the emergency stop; retain the failed pose for inspection.
            preserve_scene = bool(node.call_empty('/pause_physics'))
            self.result['scene_preserved'] = preserve_scene
            if preserve_scene:
                (self.dir/'SCENE_PRESERVED').write_text('Stopped robot; Gazebo physics paused. Close with sim_container.sh down.\n')
        if self.recorder is not None:
            self.processes.stop_one(self.recorder)
            self.result['bag'] = self.check_bag()
            self.result['bag_finalized'] = self.result['bag']['original_metadata_present']
            analysis = subprocess.run([sys.executable, str(ROOT/'scripts/analyze_nav_bag.py'), str(self.bag)],
                                      capture_output=True, text=True, timeout=180)
            (self.dir/'analysis.log').write_text(analysis.stdout+analysis.stderr)
            self.result['analysis_exit'] = analysis.returncode
            self.result['bag_contract_exit'] = self.result['bag'].get('contract_exit')
        with open(self.dir/'clearance.csv', 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['sim_t', 'floor', 'true_x', 'true_y', 'true_yaw', 'footprint_wall_m',
                             'amcl_x', 'amcl_y', 'amcl_yaw', 'cmd_safe_v', 'cmd_safe_w'])
            writer.writerows(node.samples)
        with open(self.dir/'amcl.csv', 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['sim_t', 'x', 'y', 'yaw', 'cov_xx', 'cov_yy', 'cov_yaw'])
            writer.writerows(node.amcl_hist)
        (self.dir/'rosout_warnings.txt').write_text('\n'.join(node.warnings)+'\n')
        if not preserve_scene:
            self.processes.cleanup()
        (self.dir/'result.json').write_text(json.dumps(self.result, indent=2, default=str)+'\n')
        node.client.destroy()
        node.listener.unregister()
        node.destroy_node()
        return self.result


def parse_speed(value):
    if str(value).lower() == 'max':
        return 0.0  # Gazebo unlimited real-time update rate; physics step unchanged.
    speed = float(value)
    if not math.isfinite(speed) or speed <= 0:
        raise argparse.ArgumentTypeError('--speed must be a positive factor or max')
    return speed


def add_common_args(parser):
    parser.add_argument('--params', default='P0', choices=sorted(NAV_PARAMS))
    parser.add_argument('--lidar-noise', default='on', choices=['on', 'off'])
    parser.add_argument('--odom', default='encoder', choices=['encoder', 'world'],
                        help='encoder (default, new runs) or world (9/15 reproduction only)')
    parser.add_argument('--speed', type=parse_speed, default=1.0,
                        help='[시연용] target real-time factor; physics step stays 1 ms (runs != 1 are kept apart '
                             'from the 1x statistics)')


def main():
    parser = argparse.ArgumentParser(description='persistent isolated real-map simulation stack')
    parser.add_argument('--name', required=True)
    parser.add_argument('--world', default=None, choices=sorted(BOXES))
    parser.add_argument('--floor', default='F1', choices=['F1', 'F2'])
    parser.add_argument('--spawn', default='f1_initial_test', help='waypoint name')
    parser.add_argument('--initial', default=None, help='AMCL initial pose waypoint (default = spawn)')
    parser.add_argument('--no-record', action='store_true')
    parser.add_argument('--arm-sim', action='store_true', help='robot_arm.urdf + parcel mock (building world)')
    parser.add_argument('--parcels', type=int, default=1, help='--arm-sim: parcels on the pallet / boxes stacked (1..3)')
    parser.add_argument('--load', default='pallet', choices=['pallet', 'box'],
                        help='--arm-sim: floor pallet picked by the lift carrier (default) or 10/8 shelf box')
    add_common_args(parser)
    args = parser.parse_args()
    world = args.world or args.floor.lower()
    spawn = resolve_pose(args.spawn)
    if spawn[3] != args.floor:
        raise SystemExit(f'spawn {args.spawn} is on {spawn[3]}, not {args.floor}')
    rclpy.init()
    stack = Stack(args.name, world, args.floor, args.params, args.spawn, args.initial,
                  args.lidar_noise == 'on', args.odom, not args.no_record, arm=args.arm_sim, speed=args.speed,
                  parcels=args.parcels, load=args.load)
    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(1))
    code = 0
    try:
        readiness = stack.start()
        ready = {'name': args.name, 'world': world, 'floor': args.floor, 'params': args.params,
                 'spawn': list(spawn), 'odom': args.odom, 'lidar_noise': args.lidar_noise, **readiness,
                 'config_sha256_12': stack.result['config_sha256_12']}
        (stack.dir/'ready.json').write_text(json.dumps(ready, indent=2, default=str)+'\n')
        print('READY '+json.dumps(ready, default=str), flush=True)
        while not stopping and rclpy.ok():
            rclpy.spin_once(stack.node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        code = 1
        stack.result['error'] = str(exc)
        (stack.dir/'failed.json').write_text(json.dumps({'error': str(exc)})+'\n')
        print('FAILED '+str(exc), flush=True)
    finally:
        stack.finish()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
