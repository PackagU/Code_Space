"""Jetson (real robot) adapters for the skill-test state machines. ROS 2 Humble, rclpy.

Interfaces (all configurable in config/skill_tests.yaml; nothing here opens a serial port or a camera device):
  base    geometry_msgs/Twist on `topics.cmd_vel` (default /cmd_vel = nav_safety_gate input, never the motor topic);
          nav_msgs/Odometry `topics.odom`, sensor_msgs/LaserScan `topics.scan` (sensor-data QoS, freshness checked),
          TF map->odom. Every command passes the shared motion check (skilltests.scan_core): front / rear / rotation
          clearance of the body incl. a carried load, unknown or stale data stops.
  arm     robot_arm_pkg arm_sequence node: JSON on /packagu_arm/command, status on /packagu_arm/status (taught views
          front_view/floor_view and press cycles 1/2; the real arm has no IK yet). Timeouts send `cancel`; with
          --execute a completion from a simulated or disconnected arm is rejected.
  vision  frames from the floor reader HTTP server (/frame.jpg, shares the single USB webcam) bracketed by /api/state
          to reject stale frames; buttons with tools/button_arm_test/vision.py; floor from /api/state with the
          mission's reader_observation rules and strictly increasing frame numbers per stream.
  lift    std_msgs/Float32 target/position + sensor_msgs/Range ultrasonic — NOT YET WIRED ON THE ROBOT (no lift
          driver in src/). Pickup refuses to move until `lift.cmd_topic`/`pos_topic` (and the ultrasonic topic in
          ultrasonic mode) deliver fresh data.
  floors  auto_floor_orchestrator: target_floor parameter, /floor_orchestrator/status READY, then fresh AMCL with
          small covariance before using arrival-floor coordinates.
Without --execute every actuator call is logged and skipped (dry run); sensing still runs.
"""
import json
import math
import re
import sys
import time
import urllib.request
import uuid
from pathlib import Path

import numpy as np

from . import scan_core as sc
from . import vision_core as vc
from .control import drive_to
from .geometry import wrap

ROOT = Path(__file__).resolve().parents[3]
TEMPLATE_NAME = re.compile(r'template_(.+)_(\d+)\.png')


def _import_field_modules():
    sys.path.insert(0, str(ROOT/'tools/button_arm_test'))
    sys.path.insert(0, str(ROOT/'test_workspace/elevator_mission/src/elevator_mission_pkg'))
    import vision as button_vision                          # noqa: E402  (tools/button_arm_test/vision.py)
    from elevator_mission_pkg.elevator_camera import reader_observation   # noqa: E402
    return button_vision, reader_observation


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp_of(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def arm_status_verdict(status, request_id, action, execute):
    """Pure contract check of one arm_sequence status for our request: 'done', 'failed' or None (still running)."""
    if status.get('request_id') != request_id:
        return None
    event = status.get('event')
    if event in ('failed', 'rejected', 'cancelled'):
        return 'failed'
    if event != 'completed':
        return None
    if status.get('action') not in ('', None, action):
        return 'failed'
    if execute and (status.get('simulation_mode') or not status.get('hardware_connected', False)):
        return 'failed'                         # a simulated / disconnected completion is not physical readiness
    return 'done'


class FloorFrames:
    """Strictly fresh floor-reader observations: same stream, increasing frame number and capture time, captured
    after the posture cutoff. A stream change or an older/repeated frame returns None (resets the streak)."""

    def __init__(self):
        self.stream, self.seq, self.captured, self.cutoff = None, 0, 0.0, 0.0

    def accept(self, obs):
        if not obs.get('valid'):
            return None
        stream, seq, captured = obs.get('stream_id'), obs.get('frame_seq'), obs.get('captured_at')
        if stream != self.stream:
            self.stream, self.seq, self.captured = stream, seq if type(seq) is int else 0, captured or 0.0
            return None
        if type(seq) is not int or seq <= self.seq or captured is None or captured <= self.captured \
                or captured < self.cutoff:
            return None
        self.seq, self.captured = seq, captured
        return obs.get('floor')


def lift_step_plan(pos, cmd, step, limit):
    """Next lift command, or None when the next step would pass `limit` (pure, for tests)."""
    nxt = max(cmd, pos)+step
    return None if nxt > limit+1e-9 else nxt


class RosClock:
    def __init__(self, node):
        self.node = node

    def now(self):
        return time.monotonic()

    def sleep(self, seconds):
        import rclpy
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(.05, max(0.0, end-time.monotonic())))


class RosBase:
    def __init__(self, node, clock, cfg, execute, log):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
        self.node, self.clock, self.cfg, self.execute, self.log = node, clock, cfg, execute, log
        self.Twist = Twist
        self.odom = self.scan_msg = None
        self.loaded = None                       # unknown until the runner states it (--load-state)
        t = cfg['topics']
        node.create_subscription(Odometry, t['odom'], lambda m: setattr(self, 'odom', m), qos_profile_sensor_data)
        node.create_subscription(LaserScan, t['scan'], lambda m: setattr(self, 'scan_msg', m), qos_profile_sensor_data)
        self.pub = node.create_publisher(Twist, t['cmd_vel'], 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)

    def now_ros(self):
        return self.node.get_clock().now().nanoseconds*1e-9

    def ready(self, timeout=10.0):
        """Fresh odometry, scan and TF map->odom within `timeout`; returns (ok, details)."""
        end = time.monotonic()+timeout
        details = {}
        while time.monotonic() < end:
            details = {'odom': self.pose() is not None, 'scan': self.scan() is not None, 'tf_map_odom': self.tf_ok()}
            if all(details.values()):
                break
            self.clock.sleep(.1)
        ok = all(details.values())
        details['cmd_vel_other_publishers'] = max(0, self.node.count_publishers(self.cfg['topics']['cmd_vel'])-1)
        return ok, details

    def tf_ok(self):
        try:
            self.map_to_odom((0.0, 0.0, 0.0))
            return True
        except Exception:
            return False

    def pose(self):
        o = self.odom
        if o is None or not sc.fresh(stamp_of(o), self.now_ros(), self.cfg['safety']['odom_max_age_s']):
            return None
        p = o.pose.pose
        vals = (p.position.x, p.position.y, yaw_of(p.orientation))
        return vals if all(math.isfinite(v) for v in vals) else None

    def map_to_odom(self, pose):
        from rclpy.time import Time
        t = self.tf.lookup_transform('odom', 'map', Time())
        tr, a = t.transform.translation, yaw_of(t.transform.rotation)
        x, y, yaw = pose
        return (tr.x+math.cos(a)*x-math.sin(a)*y, tr.y+math.sin(a)*x+math.cos(a)*y, wrap(yaw+a))

    def set_loaded(self, flag):
        self.loaded = bool(flag)

    def front_extent(self):
        if self.loaded is None:
            raise RuntimeError('load state unknown: pass --load-state loaded|empty')
        r = self.cfg['robot']
        return r['front_extent_loaded_m'] if self.loaded else r['front_extent_empty_m']

    def geometry(self):
        r = self.cfg['robot']
        return {'front_extent': self.front_extent(), 'rear_extent': r['rear_extent_m'],
                'half_width': max(r['half_width_m'], r['load_half_width_m'] if self.loaded else 0.0)}

    def scan(self):
        s = self.scan_msg
        if s is None or not sc.fresh(stamp_of(s), self.now_ros(), self.cfg['safety']['scan_max_age_s']):
            return None
        r = self.cfg['robot']
        return sc.Scan(s.angle_min, s.angle_increment, s.ranges, s.range_min, s.range_max,
                       (r['lidar_xy'][0], r['lidar_xy'][1], r.get('lidar_yaw', 0.0)))

    def range_at(self, bearing):
        scan = self.scan()
        return None if scan is None else scan.range_at(bearing)

    def wall(self, bearing):
        scan, pose = self.scan(), self.pose()
        return None if scan is None or pose is None else scan.wall(bearing, pose)

    def send(self, v, w):
        lim = self.cfg['limits']
        v, w = max(-lim['max_v'], min(lim['max_v'], v)), max(-lim['max_w'], min(lim['max_w'], w))
        if self.execute:
            cmd = self.Twist()
            cmd.linear.x, cmd.angular.z = float(v), float(w)
            self.pub.publish(cmd)

    def go_to(self, target, min_front=None, **kw):
        if not self.execute:
            self.log({'dry_run': 'go_to', 'target': [round(v, 3) for v in target], **kw})
            return {'status': 'REACHED', 'dry_run': True}
        s = self.cfg['safety']
        margins = {'front': s['front_m'] if min_front is None else min_front, 'rear': s['rear_m'], 'sweep': s['sweep_m']}
        kw.setdefault('timeout', self.cfg['limits']['drive_timeout_s'])
        return drive_to(self.pose, self.send, self.clock.sleep, self.clock.now, target,
                        check=lambda v, w: sc.motion_check(self.scan(), v, w, self.geometry(), margins), **kw)

    def turn_by(self, angle):
        pose = self.pose()
        if pose is None:
            return {'status': 'STALE_POSE'}
        x, y, yaw = pose
        return self.go_to((x, y, wrap(yaw+angle)))

    def stop(self):
        for _ in range(5):
            try:
                self.send(0.0, 0.0)
                self.clock.sleep(.05)
            except Exception:                    # cleanup keeps trying even while ROS is shutting down
                pass


class RosArm:
    def __init__(self, node, clock, cfg, execute, log):
        from std_msgs.msg import String
        self.node, self.clock, self.cfg, self.execute, self.log = node, clock, cfg, execute, log
        self.String = String
        self.status, self.active = {}, None
        self.current_view, self.view_time = None, 0.0
        a = cfg['arm']
        self.pub = node.create_publisher(String, a['command_topic'], 10)
        node.create_subscription(String, a['status_topic'], self.on_status, 10)

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('request_id'):
            self.status[data['request_id']] = data

    def command(self, body, timeout=None):
        rid = 'skill-'+uuid.uuid4().hex[:12]
        body = {'request_id': rid, **body}
        if not self.execute:
            self.log({'dry_run': 'arm', 'command': body})
            return {'done': True, 'reached': True, 'dry_run': True}
        self.active = rid
        self.pub.publish(self.String(data=json.dumps(body)))
        end = time.monotonic()+(timeout or self.cfg['arm']['timeout_s'])
        while time.monotonic() < end:
            verdict = arm_status_verdict(self.status.get(rid, {}), rid, body['action'], self.execute)
            if verdict:
                self.active = None
                ok = verdict == 'done'
                return {'done': ok, 'reached': ok, 'status': self.status.get(rid)}
            self.clock.sleep(.05)
        self.cancel_active()
        return {'done': False, 'reached': False, 'why': 'arm status timeout (cancel sent)', 'request_id': rid}

    def cancel_active(self):
        if self.active is None or not self.execute:
            return
        body = {'request_id': 'skill-'+uuid.uuid4().hex[:12], 'action': 'cancel', 'target_request_id': self.active}
        try:
            self.pub.publish(self.String(data=json.dumps(body)))
        except Exception:
            pass
        self.log({'arm_cancel': body})
        self.active = None

    def posture(self, name):
        if name != 'stow':
            raise ValueError(f'unknown posture {name}')
        out = self.command({'action': 'stow'})
        if out.get('done'):
            self.current_view = None
        return out

    def view(self, name, target=None):
        view = self.cfg['arm']['views'][name]
        out = self.command({'action': 'view', 'view': view})
        self.clock.sleep(self.cfg['arm']['view_settle_s'])
        self.current_view = view if out.get('done') else None
        self.view_time = time.time()            # frames captured before this instant are not used
        return {**out, 'view': view}

    def press(self, button):
        kind = 'call' if button.get('kind') == 'hall' else 'destination'
        cycle = self.cfg['arm']['press_cycle'][kind]
        return self.command({'action': 'press', 'target': kind, 'button': str(button.get('label', 'X')).upper(),
                             'press_cycle': int(cycle)})


class RosVision:
    """Same image processing as the simulation (skilltests.vision_core). The camera pose comes from the field
    calibration of each taught arm view (`vision.views[<view>]`: xyz in the robot frame, yaw, pitch, hfov)."""

    def __init__(self, base, arm, cfg, log):
        self.base, self.arm, self.cfg, self.log = base, arm, cfg, log
        self.bv, self.reader_observation = _import_field_modules()
        self.templates = None
        self.floors = FloorFrames()

    def reader_state(self):
        try:
            with urllib.request.urlopen(self.cfg['vision']['floor_state_url'], timeout=1.0) as resp:
                return json.loads(resp.read().decode())
        except (OSError, ValueError):
            return None

    def frame(self):
        """A frame captured after the arm settled: /api/state before and after /frame.jpg must show the same live
        stream with a fresh capture time newer than the posture cutoff."""
        import cv2
        before = self.reader_state()
        try:
            with urllib.request.urlopen(self.cfg['vision']['frame_url'], timeout=2.0) as resp:
                data = np.frombuffer(resp.read(), np.uint8)
        except OSError as exc:
            self.log({'camera_error': str(exc)})
            return None
        after = self.reader_state()
        now, max_age = time.time(), self.cfg['vision']['frame_max_age_s']
        for st in (before, after):
            if not isinstance(st, dict) or st.get('error') or not sc.fresh(st.get('updated'), now, max_age) \
                    or (st.get('updated') or 0.0) < self.arm.view_time:
                self.log({'camera_stale': True})
                return None
        if before.get('stream_id') != after.get('stream_id'):
            return None
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def camera(self, img):
        view = self.arm.current_view
        if view is None or view not in self.cfg['vision']['views']:
            return None, None
        cal = self.cfg['vision']['views'][view]
        pose = self.base.pose()
        if pose is None:
            return None, None
        return vc.camera_pose(pose, cal), vc.fx_from_hfov(img.shape[1], cal['hfov_rad'])

    def lit_fraction(self, img, pixel):
        u, v = (int(round(c)) for c in pixel)
        r = int(self.cfg['vision']['lit_patch_px'])
        patch = img[max(0, v-r):v+r, max(0, u-r):u+r]
        if patch.size == 0:
            return None
        return float(np.count_nonzero(vc.mask(patch, self.cfg['vision']['lit_hsv'])))/patch[..., 0].size

    def find_button(self, kind, label, prior, hint=None):
        import cv2
        img = self.frame()
        if img is None:
            return None
        pose, fx = self.camera(img)
        if pose is None:
            return None
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found = self.bv.find_buttons(gray)
        chosen = self.pick_button(gray, found, label) if found else None
        if chosen is None:
            return None
        u, v = chosen['center']
        size = (img.shape[1], img.shape[0])
        _, d = vc.ray(u, v, size, fx, pose)
        robot = self.base.pose()
        if robot is None:
            return None
        wall = self.base.wall(wrap(math.atan2(d[1], d[0])-robot[2]))
        if wall is None:
            return None
        q, n, fit = wall
        p = vc.on_wall(u, v, size, fx, pose, (q+n*self.cfg['vision']['button_protrusion_m'], n))
        if p is None or math.hypot(p[0]-prior[0], p[1]-prior[1]) > self.cfg['vision']['prior_max_m']:
            return None                                  # not the taught button (another panel, a reflection)
        return {'xyz': [float(c) for c in p], 'normal': [float(n[0]), float(n[1])], 'kind': kind, 'label': label,
                'pixel': [float(u), float(v)], 'seen': len(found), 'wall_fit': fit,
                'baseline_lit': self.lit_fraction(img, (u, v))}

    def pick_button(self, gray, found, label):
        """Read every cap (button_arm_test.vision.read_button: UP/DOWN arrows without enrollment, digits against
        the enrolled templates) and return the one whose label matches."""
        if self.templates is None:
            self.templates = self.load_templates()
        want = str(label).upper()
        for b in found:
            read, _, _, _ = self.bv.read_button(gray, b, self.templates)
            if str(read).upper() == want:
                return b
        return None

    def load_templates(self):
        """tools/button_arm_test/data/template_<label>_<n>.png -> stacked templates (same as its app.py)."""
        import cv2
        prepared = {}
        for file in sorted((ROOT/self.cfg['vision']['digit_templates']).glob('template_*.png')):
            match = TEMPLATE_NAME.fullmatch(file.name)
            mask = cv2.imread(str(file), 0)
            if match and mask is not None:
                prepared.setdefault(match.group(1), []).append(self.bv.prepare(mask))
        return self.bv.stack(prepared)

    def button_lit(self, button):
        """Re-acquire the same labelled button in a fresh frame and compare its LED signal with the pre-press
        baseline (an unchanged white cap is not 'lit'). [현장 보정] lit_hsv / thresholds."""
        import cv2
        img = self.frame()
        if img is None:
            return False
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found = self.bv.find_buttons(gray)
        again = self.pick_button(gray, found, button['label']) if found else None
        if again is None:
            return False
        frac = self.lit_fraction(img, again['center'])
        base = button.get('baseline_lit') or 0.0
        v = self.cfg['vision']
        self.log({'button_lit_fraction': frac, 'baseline': base})
        return frac is not None and frac >= v['lit_min_fraction'] and frac-base >= v['lit_min_increase']

    def floor(self):
        """One strictly fresh floor reading from tools/floor_reader (/api/state) with the mission's reader_observation
        rules; repeated, older, pre-posture or invalid frames return None."""
        state = self.reader_state()
        if state is None:
            return None
        self.floors.cutoff = self.arm.view_time
        return self.floors.accept(self.reader_observation(state, time.time(), self.cfg['vision']['floor_max_age_s']))

    def parcels(self):
        img = self.frame()
        if img is None:
            return []
        pose, fx = self.camera(img)
        if pose is None:
            return []
        v = self.cfg['vision']
        return vc.parcels(img, v['parcel_hsv'], v['min_blob_px'], fx, pose, v['parcel_mid_z_m'])

    def pallet_face(self, near_xy):
        img = self.frame()
        if img is None:
            return None
        pose, fx = self.camera(img)
        if pose is None:
            return None
        v = self.cfg['vision']
        uv = vc.project((near_xy[0], near_xy[1], .05), (img.shape[1], img.shape[0]), fx, pose)
        return vc.pallet_face(img, v['pallet_hsv'], v['min_blob_px'], fx, pose, uv)


class RosLift:
    def __init__(self, node, clock, cfg, execute, log):
        self.node, self.clock, self.cfg, self.execute, self.log = node, clock, cfg, execute, log
        self.pos = self.pos_t = self.range = self.range_t = None
        lift = cfg['lift']
        self.configured = bool(lift.get('cmd_topic') and lift.get('pos_topic')
                               and (lift['mode'] == 'fixed' or lift.get('ultrasonic_topic')))
        if self.configured:
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Range
            from std_msgs.msg import Float32
            self.Float32 = Float32
            self.pub = node.create_publisher(Float32, lift['cmd_topic'], 10)
            node.create_subscription(Float32, lift['pos_topic'], self.on_pos, 10)
            if lift['mode'] == 'ultrasonic':
                node.create_subscription(Range, lift['ultrasonic_topic'], self.on_range, qos_profile_sensor_data)

    def on_pos(self, msg):
        if math.isfinite(msg.data):
            self.pos, self.pos_t = float(msg.data), time.monotonic()

    def on_range(self, msg):
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            self.range, self.range_t = float(msg.range), time.monotonic()
        elif math.isinf(msg.range) and msg.range > 0:
            self.range, self.range_t = math.inf, time.monotonic()

    def fresh_pos(self):
        ok = self.pos_t is not None and time.monotonic()-self.pos_t <= self.cfg['lift']['max_age_s']
        return self.pos if ok else None

    def fresh_range(self):
        ok = self.range_t is not None and time.monotonic()-self.range_t <= self.cfg['lift']['max_age_s']
        return self.range if ok else None

    def ready(self):
        if not self.configured:
            return {'ok': False, 'why': 'lift interface not configured (lift.cmd_topic/pos_topic/ultrasonic_topic)'}
        end = time.monotonic()+3.0
        while time.monotonic() < end:
            if self.fresh_pos() is not None and (self.cfg['lift']['mode'] == 'fixed' or self.fresh_range() is not None):
                return {'ok': True, 'lift_m': self.pos, 'range_m': self.range}
            self.clock.sleep(.1)
        return {'ok': False, 'why': 'no fresh lift position / ultrasonic data'}

    def command(self, value):
        if self.execute:
            self.pub.publish(self.Float32(data=float(value)))

    def wait_pos(self, target, deadline):
        while time.monotonic() < deadline:
            pos = self.fresh_pos()
            if pos is not None and abs(pos-target) <= self.cfg['lift']['tolerance_m']:
                return True
            self.clock.sleep(.05)
        return False

    def halt(self):
        """Hold the current height (cleanup / failures)."""
        pos = self.fresh_pos()
        if self.configured and self.execute and pos is not None:
            try:
                self.command(pos)
            except Exception:
                pass

    def raise_for_carry(self):
        lift = self.cfg['lift']
        if not self.configured:
            return {'reached': False, 'why': 'lift interface not configured'}
        if not self.execute:
            self.log({'dry_run': 'lift raise', 'mode': lift['mode']})
            return {'reached': False, 'dry_run': True, 'why': 'dry run: no load top measured'}
        cap = self.cfg['tests']['parcel_pickup']['arm_clear_top_m']
        limit = min(lift['max_m'], cap-lift['load_top_offset_m'])
        deadline = time.monotonic()+lift['timeout_s']
        pos = self.fresh_pos()
        if pos is None:
            return {'reached': False, 'why': 'no fresh lift position'}
        if lift['mode'] == 'fixed':
            target = min(lift['fixed_m'], limit)
            self.command(target)
            ok = self.wait_pos(target, deadline)
            pos = self.fresh_pos()
            return {'reached': ok, 'mode': 'fixed', 'lift_m': pos, 'cap_m': cap,
                    'load_top_m': None if pos is None else pos+lift['load_top_offset_m']}
        cmd, seen, why = pos, False, None
        while True:
            current = self.fresh_pos()
            nxt = lift_step_plan(cmd if current is None else current, cmd, lift['step_m'], limit)
            if nxt is None:
                why = 'load-top cap reached before the ultrasonic saw the parcel'
                break
            self.command(nxt)
            cmd = nxt
            if not self.wait_pos(cmd, min(deadline, time.monotonic()+lift['step_timeout_s'])):
                why = 'lift did not follow the step'
                break
            rng = self.fresh_range()
            if rng is None:
                why = 'ultrasonic data stale'
                break
            if rng <= lift['detect_range_m']:
                seen = True
                break
            if time.monotonic() > deadline:
                why = 'deadline'
                break
        self.halt()
        pos = self.fresh_pos()
        top = None if pos is None else pos+lift['load_top_offset_m']
        return {'reached': bool(seen and top is not None and top <= cap), 'mode': 'ultrasonic', 'lift_m': pos,
                'range_m': self.range, 'cap_m': cap, 'load_top_m': top, 'why': why}

    def lower(self):
        if not self.configured:
            return {'reached': False, 'why': 'lift interface not configured'}
        if not self.execute:
            self.log({'dry_run': 'lift lower'})
            return {'reached': True, 'dry_run': True}
        self.command(0.0)
        return {'reached': self.wait_pos(0.0, time.monotonic()+self.cfg['lift']['timeout_s'])}


class RosLoad:
    """The fork holds the pallet mechanically; engagement is not sensed yet (field TODO). The robot counts as
    loaded only after the lift raise is confirmed (ParcelPickup calls base.set_loaded(True))."""

    def __init__(self, base):
        self.base = base

    def attach(self):
        return {'ok': True, 'sensed': False}

    def detach(self):
        self.base.set_loaded(False)
        return {'ok': True, 'sensed': False}


class FloorSwitch:
    """Arrival-floor map: set the orchestrator target, wait for READY on that floor and for fresh, converged AMCL."""

    def __init__(self, node, clock, cfg, execute, log):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import String
        self.node, self.clock, self.cfg, self.execute, self.log = node, clock, cfg, execute, log
        self.status = self.amcl = None
        self.samples = []                               # (stamp, msg) of AMCL poses, newest last
        f = cfg['floors']
        node.create_subscription(String, f['status_topic'], self.on_status, 10)
        node.create_subscription(PoseWithCovarianceStamped, f['amcl_topic'], self.on_amcl, 10)
        self.seed_pub = node.create_publisher(PoseWithCovarianceStamped, f['initialpose_topic'], 10)
        self.Pose = PoseWithCovarianceStamped

    def on_amcl(self, msg):
        self.amcl = msg
        self.samples = (self.samples+[(stamp_of(msg), msg)])[-50:]

    def seed(self, pose):
        """Publish the robot-side cabin stand pose (map) as /initialpose; returns the epoch (ROS time)."""
        now = self.node.get_clock().now()
        if not self.execute:
            self.log({'dry_run': 'initialpose', 'pose': [round(v, 3) for v in pose]})
            return now.nanoseconds*1e-9
        msg = self.Pose()
        msg.header.frame_id = 'map'
        msg.header.stamp = now.to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(pose[0]), float(pose[1])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(pose[2]/2), math.cos(pose[2]/2)
        f = self.cfg['floors']
        msg.pose.covariance[0] = msg.pose.covariance[7] = f['seed_cov_xy']
        msg.pose.covariance[35] = f['seed_cov_yaw']
        for _ in range(3):
            self.seed_pub.publish(msg)
            self.clock.sleep(.2)
        self.log({'initialpose': [round(v, 3) for v in pose]})
        return now.nanoseconds*1e-9

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def set_target(self, floor):
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        if not self.execute:
            self.log({'dry_run': 'orchestrator target', 'floor': floor})
            return True
        client = self.node.create_client(SetParameters, self.cfg['floors']['set_parameters_service'])
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(name='target_floor', value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                                               string_value=floor))]
        fut = client.call_async(req)
        end = time.monotonic()+5.0
        while not fut.done() and time.monotonic() < end:
            self.clock.sleep(.05)
        return fut.done() and fut.result() is not None and all(r.successful for r in fut.result().results)

    def wait_ready(self, floor, stand=None):
        """READY on `floor`, then (optionally) seed the robot-side stand pose, then several fresh converged AMCL
        samples stamped after that epoch (a single old or future-stamped sample is not evidence of the new map)."""
        f = self.cfg['floors']
        end = time.monotonic()+f['ready_timeout_s']
        epoch = None
        while time.monotonic() < end:
            st = self.status or {}
            if st.get('phase') == 'ready' and st.get('current_floor') == floor:
                if epoch is None:
                    epoch = self.seed(stand) if stand is not None and f['reseed_initialpose'] else \
                        self.node.get_clock().now().nanoseconds*1e-9
                now = self.node.get_clock().now().nanoseconds*1e-9
                good = [m for t, m in self.samples
                        if t > epoch and -.25 <= now-t <= f['amcl_max_age_s']+f['amcl_min_samples']
                        and m.pose.covariance[0] <= f['amcl_cov_xy_max'] and m.pose.covariance[7] <= f['amcl_cov_xy_max']
                        and m.pose.covariance[35] <= f['amcl_cov_yaw_max']]
                if len(good) >= f['amcl_min_samples'] and now-stamp_of(good[-1]) <= f['amcl_max_age_s']:
                    return True
            self.clock.sleep(.2)
        return False


def make_io(node, cfg, execute, log):
    from types import SimpleNamespace
    clock = RosClock(node)
    base = RosBase(node, clock, cfg, execute, log)
    arm = RosArm(node, clock, cfg, execute, log)
    return SimpleNamespace(clock=clock, base=base, arm=arm, vision=RosVision(base, arm, cfg, log),
                           lift=RosLift(node, clock, cfg, execute, log), load=RosLoad(base),
                           floors=FloorSwitch(node, clock, cfg, execute, log))
