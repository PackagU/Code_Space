"""Simulation adapters for the shared skill-test state machines (tools/jetson_skill_tests/skilltests).

The machines are the same code the Jetson tests run; only these adapters are simulation specific:
  base   /cmd_vel -> nav_safety_gate -> OpenCR shim (encoder odometry feedback, LiDAR clearance, TF map->odom)
  arm    taught pitch-only poses (arm_geometry.py): fixed yaw/roll, camera views and button cycles
  vision arm-tip camera (perception.py, OpenCV) + LiDAR wall line
  lift   lift_joint (ultrasonic stop or small fixed lift, arm_lift.py)
  load   parcel_mock attach/detach (no contact or load validation)
Ground truth is used only by the clearance watchdog and for the evaluation fields written by run_e2e.py.
"""
import collections
import hashlib
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time

import arm_geometry as ag
from dressing import LIFT_BASE_X, MAST_HALF
from perception import Perception, yaw_of

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]/'tools/jetson_skill_tests'))
from skilltests import scan_core as sc   # noqa: E402
from skilltests import vision_core as vc   # noqa: E402
from skilltests.control import drive_to   # noqa: E402
from skilltests.press_memory import PressMemory   # noqa: E402


class SimClock:
    def __init__(self, node):
        self.node = node

    def now(self):
        return self.node.now()

    def sleep(self, seconds):
        self.node.sim_pause(seconds)


class SimBase:
    """Robot data only for control: encoder odometry (/odom), LiDAR (/scan), TF map->odom. The ground-truth clearance
    watchdog is a simulation-only hard stop on top of the shared motion check (skilltests.scan_core)."""

    def __init__(self, stack, per, params):
        self.stack, self.node, self.per, self.P = stack, stack.node, per, params
        lidar = params['lidar']['pose_xyz']
        self.lidar = (lidar[0], lidar[1], 0.0)
        fp = params['robot']['footprint']
        self.rear, self.half_width = -min(p[0] for p in fp), max(abs(p[1]) for p in fp)
        self.safety = params['perception']['safety']
        self.odom_hist = collections.deque(maxlen=20000)
        self.node.create_subscription(Odometry, '/odom', self.on_odom, 50)
        pal = params['parcel']['pallet']
        self.loaded_front = LIFT_BASE_X+MAST_HALF+float(pal['rear_gap_m'])+pal['size_m'][0]
        self.loaded_half = max(self.half_width, pal['size_m'][1]/2)
        self.empty_front = float(params['lift']['fork_tip_x_m'])

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom_hist.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, p.position.x, p.position.y,
                               yaw_of(p.orientation)))

    def odom_at(self, t):
        hist = [h for h in self.odom_hist if abs(h[0]-t) <= .1]
        return min(hist, key=lambda h: abs(h[0]-t))[1:] if hist else None

    def pose(self):
        odom = self.node.odom
        if odom is None:
            return None
        stamp = odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9
        if not sc.fresh(stamp, self.node.now(), self.safety['odom_max_age_s']):
            return None
        x, y, _, yaw = self.per.odom_pose()
        return (x, y, yaw)

    def map_to_odom(self, pose):
        t = self.node.buffer.lookup_transform('odom', 'map', Time())
        tr, a = t.transform.translation, yaw_of(t.transform.rotation)
        x, y, yaw = pose
        return (tr.x+math.cos(a)*x-math.sin(a)*y, tr.y+math.sin(a)*x+math.cos(a)*y,
                math.atan2(math.sin(yaw+a), math.cos(yaw+a)))

    def odom_to_map(self, xy):
        t = self.node.buffer.lookup_transform('map', 'odom', Time())
        tr, a = t.transform.translation, yaw_of(t.transform.rotation)
        return (tr.x+math.cos(a)*xy[0]-math.sin(a)*xy[1], tr.y+math.sin(a)*xy[0]+math.cos(a)*xy[1])

    def loaded(self):
        return (self.node.parcel or {}).get('state') == 'attached'

    def set_loaded(self, flag):
        pass                                     # the simulation reads the carried state from the parcel mock

    def front_extent(self):
        return self.loaded_front if self.loaded() else self.empty_front

    def scan(self):
        msg = self.node.scan
        if msg is None:
            return None
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not sc.fresh(stamp, self.node.now(), self.safety['scan_max_age_s']):
            return None
        return sc.Scan(msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max, self.lidar)

    def geometry(self):
        return {'front_extent': self.front_extent(), 'rear_extent': self.rear,
                'half_width': self.loaded_half if self.loaded() else self.half_width}

    def range_at(self, bearing):
        scan = self.scan()
        return None if scan is None else scan.range_at(bearing)

    def wall(self, bearing):
        scan, pose = self.scan(), self.pose()
        return None if scan is None or pose is None else scan.wall(bearing, pose)

    def go_to(self, target, min_front=None, **kw):
        node = self.node
        self.stack.resume()
        margins = {'front': self.safety['front_m'] if min_front is None else min_front,
                   'rear': self.safety['rear_m'], 'sweep': self.safety['sweep_m']}

        def send(v, w):
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(v), float(w)
            node.direct_pub.publish(cmd)

        def check(v, w):
            if node.samples and node.samples[-1][5] <= .015:      # sim watchdog (ground truth footprint vs walls)
                return 'CLEARANCE_STOP'
            return sc.motion_check(self.scan(), v, w, self.geometry(), margins)
        result = drive_to(self.pose, send, node.sim_pause, node.now, target, check=check, **kw)
        self.stack.stop()
        return result

    def turn_by(self, angle):
        x, y, yaw = self.pose()
        return self.go_to((x, y, math.atan2(math.sin(yaw+angle), math.cos(yaw+angle))))

    def stop(self):
        for _ in range(3):
            self.node.direct_pub.publish(Twist())
            self.node.sim_pause(.05)
        self.stack.stop()


class SimArm:
    def __init__(self, armlift, per, params, vision):
        self.a, self.per, self.P, self.vision = armlift, per, params, vision

    def posture(self, name):
        if name == 'stow':
            return self.a.stow()
        raise ValueError(f'unknown posture {name}')

    def view(self, name, target, duration_ms=1500):
        """Move to the FIXED view posture `name` (world_params arm.view_poses, like the Jetson's taught views; the
        arm is never aimed). Reached = the arm got there AND `target` (odom) projects inside the image with
        `view_margin_frac` to spare. in_view False = the posture is fine but the target is out of the picture from
        where the robot stands: the machine may back off or turn."""
        arm = self.P['arm']
        pwm = dict(zip(('000', '001', '002', '003'), arm['view_poses'][name]))   # fixed posture, never aimed
        moved = self.a.move(pwm, duration_ms)
        angles = {spec['name']: ag.pwm_angle(arm, spec['servo'], pwm[spec['servo']]) for spec in arm['joints']}
        _, _, cp, cR = ag.fk(arm, angles, self.per.odom_pose())
        err = math.acos(max(-1.0, min(1.0, float(cR[:, 0]@(np.asarray(target, float)-cp))
                                      / float(np.linalg.norm(np.asarray(target, float)-cp)))))
        px = vc.project(target, self.per.size(), self.per.fx, (cp, cR))
        w, h = self.per.size()
        m = float(self.P['perception']['view_margin_frac'])
        seen = px is not None and m*w <= px[0] <= (1-m)*w and m*h <= px[1] <= (1-m)*h
        out = {'reached': moved['reached'] and seen, 'off_axis_deg': round(math.degrees(err), 1), 'view': name,
               'pixel': None if px is None else [round(px[0], 1), round(px[1], 1)]}
        if moved['reached'] and not seen:
            out['in_view'] = False
        return out

    def press(self, button):
        """Press with the taught cycle for this button kind (servo_protocol PRESS_CYCLES, the angles the Jetson
        runs). Whether it lands on the button is decided by the stance; the elevator node judges the contact."""
        number = int(self.P['arm']['press_cycle']['call' if button.get('kind') == 'hall' else 'destination'])
        evidence = {'label': button.get('label'), 'lit': False, 'source': 'arm_camera', 'phase': 'press_hold'}
        def observe_light():
            evidence['lit'] = bool(self.vision.button_lit(button))
        out = self.a.press_cycle(number, on_press=observe_light)
        return {'camera_confirmation': evidence, 'done': bool(out['completed']), 'cycle': number, 'label': out.get('label'),
                'steps': [{k: row.get(k) for k in ('pose', 'reached', 'tip_map')} for row in out['steps']]}


class SimVision:
    def __init__(self, base, per, params, indicators):
        self.base, self.per, self.P, self.ind = base, per, params, indicators

    def project(self, xyz, pose):
        return vc.project(xyz, self.per.size(), self.per.fx, pose)

    def find_button(self, kind, label, prior, hint=None):
        bgr, pose = self.per.frame()
        self.save(f'button_{kind}_{label}', bgr)
        if bgr is None:
            return None
        x, y, yaw = self.base.pose()
        bearing = math.atan2(prior[1]-y, prior[0]-x)-yaw
        wall = self.base.wall(math.atan2(math.sin(bearing), math.cos(bearing)))
        if wall is None:
            return self.why('no LiDAR wall on the prior bearing', kind=kind, bearing=round(bearing, 3))
        q, n, fit = wall
        plane = (q+n*float(self.P['perception']['button_protrusion_m']), n)
        found = []
        for b in self.per.buttons(bgr):
            p = self.per.on_wall(b['u'], b['v'], pose, plane)
            if p is not None and math.hypot(p[0]-prior[0], p[1]-prior[1]) < .6:
                found.append((b['area'], p))
        # one detection per cap: merge pieces within 3 cm (keep the larger), then the largest 4 (floor board) or
        # 2 (call board) - a stray bright speck must not shift the row/column reading
        found.sort(key=lambda f: -f[0])
        cands = []
        for area, p in found:
            if all(math.dist(p, q) > .03 for q in cands):
                cands.append(p)
        cands = cands[:2 if kind == 'hall' else 4]
        if not cands:
            return self.why('no cap on the wall plane near the prior', kind=kind, caps=len(self.per.buttons(bgr)))
        if kind == 'hall':                          # demo call board: UP on top, DOWN below
            if len(cands) < 2:
                return self.why('fewer than 2 call caps', seen=len(cands))
            p = max(cands, key=lambda c: c[2]) if label == 'UP' else min(cands, key=lambda c: c[2])
        else:                                       # demo floor board: 1 2 on top, 3 4 below (as seen facing it)
            if len(cands) < 4:
                return self.why('fewer than 4 floor caps', seen=len(cands))
            rx, ry = -n[1], n[0]                    # viewer's right on the wall
            rows = sorted(cands, key=lambda c: -c[2])
            row = sorted(rows[:2] if label in ('1', '2') else rows[2:4], key=lambda c: c[0]*rx+c[1]*ry)
            p = row[0] if label in ('1', '3') else row[1]
            board = vc.floor_board_pose(self.per.buttons(bgr), self.P['dressing']['demo_buttons']['car_caps'],
                                       label, self.per.size(), self.per.fx, pose, p, n)
            if board is None:
                return self.why('floor board image geometry disagrees with LiDAR', label=label)
            p, n = board['xyz'], board['normal']
            fit = dict(fit, camera_board_reprojection_px=board['reprojection_px'])
        return {'xyz': [float(v) for v in p], 'normal': [float(n[0]), float(n[1])], 'seen': len(cands),
                'wall_fit': fit, 'label': label, 'kind': kind}

    debug_dir = None
    shots = 0

    def why(self, reason, **data):
        """Record why no button was returned (debug_dir/vision.jsonl) and return None."""
        if self.debug_dir is not None:
            import json
            with open(self.debug_dir/'vision.jsonl', 'a') as handle:
                handle.write(json.dumps({'t': round(self.per.node.now(), 2), 'why': reason, **data})+'\n')
        return None

    def save(self, tag, bgr):
        if self.debug_dir is not None and bgr is not None:
            import cv2
            SimVision.shots += 1
            cv2.imwrite(str(self.debug_dir/f'camera_{SimVision.shots:03d}_{tag}.png'), bgr)

    def button_lit(self, button):
        bgr, pose = self.per.frame()
        self.save(f"confirm_{button.get('label')}", bgr)
        if bgr is None:
            return False
        prior_uv = self.project(button['xyz'], pose)
        if prior_uv is None:
            return False
        cap = vc.selected_cap(self.per.buttons(bgr), button['kind'], str(button['label']), prior_uv)
        if cap is None:
            self.why('requested cap not reacquired for lamp check', label=button.get('label'))
            return False
        uv = (cap['u'], cap['v'])
        lit = vc.button_light(bgr, uv, self.P['perception']['hsv']['lamp'],
                              radius=min(int(self.P['perception']['lamp_match_px']), max(2, min(cap['bbox'][2:])//2)),
                              min_fraction=.3)
        self.why('selected button lamp', label=button.get('label'), pixel=list(uv), prior_pixel=list(prior_uv), lit=lit)
        return lit

    def parcels(self):
        bgr, pose = self.per.frame()
        self.save('parcel_scan', bgr)
        return [] if bgr is None else self.per.parcels(bgr, pose)

    def pallet_face(self, near_xy):
        bgr, pose = self.per.frame()
        if bgr is None:
            return None
        self.save('pallet_face', bgr)
        uv = self.project((near_xy[0], near_xy[1], .05), pose)
        face = self.per.pallet_face(bgr, pose, near_uv=uv)
        if face is None:
            return None
        return {'mid': [float(v) for v in face['mid']], 'normal': [float(v) for v in face['normal']],
                'width_m': face['width_m']}

    def floor(self):
        bgr, _ = self.per.frame(settle=.2)
        if bgr is None:
            return None
        digit, _ = self.per.digit(bgr)
        self.save(f'display_{digit}', bgr)
        return f'F{digit}' if digit else None


class SimLift:
    """lift_joint through arm_lift (ultrasonic stop or small fixed lift). The load top is computed from the measured
    joint position (robot knowledge) and the command never goes past the allowed load-top height."""

    def __init__(self, armlift):
        self.a = armlift

    def ready(self):
        pos = self.a.node.joints.get(self.a.lift_p['joint'])
        return {'ok': pos is not None, 'lift_m': pos}

    def raise_for_carry(self):
        return self.a.carry_up()

    def lower(self):
        self.a.lift_hold = None
        return self.a.lift_to(self.a.lift_p['down_m'], 'fork_down')


class SimLoad:
    def __init__(self, armlift):
        self.a = armlift

    def attach(self):
        r = self.a.parcel('attach', 'attached')
        return {'ok': r['parcel_ok'], 'gap_m': r['attach_gap_m']}

    def detach(self):
        r = self.a.parcel('detach', 'placed')
        return {'ok': r['parcel_ok']}


class SimPressMemory(PressMemory):
    def __init__(self, stack, base, params, armlift):
        revision = hashlib.sha256(json.dumps({'arm': params['arm'], 'poses': armlift.sp.POSE_TABLES,
                                             'maps': stack.result['config_sha256_12']}, sort_keys=True).encode()).hexdigest()
        super().__init__(HERE.parents[1]/'logs/button_press_memory/sim.json', revision)
        self.base, self.stack = base, stack

    def commit(self, entry):
        super().commit(entry)
        (self.stack.dir/'button_press_memory.json').write_text(self.path.read_text())

    def capture(self, floor, kind, label, button, pose, tip):
        x, y, yaw = pose
        px, py = self.base.odom_to_map((x, y))
        ax, ay = self.base.odom_to_map((x+math.cos(yaw), y+math.sin(yaw)))
        bx, by = self.base.odom_to_map(button['xyz'][:2])
        return {'floor': floor, 'kind': kind, 'label': str(label), 'frame': 'map',
                'button_xyz': [bx, by, button['xyz'][2]], 'press_pose': [px, py, math.atan2(ay-py, ax-px)],
                'arm_pwm': tip.get('pwm'), 'source': 'arm_camera', 'run': self.stack.name,
                'sim_time': self.stack.node.now()}


def make_io(stack, armlift, params, indicators):
    per = Perception(stack.node, params)
    base = SimBase(stack, per, params)
    vision = SimVision(base, per, params, indicators)
    vision.debug_dir = stack.dir           # camera frames used for decisions are kept with the run
    return SimpleNamespace(clock=SimClock(stack.node), base=base, arm=SimArm(armlift, per, params, vision),
                           vision=vision, lift=SimLift(armlift), load=SimLoad(armlift), per=per,
                           press_memory=SimPressMemory(stack, base, params, armlift))
