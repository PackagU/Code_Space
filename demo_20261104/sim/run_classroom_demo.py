#!/usr/bin/env python3
"""11/4 classroom demo - one command simulation test (build -> launch -> 17 steps -> verify -> logs -> cleanup).

  python3 demo_20261104/sim/run_classroom_demo.py [--label 402|310|205|...] [--speed max|3] [--gui] [--repeats N]

Host side: brings up the isolated container (sim/real_maps/sim_container.sh, internal network, ROS_LOCALHOST_ONLY,
domain 78, X11 socket for the arm camera) and re-runs itself inside. Inside: classroom world (generate_classroom.py),
robot with arm + lift + arm-tip camera, OpenCR shim, nav_safety_gate, Nav2 on the generated map, parcel mock with
labelled parcels, buttons + floor display stand-in (classroom_nodes.py). The robot side uses only camera, joints,
encoder odometry, LiDAR, AMCL and taught priors (the same skill-test state machines as the Jetson). Ground truth is
used for evaluation and for the world side only. Results: logs/classroom_demo/<run>/ (report.md, summary.json,
steps.csv, projector/*.png annotated arm-camera frames, trajectory.png). Simulation evidence only.
"""
import os
import shlex
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SC = ROOT/'sim/real_maps/sim_container.sh'


def trampoline():
    if os.environ.get('PACKAGU_SIM_ISOLATED') == '1':
        return
    os.environ.setdefault('PACKAGU_SIM_CONTAINER', 'packagu_sim_classroom')
    os.environ.setdefault('PACKAGU_SIM_DOMAIN', '78')
    os.environ['PACKAGU_SIM_CMD'] = ' '.join(shlex.quote(a) for a in ['python3', *sys.argv])
    subprocess.run(['bash', str(SC), 'up', '--gui'], check=True)      # X11 socket: the arm camera needs GL
    if '--gui' in sys.argv:
        subprocess.Popen([sys.executable, str(ROOT/'sim/real_maps/arrange_gui.py'), '--container',
                          os.environ['PACKAGU_SIM_CONTAINER'], '--wait', '900'], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
    os.execvp('bash', ['bash', str(SC), 'exec', 'python3', '-u', str(Path(sys.argv[0]).resolve().relative_to(ROOT)),
                       *sys.argv[1:]])            # the script that was started (runner or a diagnostic importing it)


trampoline()

import argparse   # noqa: E402
import copy       # noqa: E402
import csv        # noqa: E402
import json       # noqa: E402
import math       # noqa: E402
import time       # noqa: E402

REAL = ROOT/'sim/real_maps'
GEN = HERE/'generated'


def p1_label():
    """--label value (pre-parsed: the scene is generated before the ROS imports)."""
    for i, arg in enumerate(sys.argv):
        if arg == '--label' and i+1 < len(sys.argv):
            return sys.argv[i+1]
        if arg.startswith('--label='):
            return arg.split('=', 1)[1]
    return None


def prepare():
    """Build and generate whatever this checkout is missing (inside the container)."""
    if not (ROOT/'install/slam_pkg/share/slam_pkg').is_dir() or os.environ.get('SIM_REBUILD') == '1':
        print('[prepare] colcon build (logs/real_map_sim/build.log)', flush=True)
        (ROOT/'logs/real_map_sim').mkdir(parents=True, exist_ok=True)
        with open(ROOT/'logs/real_map_sim/build.log', 'w') as log:
            subprocess.run(['colcon', 'build', '--symlink-install', '--base-paths', 'src',
                            'test_workspace/elevator_auto_map_switch/src', '--packages-select', 'common_pkg', 'slam_pkg',
                            'drive_pkg', 'auto_floor_orchestrator_pkg'], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                           check=True)
        print('[prepare] re-run to load the new install space', flush=True)
        os.execvp('bash', ['bash', '-c', 'source install/setup.bash && exec python3 -u "$@"', 'bash', *sys.argv])
    if not (REAL/'generated/robot_arm_cam.urdf').exists():
        print('[prepare] sim/real_maps worlds + robot URDFs', flush=True)
        subprocess.run([sys.executable, str(REAL/'generate_worlds.py')], cwd=ROOT, stdout=subprocess.DEVNULL, check=True)
    print('[prepare] classroom world/map/scene', flush=True)
    label = p1_label()
    if label is not None and (len(label) != 3 or not label.isdigit()):
        raise SystemExit('--label must be 3 digits')
    out = subprocess.run([sys.executable, str(HERE/'generate_classroom.py')]+(['--p1-label', label] if label else []),
                         cwd=ROOT, capture_output=True, text=True, check=True)
    (GEN/'generate.log').write_text(out.stdout+out.stderr)


prepare()
sys.path[:0] = [str(HERE), str(REAL), str(ROOT/'tools/jetson_skill_tests')]

import cv2                                               # noqa: E402
import numpy as np                                       # noqa: E402
import rclpy                                             # noqa: E402
import yaml                                              # noqa: E402
from std_msgs.msg import String                          # noqa: E402

import sim_stack as ss                                   # noqa: E402
from arm_lift import ArmLift                             # noqa: E402
from dressing import pallet_center_ahead                 # noqa: E402
from isolation import assert_isolated                    # noqa: E402
from sim_adapters import SimArm, make_io                 # noqa: E402
from skilltests import vision_core as vc                 # noqa: E402
from skilltests.button import ButtonMixin                # noqa: E402
from skilltests.geometry import rotate                   # noqa: E402
from skilltests.machine import Machine                   # noqa: E402
from skilltests.parcel_pickup import ParcelPickup        # noqa: E402
import glyphs                                            # noqa: E402
import classroom_spot as spot                            # noqa: E402
from classroom_arm import LOW_LABELS, add_low_cycles, make_classroom_arm   # noqa: E402

WORLD = ss.WORLD
CFG = yaml.safe_load((HERE/'classroom.yaml').read_text())
SCENE = json.loads((GEN/'classroom_scene.json').read_text())
ss.LOG = ROOT/'logs/classroom_demo'
ss.BOXES['classroom'] = {'F1': str(GEN/'classroom_boxes.json')}
ss.NAV_MAPS['F1'] = GEN/'classroom_map.yaml'
PP = WORLD['perception']
SKILL_KEYS = ('press_push_m', 'press_height_tol_m', 'front_stop_m', 'reaim_m', 'press_retries', 'back_off_m', 'line_up_m',
              'slow_v', 'cabin_v', 'press_envelope_m', 'look_back_steps', 'look_back_m', 'confirm_window_s',
              'recheck_max', 'depth_adjust_max_m')
PICK_KEYS = ('scan_dist_m', 'scan_yaws_rad', 'scan_z_m', 'same_parcel_m', 'predock_m', 'face_looks', 'face_z_m',
             'pallet_face_width_m', 'dock_v', 'arm_clear_top_m', 'approach_tol_m', 'undock_m')
TITLES = ['중앙 대기', 'A로 이동', '택배 찾기', '송장 읽기', '택배 싣기', 'B로 이동', '▲ 호출 버튼', '층 버튼 {T}',
          '{T}층 확인', '하차', 'A로 이동', '택배 내려놓기', 'B로 이동', '▼ 호출 버튼', '층 버튼 1', '1층 확인', '하차·복귀']


class StepFailed(RuntimeError):
    pass


class ClassroomStack(ss.Stack):
    """sim_stack.Stack on the generated classroom world and map (single floor at the map origin)."""

    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.robot = GEN/'robot_arm_cam_classroom.urdf'   # arm camera at the classroom resolution (classroom.yaml)

    def guard_map(self, stage):
        pass                                      # generated simulation map (hashed below), not a field map

    def config_hashes(self):
        files = {'nav2_params': self.nav_params_file, 'nav_safety': ROOT/'src/drive_pkg/config/nav_safety.yaml',
                 'drive_calib': ROOT/'src/drive_pkg/config/drive_calib.yaml', 'world_params': REAL/'world_params.yaml',
                 'classroom': HERE/'classroom.yaml', 'robot_urdf': self.robot, 'world': GEN/'classroom.world',
                 'map_yaml': GEN/'classroom_map.yaml', 'map_pgm': GEN/'classroom_map.pgm',
                 'scene': GEN/'classroom_scene.json', 'servo_protocol': ROOT/'src/robot_arm_pkg/robot_arm_pkg/servo_protocol.py'}
        self.config_files = files
        return {k: ss.sha12(v) for k, v in files.items()}

    def start(self):
        assert_isolated()
        node = self.node
        self.result['config_sha256_12'] = self.config_hashes()
        self.result['provenance'] = ss.provenance(self.dir, self.config_files)
        self.processes.start('gazebo', ['gzserver', '--verbose', str(GEN/'classroom.world'),
                                        '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'])
        rsp = self.dir/'rsp.yaml'
        rsp.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
            'use_sim_time': True, 'robot_description': self.robot.read_text()}}}))
        self.processes.start('rsp', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                                     '--ros-args', '--params-file', str(rsp)])
        x, y, yaw = self.spawn[:3]
        spawn = self.processes.start('spawn', ['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', 'packagu_sim',
            '-file', str(self.robot), '-x', str(x), '-y', str(y), '-z', '0.005', '-Y', str(yaw), '-timeout', '90'])
        node.wait(lambda: (node.truth is not None and node.scan is not None) or spawn.poll() not in (None, 0), 120)
        if spawn.poll() not in (None, 0) or node.truth is None or node.scan is None:
            raise RuntimeError('robot spawn failed or no ground truth/scan; see spawn.log')
        if self.gui:
            self.start_gui()
        node.pause(2)
        self.result['truth_spawn_pose'] = node.truth_map()
        self.processes.start('bridge', [sys.executable, str(REAL/'opencr_shim.py'), '--ros-args',
            '--params-file', str(ROOT/'src/drive_pkg/config/drive_calib.yaml'),
            '-p', 'cmd_vel_topic:=/cmd_vel_safe', '-p', 'publish_tf:=false', '-p', 'use_sim_time:=true'])
        self.processes.start('gate', [sys.executable, str(ROOT/'src/drive_pkg/drive_pkg/nav_safety_gate.py'), '--ros-args',
            '--params-file', str(ROOT/'src/drive_pkg/config/nav_safety.yaml'), '-p', 'use_sim_time:=true'])
        self.processes.start('parcels', [sys.executable, str(HERE/'classroom_nodes.py'), 'parcels', '--ros-args',
            '-p', 'use_sim_time:=true', '-p', 'stack_count:=2', '-p', 'carry_mode:=pallet',
            '-p', f"initial_attached:={'true' if getattr(self, 'start_loaded', False) else 'false'}"])
        self.processes.start('elevator', [sys.executable, str(HERE/'classroom_nodes.py'), 'elevator', '--ros-args',
            '-p', 'use_sim_time:=true'])
        self.start_nav()
        if self.speed != 1.0:
            self.set_speed(self.speed)
        # sim_stack.readiness() counts scans appended after len(scan_stamps); the deque is capped at 400, so once it is
        # full (a slow --speed tuning) no new scan is counted. Start the count from an empty deque.
        self.node.scan_stamps.clear()
        self.result['readiness'] = self.readiness()
        (self.dir/'readiness.json').write_text(json.dumps(self.result['readiness'], indent=1, default=str))
        if not self.result['readiness']['ready']:
            raise RuntimeError('readiness check failed: '+json.dumps(self.result['readiness']['failed']))
        if self.gui:
            self.follow_camera()
        return self.result['readiness']


class ClassroomArmLift(ArmLift):
    """ArmLift whose move() does not rely on the 0.2 s hold timer to finish a pose. The joint pose trajectory plugin
    applies the trajectory points at cumulative times (0.1, 0.3, 0.6 s ...), so a 15-point move only gets part way and
    sim/real_maps relies on the hold timer re-sending the target to finish it. When the runner node is busy (camera,
    TF, logs) the timer did not fire for 13 s and the arm stayed half way (2026-10-14, 1x speed). Here the target is
    re-sent explicitly while waiting for the joints."""

    def move(self, pwm_pose, duration_ms):
        target = self.target(pwm_pose)
        current = {n: self.node.joints.get(n, v) if self.node.joints.get(n) is not None else v for n, v in target.items()}
        steps = max(1, int(duration_ms/100))
        self.hold = None
        self._send(list(target), [([current[n]+(target[n]-current[n])*k/steps for n in target],
                                   duration_ms*k/steps/1000.0) for k in range(1, steps+1)])
        tol = float(self.arm['tolerance_rad'])
        self.node.sim_pause(duration_ms/1000)
        self.hold = target
        reached = lambda: all(e is not None and e <= tol for e in self.errors(target).values())   # noqa: E731
        end, ok = time.monotonic()+duration_ms/1000*2+10, False
        while time.monotonic() < end:
            self._send(list(target), [(list(target.values()), .05)])
            if self.node.wait(reached, .5):
                ok = True
                break
        errs = [e for e in self.errors(target).values() if e is not None]
        return {'reached': ok, 'max_err_rad': max(errs) if errs else None}


class ClassroomPickup(ParcelPickup):
    """Shared parcel pickup; the shipping label is read at the predock stance (before_dock hook)."""
    demo = None

    def before_dock(self, face):
        self.demo.label_step(self, face)


class ClassroomB(ButtonMixin, Machine):
    """Wall B: call button, floor button, floor display (shared button procedure; no door, no map switch)."""
    name = 'classroom_b'


# ------------------------------------------------------------------ projector frames
def annotate(bgr, title, boxes=(), lines=(), colour=(60, 220, 200)):
    k = 640.0/bgr.shape[1]                      # boxes are in camera pixels; the projector frame is 640 x 480
    img = cv2.resize(bgr, (640, 480), interpolation=cv2.INTER_AREA if k < 1 else cv2.INTER_NEAREST)
    for (x, y, w, h), text, warn in boxes:
        c = (40, 190, 255) if warn else colour
        cv2.rectangle(img, (int(x*k), int(y*k)), (int((x+w)*k), int((y+h)*k)), c, 2)
        if text:
            cv2.putText(img, text, (int(x*k), max(14, int(y*k)-6)), cv2.FONT_HERSHEY_SIMPLEX, .55, c, 2)
    cv2.rectangle(img, (0, 0), (640, 26), (0, 0, 0), -1)
    cv2.putText(img, title, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, .55, (240, 240, 240), 1)
    y = 480-8-22*(len(lines)-1)
    if lines:
        cv2.rectangle(img, (0, y-20), (640, 480), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (8, y+22*i), cv2.FONT_HERSHEY_SIMPLEX, .55, (40, 190, 255), 2)
    return img


class Demo:
    def __init__(self, stack, args):
        self.stack, self.node, self.args = stack, stack.node, args
        self.P = copy.deepcopy(WORLD)
        self.P['arm']['camera'].update(width=int(CFG['camera']['width']), height=int(CFG['camera']['height']),
                                       update_rate_hz=float(CFG['camera']['update_rate_hz']))
        self.armlift = ClassroomArmLift(stack, self.P)
        self.low = add_low_cycles(self.armlift.sp, self.P['arm'], CFG['boards']['low_row_dz_m'], self.P['arm']['press_cycle'])
        io = make_io(stack, self.armlift, self.P, {})
        arm = make_classroom_arm(SimArm)(self.armlift, io.per, self.P, io.vision)
        arm.low = self.low
        io.arm, io.press_memory = arm, None
        raw_frame, raw_find = io.per.frame, io.vision.find_button
        self.last_frame, self.button_shot = (None, None), None

        def frame(*a, **k):
            out = raw_frame(*a, **k)
            self.last_frame = out
            return out

        def find_button(kind, label, prior, hint=None):
            found = raw_find(kind, label, prior, hint)
            bgr, pose = self.last_frame
            if found is not None and bgr is not None:
                uv = vc.project(tuple(found['xyz']), io.per.size(), io.per.fx, pose)
                caps = io.per.buttons(bgr)
                boxes = [(c['bbox'], None, False) for c in caps]
                if uv is not None and caps:
                    sel = min(caps, key=lambda c: math.hypot(c['u']-uv[0], c['v']-uv[1]))
                    boxes.append((sel['bbox'], f'target {label}', True))
                glyph = {'UP': 'UP', 'DOWN': 'DOWN'}.get(str(label), f'floor {label}')
                note = f'  (label {self.label} -> floor {self.target})' if kind == 'car' and str(label) != '1' else ''
                self.button_shot = (str(label), annotate(bgr, f'button board  (arm camera, line-up look)', boxes,
                                                         [f'target {glyph} found{note}']))
            return found
        io.per.frame, io.vision.find_button = frame, find_button
        # only parcels seen inside the taped storage zone at A count (a brown desk is cardboard-coloured too)
        zone = CFG['parcels']['storage_zone']
        raw_parcels = io.vision.parcels

        def zone_parcels():
            found = raw_parcels()
            keep = []
            for p in found:
                mx, my = io.base.odom_to_map(p['xyz'][:2])
                if zone['x'][0] <= mx <= zone['x'][1] and zone['y'][0] <= my <= zone['y'][1]:
                    keep.append(p)
            self.parcel_filter.append({'seen': len(found), 'in_zone': len(keep)})
            return keep
        io.vision.parcels = zone_parcels
        self.parcel_filter = []
        (stack.dir/'camera').mkdir(exist_ok=True)
        (stack.dir/'projector').mkdir(exist_ok=True)
        io.vision.debug_dir = stack.dir/'camera'
        self.io = io
        self.world_state = None
        self.node.create_subscription(String, '/classroom/state', lambda m: setattr(self, 'world_state', json.loads(m.data)), 10)
        self.tmpl_label = glyphs.templates('label', CFG['parcels']['label'])
        self.tmpl_display = glyphs.templates('display')
        self.rows, self.open = [], None
        self.label = self.target = None
        self.shots = 0
        self.expected_label = args.label

    # ---------------------------------------------------------------- bookkeeping
    def title(self, no):
        return TITLES[no].replace('{T}', str(self.target if self.target else 'T'))

    def begin(self, no):
        self.open = (no, self.stack.begin(f'step_{no:02d}'))
        print(f'[STEP {no:02d}] {self.title(no)} ...', flush=True)

    def close(self, evidence=None, error=None):
        no, seg = self.open
        self.open = None
        row = self.stack.end(seg, 'FAIL' if error else 'PASS')
        row.update({'no': no, 'title': self.title(no), 'evidence': evidence or {}, 'error': error})
        self.rows.append(row)
        wall = row.get('min_wall_m')
        print(f"[STEP {no:02d}] {'FAIL' if error else 'PASS'} {self.title(no)}  sim {row['sim_s']:.1f} s"
              f"  min_wall {wall if wall is None else round(wall, 3)}  {error or ''}", flush=True)
        (self.stack.dir/'steps_progress.json').write_text(json.dumps(self.rows, indent=1, default=str))
        if error:
            raise StepFailed(f'step {no}: {error}')

    def step(self, no, fn):
        self.begin(no)
        try:
            evidence = fn() or {}
        except StepFailed:
            raise
        except Exception as exc:
            self.close(error=f'{type(exc).__name__}: {exc}')
        self.close(evidence)

    def shot(self, name, img):
        self.shots += 1
        path = self.stack.dir/'projector'/f'{self.shots:02d}_{name}.png'
        cv2.imwrite(str(path), img)
        return path.name

    def truth(self):
        t = self.node.truth_map()
        return None if t is None else [round(v, 3) for v in t[1:]]

    # ---------------------------------------------------------------- motion
    def ensure_stowed(self):
        if not self.armlift.stowed():
            r = self.armlift.stow()
            if not r['reached']:
                raise RuntimeError(f'arm not stowed: {r}')

    def nav(self, name, timeout=240):
        goal = tuple(CFG['poses'][name])+('F1',)
        self.ensure_stowed()
        self.stack.update_footprint(self.io.base.loaded())
        status = self.stack.navigate(goal, timeout)
        self.stack.stop()
        t = self.truth()
        err = None if t is None else round(math.hypot(t[0]-goal[0], t[1]-goal[1]), 3)
        if status != 'SUCCEEDED':
            raise RuntimeError(f'Nav2 {status} to {name}')
        return {'goal': name, 'nav2': status, 'true_pose': t, 'true_arrival_err_m': err}

    def turn_right(self, machine, purpose):
        machine.stow()
        machine.enter('TURN_RIGHT_90', purpose=purpose)
        r = self.io.base.turn_by(-math.pi/2)
        machine.require(r.get('status') == 'REACHED', 'turn failed', result=r)
        return r

    # ---------------------------------------------------------------- steps
    def s0(self):
        self.ensure_stowed()
        ready = self.io.lift.ready()
        if not ready.get('ok'):
            raise RuntimeError(f'lift not ready {ready}')
        self.node.sim_pause(1.0)
        return {'start_pose_true': self.truth(), 'lift': ready, 'expected_label': self.expected_label}

    def pickup(self):
        """Steps 2-4 in one run of the shared pickup machine (label read in the before_dock hook)."""
        cfg = {k: PP[k] for k in PICK_KEYS}
        pal = WORLD['parcel']['pallet']
        cfg.update(pallet_depth_m=pal['size_m'][0], fork_centre_ahead_m=pallet_center_ahead(pal),
                   approach_pose_map=list(CFG['poses']['a_approach']), turn_before_scan_rad=0.0)
        m = ClassroomPickup(self.io, cfg, log_path=self.stack.dir/'pickup.jsonl', echo=None)
        m.demo = self
        self.pick_machine = m
        self.begin(2)
        try:
            m.run('steps')
        except StepFailed:
            raise
        except Exception as exc:
            states = ' > '.join(e['state'] for e in m.events[-12:])
            self.close(error=f'{type(exc).__name__}: {exc} | states: {states}')
        ev = {k: v for e in m.events if e['state'] in ('DOCK', 'PICK_UP', 'LIFT_LOAD', 'DONE') for k, v in e.items()
              if k in ('target', 'attach', 'lift', 'load_top_m')}
        ev['states'] = [e['state'] for e in m.events if e['t'] >= self.label_done_t]
        ev['parcel_zone_filter'] = self.parcel_filter
        ev['loaded'] = self.io.base.loaded()
        self.close(ev)

    def label_step(self, m, face):
        """Step 3 at the predock stance: arm camera reads the shipping label (robot data only)."""
        scan = [e for e in m.events if e['state'] in ('SCAN_PARCELS', 'LOOK_PALLET_FACE', 'GO_TO_PREDOCK')]
        parcels = next((e.get('parcels') for e in m.events if e['state'] == 'SCAN_PARCELS'), None)
        self.close({'parcels_seen_xy_y_robot': parcels, 'zone_filter': list(self.parcel_filter),
                    'pallet_face': {k: face[k] for k in ('centre', 'normal')},
                    'states': [e['state'] for e in scan]})
        self.begin(3)
        try:
            ev = self.read_label(m, face)
        except StepFailed:
            raise
        except Exception as exc:
            self.close(error=f'{type(exc).__name__}: {exc}')
        self.close(ev)
        self.label_done_t = self.io.clock.now()
        self.begin(4)

    def read_label(self, m, face):
        io, rd = self.io, CFG['reading']
        pal = WORLD['parcel']['pallet']
        size = pal['parcel']['size_m']
        c, n = face['centre'], face['normal']
        target = (c[0]-n[0]*size[0]/2, c[1]-n[1]*size[0]/2, pal['size_m'][2]+size[2]/2)
        view = SCENE['label_view']['name']
        if view not in self.P['arm']['view_poses']:
            self.P['arm']['view_poses'][view] = SCENE['label_view']['pwm']
        m.enter('READ_LABEL', view=view)
        m.view(view, target)
        need, streak, last, reads = int(rd['label_frames']), 0, None, []
        for k in range(12):
            bgr, pose = io.per.frame(settle=.3)
            if bgr is None:
                reads.append({'frame': k, 'text': None, 'why': 'no fresh frame'})
                streak, last = 0, None
                continue
            uv = vc.project(target, io.per.size(), io.per.fx, pose)
            cv2.imwrite(str(self.stack.dir/'camera'/f'label_raw_{k:02d}.png'), bgr)      # for offline re-reading
            r = glyphs.read_label(bgr, PP['hsv']['parcel'], self.tmpl_label, near_uv=uv)
            ok = r['text'] is not None and all(glyphs.accept(g, rd) for g in r['glyphs'])
            text = r['text'] if ok else None
            streak = streak+1 if text is not None and text == last else (1 if text is not None else 0)
            last = text
            reads.append({'frame': k, 'text': text, 'raw': r['text'], 'why': r.get('why'),
                          'confidence': [g['confidence'] for g in r.get('glyphs', [])]})
            boxes = [((g['bbox']), g['digit'] or '?', i == 0 and text is not None) for i, g in enumerate(r.get('glyphs', []))]
            if 'label_bbox' in r:
                boxes.append((r['label_bbox'], 'label', False))
            floor = int(text[0]) if text and len(text) == 3 and text[0] in rd['valid_first_digits'] else None
            lines = [f'label {text or "?"}  frames {streak}/{need}'] + ([f'first digit {text[0]} -> floor {floor}'] if floor else [])
            if k == 0 or streak >= need:
                self.shot(f'label_{text or "unread"}', annotate(bgr, f'step 3 label  (arm camera)', boxes, lines))
            if streak >= need:
                break
        m.note(reads=reads)
        m.require(streak >= need, f'label not read in {need} consecutive fresh frames', reads=reads[-4:])
        floor = int(last[0]) if len(last) == 3 and last[0] in rd['valid_first_digits'] else None
        m.require(floor is not None, 'label first digit is not a delivery floor (2..4): stop', label=last)
        self.label, self.target = last, floor
        m.note(label=last, target_floor=floor)
        return {'label_read': last, 'target_floor': floor, 'frames': reads, 'expected': self.expected_label,
                'correct': last == self.expected_label}

    def b_machine(self):
        cfg = {k: PP[k] for k in SKILL_KEYS}
        cfg.update(press_view={k: PP['press_view'][k] for k in ('call', 'destination')},
                   press_tip={k: dict(SCENE['press_tips'][k]['top']) for k in ('call', 'destination')},
                   confirm_by={'call': [], 'destination': []}, press_visual_required=True, floor='F1')
        return ClassroomB(self.io, cfg, log_path=self.stack.dir/'b_wall.jsonl', echo=None)

    def press(self, kind, label):
        m = self.b_machine()
        key = 'call' if kind == 'hall' else 'destination'
        row = 'low' if label in LOW_LABELS else 'top'
        m.cfg['press_tip'][key] = dict(SCENE['press_tips'][key][row])
        cap = SCENE['caps'][label]
        before = len((self.world_state or {}).get('presses', []))
        try:
            if kind == 'car':
                m.enter('GO_TO_FLOOR_BOARD_VIEW')
                m.stow()
                m.drive(self.io.base.map_to_odom(tuple(CFG['poses']['b_floor_view'])), reverse='auto',
                        max_v=PP['slow_v']*2)
            button = m.press_button(kind, label, cap['xyz'], 'hall_view' if kind == 'hall' else 'cabin_view', confirm_by=[])
        finally:
            (self.stack.dir/f'press_{label}.json').write_text(json.dumps(m.result(), indent=1, default=str))
        self.node.wait(lambda: len((self.world_state or {}).get('presses', [])) > before, 3)
        world = [p['label'] for p in (self.world_state or {}).get('presses', [])[before:]]
        if self.button_shot and self.button_shot[0] == label:
            name = {'UP': 'UP', 'DOWN': 'DOWN'}.get(label, f'floor_{label}')
            self.shot(f'button_{name}', self.button_shot[1])
            self.button_shot = None
        ev = {'cycle': next((e.get('press', {}).get('cycle') for e in m.events if e['state'] == 'PRESS_BUTTON'), None),
              'tip_row': row, 'robot_confirm': next((e for e in reversed(m.events) if e['state'] == 'CONFIRM_PRESS'), {}),
              'button_estimate': {k: button.get(k) for k in ('xyz', 'normal', 'seen')},
              'world_presses': world, 'states': [e['state'] for e in m.events]}
        if world != [label]:
            raise RuntimeError(f'world registered {world or "no press"} for requested {label}')
        return ev

    def display(self, floor):
        m = self.b_machine()
        io, rd = self.io, CFG['reading']
        try:
            self.turn_right(m, 'face the floor display')
            d = SCENE['display']['centre']
            prior = io.base.map_to_odom((d[0], d[1], 0.0))
            m.enter('DISPLAY_ARM_POSTURE')
            m.view('display_view', (prior[0], prior[1], d[2]))
            need, streak, seen = int(rd['floor_frames']), 0, []
            end = io.clock.now()+float(rd['floor_timeout_s'])
            m.enter('WAIT_TARGET_FLOOR', target=floor, frames=need)
            first = True
            while io.clock.now() < end and streak < need:
                bgr, _ = io.per.frame(settle=.2)
                r = glyphs.read_display(bgr, PP['hsv']['digit'], self.tmpl_display) if bgr is not None else {'digit': None}
                digit = r['digit'] if r.get('digit') and glyphs.accept(r, rd) else None
                changed = not seen or digit != seen[-1]
                streak = streak+1 if digit == str(floor) else 0
                seen.append(digit)
                if bgr is not None and (first or changed or streak >= need):
                    first = False
                    box = [(r['bbox'], f'read {digit}', digit == str(floor))] if r.get('bbox') else []
                    self.shot(f'display_{digit or "none"}_target_{floor}',
                              annotate(bgr, f'floor display  (arm camera)  target {floor}', box,
                                       [f'display {digit or "?"}  confirmed {streak}/{need}']))
                io.clock.sleep(.3)
            m.note(readings=seen[-20:], streak=streak)
            m.require(streak >= need, f'floor {floor} not confirmed in {need} consecutive fresh frames', readings=seen[-10:])
            return {'target': floor, 'readings': seen, 'confirm_frames': streak,
                    'world_display': (self.world_state or {}).get('display_floor'),
                    'world_moves': (self.world_state or {}).get('moves', [])[-4:]}
        finally:
            (self.stack.dir/f'display_{floor}.json').write_text(json.dumps(m.result(), indent=1, default=str))

    def exit_turn(self):
        m = self.b_machine()
        r = self.turn_right(m, 'leave toward A')
        return {'turn': r.get('status'), 'true_pose': self.truth()}

    def drop_spot(self):
        """The empty taped spot of P1 seen by the arm camera (floor tape) + the LiDAR wall ahead -> dock stance in odom.
        None when the tape is not found (the caller falls back to the map pose)."""
        io = self.io
        pal = WORLD['parcel']['pallet']
        p1 = CFG['parcels']['items'][0]['xy']
        prior = io.base.map_to_odom((p1[0], p1[1], 0.0))[:2]
        wall = io.base.wall(0.0)                                   # LiDAR line ahead: (point, normal to robot, fit)
        if wall is None:
            return None, {'why': 'no LiDAR wall ahead'}
        q, n, fit = wall
        heading = math.atan2(-n[1], -n[0])                         # square to the wall
        r = io.arm.view('pallet_view', (prior[0], prior[1], 0.0))
        bgr, pose = io.per.frame(settle=.3)
        self.ensure_stowed()
        if bgr is None or not r.get('reached'):
            return None, {'why': 'tape view not reached', 'view': r}
        cv2.imwrite(str(self.stack.dir/'camera'/'drop_spot_raw.png'), bgr)
        pts = spot.tape_points(bgr, io.per.size(), io.per.fx, pose, vc.on_plane_z)
        margin = .05
        length, width = pal['size_m'][0]+2*margin, pal['size_m'][1]+2*margin
        found = spot.find_spot(pts, prior, heading, length, width, wall=(q, n))
        info = {'tape_points': int(len(pts)), 'wall_fit': fit, 'found': found}
        if found is None:
            return None, info
        c = found['centre']
        ahead = pallet_center_ahead(pal)
        stance = (c[0]-math.cos(heading)*ahead, c[1]-math.sin(heading)*ahead, heading)
        tmx, tmy = io.base.odom_to_map(c)
        info['spot_map_estimate'] = [round(tmx, 3), round(tmy, 3)]
        boxes, lines = [], [f'tape spot found  (map est. {tmx:.2f}, {tmy:.2f})']
        uv = vc.project((c[0], c[1], 0.0), io.per.size(), io.per.fx, pose)
        if uv is not None:
            boxes.append(([uv[0]-30, uv[1]-20, 60, 40], 'drop here', True))
        self.shot('drop_spot', annotate(bgr, 'step 11 drop spot  (arm camera, floor tape)', boxes, lines))
        return stance, info

    def drop(self):
        io = self.io
        pal = WORLD['parcel']['pallet']
        p1 = CFG['parcels']['items'][0]['xy']
        yaw = float(CFG['parcels']['pallet_yaw'])
        target, spot_info = self.drop_spot()
        source = 'floor tape + LiDAR wall'
        if target is None:                                         # fallback: remembered map pose (AMCL)
            source = 'map pose (tape not found)'
            stance_map = (p1[0]+math.cos(yaw)*-pallet_center_ahead(pal), p1[1]+math.sin(yaw)*-pallet_center_ahead(pal), yaw)
            target = io.base.map_to_odom(stance_map)
        r = io.base.go_to(target, max_v=PP['dock_v'], tol=.01, yaw_tol=.015)
        if r.get('status') != 'REACHED':
            raise RuntimeError(f'drop stance not reached {r}')
        low = io.lift.lower()
        if not low.get('reached'):
            raise RuntimeError(f'lift not lowered {low}')
        det = io.load.detach()
        if not det.get('ok'):
            raise RuntimeError(f'parcel not released {det}')
        self.armlift.lift_hold = None
        x, y, yaw_now = io.base.pose()
        dx, dy = rotate((-float(PP['undock_m']), 0.0), yaw_now)
        back = io.base.go_to((x+dx, y+dy, yaw_now), reverse=True, max_v=PP['dock_v'], final_yaw=False)
        parcel = self.node.parcel or {}
        placed = parcel.get('pose')
        err = None if not placed else round(math.hypot(placed[0]-p1[0], placed[1]-p1[1]), 3)
        ev = {'stance_source': source, 'spot': spot_info, 'dock': r.get('status'), 'lift': low, 'release': det,
              'back_off': back.get('status'), 'placed_true_xy': None if not placed else [round(placed[0], 3), round(placed[1], 3)],
              'placement_err_m': err, 'limit_m': 0.10}
        if err is None or err > 0.10:
            raise RuntimeError(f'parcel placed {err} m from its original spot (limit 0.10 m)')
        return ev

    def finish_nav(self):
        # Nav2 accepts a goal within xy_goal_tolerance 0.20 m (field nav2 params) of the AMCL pose; 0.05 m margin for
        # the AMCL error. The end point is where the demo stops, not a functional target.
        ev = self.nav('end')
        ev['limit_m'] = 0.25
        if ev['true_arrival_err_m'] is None or ev['true_arrival_err_m'] > 0.25:
            raise RuntimeError(f"final position {ev['true_arrival_err_m']} m from S (limit 0.25 m)")
        return ev

    # ---------------------------------------------------------------- scenario
    def run(self):
        self.label_done_t = 0.0
        self.step(0, self.s0)
        self.step(1, lambda: self.nav('a_approach'))
        self.pickup()                                            # steps 2, 3, 4
        self.step(5, lambda: self.nav('b_call_view'))
        self.step(6, lambda: self.press('hall', 'UP'))
        self.step(7, lambda: self.press('car', str(self.target)))
        self.step(8, lambda: self.display(self.target))
        self.step(9, self.exit_turn)
        self.step(10, lambda: self.nav('drop_predock'))
        self.step(11, self.drop)
        self.step(12, lambda: self.nav('b_call_view'))
        self.step(13, lambda: self.press('hall', 'DOWN'))
        self.step(14, lambda: self.press('car', '1'))
        self.step(15, lambda: self.display(1))

        def last():
            self.exit_turn()
            return self.finish_nav()
        self.step(16, last)


# ------------------------------------------------------------------ outputs
def trajectory_png(stack, path):
    """Top view: map, props, ground-truth path (evaluation)."""
    s, pad = 120, 40
    x0, x1, y0, y1 = -0.4, 5.4, -0.4, 2.4
    W, H = int((x1-x0)*s)+2*pad, int((y1-y0)*s)+2*pad
    img = np.full((H, W, 3), 250, np.uint8)
    X = lambda x: int(pad+(x-x0)*s)
    Y = lambda y: int(H-pad-(y-y0)*s)
    for xb, yb, sx, sy, _ in json.loads((GEN/'classroom_boxes.json').read_text()):
        cv2.rectangle(img, (X(xb-sx/2), Y(yb+sy/2)), (X(xb+sx/2), Y(yb-sy/2)), (60, 60, 60), -1)
    cv2.rectangle(img, (X(0), Y(2)), (X(5), Y(0)), (0, 200, 255), 1)
    for item in CFG['parcels']['items']:
        x, y = item['xy']
        cv2.rectangle(img, (X(x-.14), Y(y+.2)), (X(x+.14), Y(y-.2)), (90, 140, 70), -1)
        cv2.putText(img, f"{item['name']} {item['label']}", (X(x+.2), Y(y)+5), cv2.FONT_HERSHEY_SIMPLEX, .45, (30, 30, 30), 1)
    for label, cap in SCENE['caps'].items():
        cv2.circle(img, (X(cap['xyz'][0]), Y(cap['xyz'][1])), 3, (200, 80, 0), -1)
    d = SCENE['display']['centre']
    cv2.rectangle(img, (X(d[0]-.15), Y(d[1]+.02)), (X(d[0]+.15), Y(d[1]-.02)), (40, 40, 220), -1)
    pts = [(X(r[2]), Y(r[3])) for r in stack.node.samples[::5]]
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (220, 90, 40), 2)
    for name in ('start', 'a_approach', 'b_call_view', 'b_floor_view'):
        x, y = CFG['poses'][name][:2]
        cv2.drawMarker(img, (X(x), Y(y)), (0, 160, 255), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(img, 'A', (X(-.3), Y(1)), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 0), 2)
    cv2.putText(img, 'B', (X(5.15), Y(1)), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 0), 2)
    cv2.putText(img, 'audience side', (X(2.0), Y(-.3)), cv2.FONT_HERSHEY_SIMPLEX, .45, (90, 90, 90), 1)
    cv2.imwrite(str(path), img)


def contact_sheet(directory, out):
    files = sorted(directory.glob('*.png'))
    if not files:
        return
    tiles = [cv2.resize(cv2.imread(str(f)), (320, 240)) for f in files]
    cols = 4
    while len(tiles) % cols:
        tiles.append(np.full((240, 320, 3), 255, np.uint8))
    rows = [np.hstack(tiles[i:i+cols]) for i in range(0, len(tiles), cols)]
    cv2.imwrite(str(out), np.vstack(rows))


NAV_STEPS = (1, 5, 10, 12, 16)          # Nav2 legs: body-to-wall >= 0.15 m; skill stages (docking, pressing) >= 0.05 m


def wall_check(rows):
    """Minimum ground-truth body-to-wall distance per step against the E2E criteria (sim/real_maps run_e2e)."""
    out = []
    for r in rows:
        mw = r.get('min_wall_m')
        limit = .15 if r['no'] in NAV_STEPS else .05
        if mw is not None and mw < limit:
            out.append({'no': r['no'], 'min_wall_m': round(mw, 3), 'limit_m': limit})
    return out


def write_report(stack, demo, run_info):
    rows = demo.rows
    passed = [r for r in rows if r['status'] == 'PASS']
    world = demo.world_state or {}
    summary = {'run': stack.name, 'validation_scope': 'simulation (assumed dimensions)', **run_info,
               'steps_passed': len(passed), 'steps_total': 17, 'completed': len(passed) == 17,
               'label_expected': demo.expected_label, 'label_read': demo.label, 'target_floor': demo.target,
               'world_button_presses': [p['label'] for p in world.get('presses', [])],
               'expected_presses': ['UP', str(demo.target), 'DOWN', '1'] if demo.target else None,
               'display_final': world.get('display_floor'),
               'min_wall_m': min((r['min_wall_m'] for r in rows if r.get('min_wall_m') is not None), default=None),
               'sim_total_s': round(sum(r['sim_s'] for r in rows), 1), 'wall_total_s': round(sum(r['wall_s'] for r in rows), 1),
               'low_row_cycles_sim_ik': demo.low, 'label_view': SCENE['label_view']['name'],
               'wall_violations': wall_check(rows)}
    (stack.dir/'summary.json').write_text(json.dumps(summary, indent=1, default=str))
    (stack.dir/'steps.json').write_text(json.dumps(rows, indent=1, default=str))
    with open(stack.dir/'steps.csv', 'w', newline='') as h:
        w = csv.writer(h)
        w.writerow(['no', 'title', 'status', 'sim_s', 'wall_s', 'min_wall_m', 'error'])
        for r in rows:
            w.writerow([r['no'], r['title'], r['status'], round(r['sim_s'], 1), round(r['wall_s'], 1),
                        None if r.get('min_wall_m') is None else round(r['min_wall_m'], 3), r.get('error') or ''])
    lines = [f"# 강의실 시연 시뮬 결과 — {stack.name}", '',
             f"- 범위: 시뮬레이션(가정 치수). 송장 {demo.expected_label} → 판독 {demo.label} → 목표층 {demo.target}",
             f"- 단계 {len(passed)}/17 통과, 시뮬 시간 {summary['sim_total_s']} s, 실제 시간 {summary['wall_total_s']} s",
             f"- 월드에 등록된 버튼 누름: {summary['world_button_presses']} (기대 {summary['expected_presses']})",
             f"- 최소 벽 거리(몸체 외곽, 참값): {summary['min_wall_m']} · 기준 위반(주행 0.15 m, 스킬 0.05 m): "
             f"{summary['wall_violations'] or '없음'}", '',
             '| # | 단계 | 결과 | 시뮬 s | 최소 벽 거리 m | 비고 |', '|---|---|---|---|---|---|']
    for r in rows:
        ev = r.get('evidence') or {}
        note = r.get('error') or ''
        if not note:
            for key in ('label_read', 'world_presses', 'readings', 'placement_err_m', 'true_arrival_err_m'):
                if key in ev:
                    val = ev[key]
                    if key == 'readings':
                        val = ''.join(str(v) if v else '-' for v in val)[-24:]
                    note = f'{key} {val}'
                    break
        mw = r.get('min_wall_m')
        lines.append(f"| {r['no']} | {r['title']} | {r['status']} | {r['sim_s']:.1f} | "
                     f"{'' if mw is None else round(mw, 3)} | {note} |")
    (stack.dir/'report.md').write_text('\n'.join(lines)+'\n')
    return summary


def run_once(args, index):
    name = f"classroom_{args.label}_{time.strftime('%Y%m%d_%H%M%S')}" + (f'_r{index}' if args.repeats > 1 else '')
    stack = ClassroomStack(name, world='classroom', floor='F1', params=args.params,
                           spawn=tuple(CFG['poses']['start'])+('F1',), arm=True, speed=args.speed, parcels=2,
                           gui=args.gui, load='pallet', record=not args.no_record)
    (stack.dir/'classroom_scene.json').write_text((GEN/'classroom_scene.json').read_text())
    demo, info = None, {'label_arg': args.label, 'speed': args.speed}
    try:
        print(f'[run] {name}: starting stack', flush=True)
        info['readiness'] = stack.start()
        demo = Demo(stack, args)
        demo.run()
    except StepFailed as exc:
        info['failed'] = str(exc)
    except Exception as exc:
        info['failed'] = f'{type(exc).__name__}: {exc}'
        print(f'[run] ERROR {info["failed"]}', flush=True)
    finally:
        summary = None
        if demo is not None:
            if demo.open is not None:
                demo.rows.append({'no': demo.open[0], 'title': demo.title(demo.open[0]), 'status': 'FAIL',
                                  'sim_s': 0.0, 'wall_s': 0.0, 'error': info.get('failed')})
            try:
                trajectory_png(stack, stack.dir/'trajectory.png')
                contact_sheet(stack.dir/'projector', stack.dir/'projector_sheet.png')
            except Exception as exc:
                info['figure_error'] = str(exc)
            summary = write_report(stack, demo, info)
        stack.result['classroom'] = summary or info
        stack.finish(preserve_scene=bool(info.get('failed')) and args.keep_failed)
    print(f'[run] RESULT {name}: ' + (f"{summary['steps_passed']}/17 steps" if summary else 'no steps')
          + (f"  FAILED {info['failed']}" if info.get('failed') else '  COMPLETED'), flush=True)
    print(f'[run] report: logs/classroom_demo/{name}/report.md', flush=True)
    return summary and summary['completed']


def main():
    p = argparse.ArgumentParser(description='11/4 classroom demo simulation (one command)')
    p.add_argument('--label', default=SCENE['parcels']['items'][0]['label'], help='P1 shipping label (3 digits)')
    p.add_argument('--speed', type=ss.parse_speed, default=1.0,
                   help='target real-time factor or max (default 1 = real time; faster runs starve the runner node)')
    p.add_argument('--params', default='P0', choices=sorted(ss.NAV_PARAMS))
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--gui', action='store_true', help='gzclient + RViz + arm camera window (observation only)')
    p.add_argument('--no-record', action='store_true', help='skip the rosbag')
    p.add_argument('--keep-failed', action='store_true', help='pause Gazebo and keep the scene after a failure')
    args = p.parse_args()
    if args.label != SCENE['parcels']['items'][0]['label']:
        raise SystemExit('scene label mismatch (generate_classroom.py --p1-label)')
    rclpy.init()
    results = []
    try:
        for k in range(args.repeats):
            results.append(bool(run_once(args, k+1)))
    finally:
        rclpy.try_shutdown()
    print(f'[run] {sum(results)}/{len(results)} runs completed all 17 steps', flush=True)
    return 0 if results and all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
