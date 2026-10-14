"""Simulation-only arm (4-DOF) and lift actions for run_e2e.py --arm-sim.

- Lift: reuses test_workspace/gazebo_world_swap/scripts/lift_cycle.py unchanged (lift_joint via
  /set_lift_trajectory, completion = /joint_states position within tolerance).
- Arm: PWM poses and press sequences are READ from src/robot_arm_pkg/robot_arm_pkg/servo_protocol.py
  (the field SSOT; no serial port is opened). PWM -> joint angle uses the assumed mapping in
  world_params.yaml arm.pwm_to_angle ([가정값]); completion = /joint_states within tolerance.
- Load: attach/detach mock via parcel_mock.py (no contact / load validation). 'pallet' (default): the carrier
  enters the floor pallet pocket at lift 0, attach, raise and hold while driving; unload = lower to 0, detach.
  Carry height (world_params lift.carry, --lift-stop): 'ultrasonic' raises until the mast-front ultrasonic sees
  the parcel top (so the parcel stays below the arm and its camera), 'fixed' raises only fixed_lift_m.
  The ultrasonic stop needs an interruptible move, so it publishes /set_lift_trajectory itself (same message
  as lift_cycle.py); fixed and lowering moves still use lift_cycle.py. 'box' (10/8): lift up, attach/detach at
  the shelf height, lift down.
Button "press" is a taught joint-space pose with base yaw and wrist roll locked forward.
The world checks geometric arm-tip/button contact; force and actual switch mechanics are not modelled.
"""
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

import arm_geometry as ag
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: F401

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LIFT_CYCLE = ROOT/'test_workspace/gazebo_world_swap/scripts/lift_cycle.py'
CAMERA_VIEWS = ROOT/'src/robot_arm_pkg/config/camera_views.json'   # Kim's taught camera poses (read only)
INDICATORS = HERE/'generated/elevator_indicators.json'


def servo_protocol():
    return ag.servo_protocol(ROOT)


def rot(axis, angle):
    x, y, z = np.asarray(axis, float)/np.linalg.norm(axis)
    c, s, C = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s], [y*x*C+z*s, c+y*y*C, y*z*C-x*s], [z*x*C-y*s, z*y*C+x*s, c+z*z*C]])


class ArmLift:
    def __init__(self, stack, params):
        self.stack, self.node = stack, stack.node
        self.params, self.arm, self.lift_p = params, params['arm'], params['lift']
        self.sp = servo_protocol()
        self.names = {j['servo']: j['name'] for j in self.arm['joints']}
        self.traj_pub = self.node.create_publisher(JointTrajectory, '/set_arm_trajectory', 10)
        self.parcel_pub = self.node.create_publisher(String, '/sim/parcel/command', 10)
        self.lift_pub = self.node.create_publisher(JointTrajectory, '/set_lift_trajectory', 10)
        self.pause_pub = self.node.create_publisher(Bool, '/sim/arm_pose_writes_paused', 10)
        self.pose_writes_until = 0.0
        self.load_mode = getattr(stack, 'load', 'box')
        self.lift_hold = None   # carry height re-sent like the arm hold (the plugin releases the joint)
        self.lift_stop = getattr(stack, 'lift_stop', params['lift']['carry']['mode'])
        self.us_range = None
        self.node.create_subscription(Range, '/sim/ultrasonic_front',
                                      lambda m: setattr(self, 'us_range', m.range), 10)
        # Servo hold: gazebo_ros_joint_pose_trajectory releases joints after its last point and they
        # drift (~0.02 rad/s observed). A real hobby servo actively holds its commanded position, so the
        # last commanded target is re-sent every 0.2 s while the runner spins.
        self.hold = None
        self.node.create_timer(.2, self._hold)
        self.hold = self.target(self.sp.HOME)   # power-on: hold the stow pose

    def _send(self, names, points):
        traj = JointTrajectory()
        traj.header.frame_id = 'world'   # joint-only mode (lift_cycle.py 2026-08-21 finding)
        traj.joint_names = names
        for positions, t in points:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            point.time_from_start = Duration(sec=int(t), nanosec=int((t-int(t))*1e9))
            traj.points.append(point)
        self.traj_pub.publish(traj)
        self.pose_writes_until = max(self.pose_writes_until, self.node.now()+points[-1][1])

    def _hold(self):
        pausing = (self.node.elevator or {}).get('transferring', False)
        self.pause_pub.publish(Bool(data=bool(pausing and self.node.now() >= self.pose_writes_until+.3)))
        if pausing:
            return  # let Gazebo relocate the cabin without kinematic joint pose writes
        if self.hold:
            self._send(list(self.hold), [(list(self.hold.values()), .05)])
        if self.lift_hold is not None:
            self._lift([(self.lift_hold, .05)])

    def _lift(self, points):
        traj = JointTrajectory()
        traj.header.frame_id = 'world'
        traj.joint_names = [self.lift_p['joint']]
        for position, t in points:
            point = JointTrajectoryPoint()
            point.positions = [float(position)]
            point.time_from_start = Duration(sec=int(t), nanosec=int((t-int(t))*1e9))
            traj.points.append(point)
        self.lift_pub.publish(traj)
        self.pose_writes_until = max(self.pose_writes_until, self.node.now()+points[-1][1])

    def carry_up(self):
        """Raise the loaded pallet to its carry height. The load top (measured lift position + load height above
        the fork, robot knowledge) is capped BEFORE every command at perception.arm_clear_top_m, so the parcel never
        rises in front of the arm or its camera. ultrasonic: stop when the mast-front sensor sees the parcel top;
        fixed: a small fixed lift (clamped to the cap). Overall deadline lift.timeout_s (sim seconds)."""
        carry = self.lift_p['carry']
        offset = float(carry['load_top_offset_m'])
        cap = float(self.params['perception']['arm_clear_top_m'])
        limit = min(float(carry['max_lift_m']), cap-offset)
        self.lift_hold = None
        if self.lift_stop == 'fixed':
            target = min(float(carry['fixed_lift_m']), limit)
            out = self.lift_to(target, 'load_carry')
            pos = out.get('lift_pos_m')
            self.lift_hold = target if out['reached'] else None
            return out | {'mode': 'fixed', 'cap_m': cap, 'load_top_m': None if pos is None else pos+offset}
        us = self.lift_p['ultrasonic']
        detect, step, speed = float(us['detect_range_m']), .1, float(carry['speed_mps'])
        start = self.node.joints.get(self.lift_p['joint'])
        if start is None:
            return {'mode': 'ultrasonic', 'reached': False, 'why': 'no lift position feedback'}
        deadline = self.node.now()+float(self.lift_p['timeout_s'])
        # single-point commands stepped up like lift_cycle.py (one point per message), stopped by the ultrasonic
        cmd, seen, why = start, False, None
        while True:
            if self.node.now() > deadline:
                why = 'deadline'
                break
            if cmd+speed*step > limit:
                why = 'load-top cap reached before the ultrasonic saw the parcel'
                break
            cmd += speed*step
            self._lift([(cmd, step)])
            self.node.sim_pause(step)
            if self.us_range is not None and self.us_range <= detect:
                seen = True
                break
        stop = self.node.joints.get(self.lift_p['joint'])
        self.lift_hold = stop                      # hold where it stopped
        self._lift([(stop, .05)])
        self.node.sim_pause(1.0)
        pos = self.node.joints.get(self.lift_p['joint'])
        stable = pos is not None and stop is not None and abs(pos-stop) <= float(carry['hold_tolerance_m'])
        top = None if pos is None else pos+offset
        return {'mode': 'ultrasonic', 'detected': seen, 'range_m': self.us_range, 'stop_lift_m': stop,
                'lift_pos_m': pos, 'sensor_z_m': us['mount_xyz'][2], 'cap_m': cap, 'why': why,
                'load_top_m': top, 'reached': bool(seen and stable and top is not None and top <= cap)}

    # ---------------------------------------------------------------- arm
    def rad(self, servo, pwm):
        return ag.pwm_angle(self.arm, servo, pwm)

    def target(self, pwm_pose):
        for servo, pwm in pwm_pose.items():
            self.sp.check_pwm(servo, pwm)
        return {self.names[s]: self.rad(s, p) for s, p in pwm_pose.items()}

    def errors(self, target):
        return {n: (None if self.node.joints.get(n) is None else abs(self.node.joints[n]-v)) for n, v in target.items()}

    def tip_map(self):
        """Forward kinematics of the assumed arm in the current floor's map frame (for the record only)."""
        truth = self.node.truth_map()
        if truth is None or any(self.node.joints.get(n) is None for n in self.names.values()):
            return None
        _, x, y, yaw = truth
        R = rot([0, 0, 1], yaw)
        p = np.array([x, y, 0.0])+R@np.array(self.arm['mount_xyz'])
        for spec in self.arm['joints']:
            p = p+R@np.array(spec['parent_offset'])
            R = R@rot(spec['axis'], self.node.joints[spec['name']])
        p = p+R@np.array(self.arm['tip_offset'])
        return [round(float(v), 3) for v in p]

    def camera_pose_map(self):
        """Arm-tip camera position and rotation in the current floor's map frame ([가정값] arm geometry)."""
        truth = self.node.truth_map()
        if truth is None or any(self.node.joints.get(n) is None for n in self.names.values()):
            return None
        _, x, y, yaw = truth
        R = rot([0, 0, 1], yaw)
        p = np.array([x, y, 0.0])+R@np.array(self.arm['mount_xyz'])
        for spec in self.arm['joints']:
            p = p+R@np.array(spec['parent_offset'])
            R = R@rot(spec['axis'], self.node.joints[spec['name']])
        p = p+R@np.array(self.arm['tip_offset'])
        cam = self.arm['camera']
        p = p+R@np.array(cam['xyz'])
        for axis, angle in zip(([1, 0, 0], [0, 1, 0], [0, 0, 1]), cam['rpy']):
            R = R@rot(axis, angle)
        return p, R

    def look(self, view, floor):
        """Move to a taught camera view (camera_views.json) and check whether the floor display lies inside the
        assumed camera field of view (geometry only; the image is not processed)."""
        views = json.loads(CAMERA_VIEWS.read_text())
        out = {'view': view, **self.move(views[view], int(views.get('duration_ms', 2000)))}
        self.node.sim_pause(1.0)
        pose = self.camera_pose_map()
        if pose is None or not INDICATORS.exists():
            return out | {'display_in_fov': None}
        p, R = pose
        target = np.array(json.loads(INDICATORS.read_text())['floors'][floor]['display_map']['xyz'])
        v = R.T@(target-p)
        cam = self.arm['camera']
        hfov = float(cam['hfov_rad'])
        vfov = 2*math.atan(math.tan(hfov/2)*cam['height']/cam['width'])
        bearing, elevation = math.atan2(v[1], v[0]), math.atan2(v[2], math.hypot(v[0], v[1]))
        return out | {'camera_map': [round(float(c), 3) for c in p], 'display_dist_m': round(float(np.linalg.norm(v)), 3),
                      'display_bearing_deg': round(math.degrees(bearing), 1),
                      'display_elevation_deg': round(math.degrees(elevation), 1),
                      'display_in_fov': bool(v[0] > 0 and abs(bearing) <= hfov/2 and abs(elevation) <= vfov/2)}

    def move(self, pwm_pose, duration_ms):
        if not self.node.wait(lambda: not (self.node.elevator or {}).get('transferring', False), 15):
            raise RuntimeError('arm pose write blocked by unfinished cabin transfer')
        target = self.target(pwm_pose)
        current = {n: self.node.joints.get(n, v) if self.node.joints.get(n) is not None else v for n, v in target.items()}
        steps = max(1, int(duration_ms/100))
        self.hold = None
        self._send(list(target), [([current[n]+(target[n]-current[n])*k/steps for n in target],
                                   duration_ms*k/steps/1000.0) for k in range(1, steps+1)])
        tol = float(self.arm['tolerance_rad'])
        self.node.sim_pause(duration_ms/1000)
        self.hold = target
        ok = self.node.wait(lambda: all(e is not None and e <= tol for e in self.errors(target).values()),
                            duration_ms/1000*2+10)
        errs = [e for e in self.errors(target).values() if e is not None]
        return {'reached': ok, 'max_err_rad': max(errs) if errs else None}

    def stowed(self):
        target = self.target(self.sp.HOME)
        return all(e is not None and e <= float(self.arm['tolerance_rad']) for e in self.errors(target).values())

    def stow(self):
        return self.move(self.sp.HOME, self.sp.HOMING_DURATION_MS)

    def press_cycle(self, number, on_press=None):
        """Run servo_protocol.PRESS_CYCLES[number]; on_press() fires once during the press hold step."""
        poses = self.sp.POSE_TABLES[number]
        rows, pressed = [], False
        for pose, duration_ms, send in self.sp.PRESS_CYCLES[number]:
            if send:
                result = self.move(poses[pose], duration_ms)
            else:
                self.node.sim_pause(duration_ms/1000)
                result = {'reached': all(e is not None and e <= float(self.arm['tolerance_rad'])
                                         for e in self.errors(self.target(poses[pose])).values()),
                          'max_err_rad': None}
                if on_press and not pressed and result['reached']:
                    on_press()
                    pressed = True
            rows.append({'pose': pose, 'duration_ms': duration_ms, 'send': send, **result, 'tip_map': self.tip_map()})
            if not result['reached']:
                break
        return {'cycle': number, 'label': self.sp.CYCLE_LABELS.get(number), 'steps': rows, 'pressed': pressed,
                'stowed': self.stowed(), 'completed': all(r['reached'] for r in rows) and len(rows) ==
                len(self.sp.PRESS_CYCLES[number])}

    # ---------------------------------------------------------------- lift + parcel
    def lift_to(self, target, tag):
        tol, timeout = self.lift_p['tolerance_m'], self.lift_p['timeout_s']
        proc = subprocess.run([sys.executable, str(LIFT_CYCLE), '--tag', tag, '--up', str(target), '--down',
                               str(target), '--tol', str(tol), '--timeout', str(timeout)],
                              capture_output=True, text=True, timeout=timeout*3+30)
        (self.stack.dir/f'lift_{tag}.log').write_text(proc.stdout+proc.stderr)
        self.node.pause(.5)
        pos = self.node.joints.get(self.lift_p['joint'])
        return {'lift_cycle_exit': proc.returncode, 'lift_pos_m': pos,
                'reached': proc.returncode == 0 and pos is not None and abs(pos-target) <= tol}

    def parcel(self, cmd, expect):
        self.parcel_pub.publish(String(data=json.dumps({'cmd': cmd})))
        ok = self.node.wait(lambda: (self.node.parcel or {}).get('state') == expect, 10)
        return {'parcel_cmd': cmd, 'parcel_state': (self.node.parcel or {}).get('state'), 'parcel_ok': ok,
                'attach_gap_m': (self.node.parcel or {}).get('attach_gap_m')}

    def load_unload(self, kind):
        if self.load_mode == 'pallet':
            return self.pallet_load_unload(kind)
        return self.box_load_unload(kind)

    def pallet_load_unload(self, kind):
        """'load': carrier in the pocket at lift 0 -> attach -> lift to carry height (held while driving);
        'unload': lower to lift 0 (pallet on the floor) -> detach."""
        if kind == 'load':
            out = {'parcel': self.parcel('attach', 'attached')}
            if not out['parcel']['parcel_ok']:
                return out | {'completed': False}
            out['up'] = self.carry_up()
            if out['up']['mode'] == 'fixed':
                self.lift_hold = float(self.lift_p['carry']['fixed_lift_m']) if out['up']['reached'] else None
            self.node.pause(.5)
            out['up']['load_top_m'] = (self.node.parcel or {}).get('load_top_m')
            return out | {'completed': out['up']['reached']}
        self.lift_hold = None
        out = {'down': self.lift_to(self.lift_p['down_m'], 'unload_down')}
        if not out['down']['reached']:
            return out | {'completed': False}
        out['parcel'] = self.parcel('detach', 'placed')
        return out | {'completed': out['parcel']['parcel_ok']}

    def box_load_unload(self, kind):
        """kind 'load': lift up -> attach -> lift down; 'unload': lift up -> detach -> lift down."""
        up = self.lift_p['load_up_m'] if kind == 'load' else self.lift_p['unload_up_m']
        out = {'up': self.lift_to(up, f'{kind}_up')}
        if not out['up']['reached']:
            return out | {'completed': False}
        out['parcel'] = self.parcel('attach' if kind == 'load' else 'detach', 'attached' if kind == 'load' else 'placed')
        out['down'] = self.lift_to(self.lift_p['down_m'], f'{kind}_down')
        out['completed'] = out['parcel']['parcel_ok'] and out['down']['reached']
        return out
