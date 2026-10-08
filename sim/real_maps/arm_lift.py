"""Simulation-only arm (4-DOF) and lift actions for run_e2e.py --arm-sim.

- Lift: reuses test_workspace/gazebo_world_swap/scripts/lift_cycle.py unchanged (lift_joint via
  /set_lift_trajectory, completion = /joint_states position within tolerance).
- Arm: PWM poses and press sequences are READ from src/robot_arm_pkg/robot_arm_pkg/servo_protocol.py
  (the field SSOT; no serial port is opened). PWM -> joint angle uses the assumed mapping in
  world_params.yaml arm.pwm_to_angle ([가정값]); completion = /joint_states within tolerance.
- Parcel: attach/detach mock via parcel_mock.py (no contact / load validation).
Button "press" is a joint-space pose reached at an assumed arm geometry; no button contact is modelled.
"""
import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: F401

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
LIFT_CYCLE = ROOT/'test_workspace/gazebo_world_swap/scripts/lift_cycle.py'


def servo_protocol():
    path = ROOT/'src/robot_arm_pkg/robot_arm_pkg/servo_protocol.py'
    spec = importlib.util.spec_from_file_location('field_servo_protocol', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rot(axis, angle):
    x, y, z = np.asarray(axis, float)/np.linalg.norm(axis)
    c, s, C = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s], [y*x*C+z*s, c+y*y*C, y*z*C-x*s], [z*x*C-y*s, z*y*C+x*s, c+z*z*C]])


class ArmLift:
    def __init__(self, stack, params):
        self.stack, self.node = stack, stack.node
        self.arm, self.lift_p = params['arm'], params['lift']
        self.sp = servo_protocol()
        self.names = {j['servo']: j['name'] for j in self.arm['joints']}
        self.traj_pub = self.node.create_publisher(JointTrajectory, '/set_arm_trajectory', 10)
        self.parcel_pub = self.node.create_publisher(String, '/sim/parcel/command', 10)
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

    def _hold(self):
        if self.hold:
            self._send(list(self.hold), [(list(self.hold.values()), .05)])

    # ---------------------------------------------------------------- arm
    def rad(self, servo, pwm):
        conv = self.arm['pwm_to_angle']
        return conv['sign'][servo]*(pwm-conv['center_us'])*conv['rad_per_us']

    def target(self, pwm_pose):
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
        """kind 'load': lift up -> attach -> lift down; 'unload': lift up -> detach -> lift down."""
        up = self.lift_p['load_up_m'] if kind == 'load' else self.lift_p['unload_up_m']
        out = {'up': self.lift_to(up, f'{kind}_up')}
        if not out['up']['reached']:
            return out | {'completed': False}
        out['parcel'] = self.parcel('attach' if kind == 'load' else 'detach', 'attached' if kind == 'load' else 'placed')
        out['down'] = self.lift_to(self.lift_p['down_m'], f'{kind}_down')
        out['completed'] = out['parcel']['parcel_ok'] and out['down']['reached']
        return out
