"""Arm pieces for the classroom demo simulation.

Low-row press cycles: the taught cycles (src/robot_arm_pkg servo_protocol.py, the Jetson's angles) put the arm tip at
the top row of the demo boards (call board UP, floor buttons 1 2). The bottom row (DOWN, 3 4) is 8 cm lower, so the
simulation adds one cycle per board: the press pose lowered by that height at the same reach, entered and left
STRAIGHT along the press direction from a point `standoff_m` back at the same height (inverse kinematics, assumed
arm geometry). Simply lowering every pose of the taught cycle is wrong: its retreat rises while still at the board
and swept the button above (2026-10-14 sim: pressing 4 also registered 2 twice). [시뮬 가정] The real arm needs
these poses TAUGHT (Kim) with the same straight-out retreat; servo_protocol.py is not modified.

ClassroomArm: SimArm whose press() runs the cycle of the button's row instead of one cycle per button kind.
"""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]/'sim/real_maps'))
import arm_geometry as ag   # noqa: E402

LOW_LABELS = ('DOWN', '3', '4')
KEEP = ('home', 'press_ready')


def _ik_all(arm, target):
    """Both planar solutions (shoulder/elbow, yaw toward the target, wrist roll 0) within the joint limits."""
    n = ag.names(arm)
    lim = ag.limits(arm)
    j = arm['joints']
    mount = np.array(arm['mount_xyz'], float)+np.array(j[0]['parent_offset'], float)+np.array(j[1]['parent_offset'], float)
    l1 = float(j[2]['parent_offset'][0])
    l2 = float(j[3]['parent_offset'][0])+float(arm['tip_offset'][0])
    d = np.asarray(target, float)-mount
    yaw = math.atan2(d[1], d[0])
    r, h = math.hypot(d[0], d[1]), d[2]
    c2 = (r*r+h*h-l1*l1-l2*l2)/(2*l1*l2)
    if abs(c2) > 1:
        return []
    out = []
    for phi2 in (math.acos(c2), -math.acos(c2)):
        phi1 = math.atan2(h, r)-math.atan2(l2*math.sin(phi2), l1+l2*math.cos(phi2))
        angles = {n[0]: yaw, n[1]: -phi1, n[2]: -phi2, n[3]: 0.0}
        if all(lim[k][0] <= v <= lim[k][1] for k, v in angles.items()):
            out.append(angles)
    return out


def lowered_pose(arm, pwm, dz, back=0.0):
    """PWM pose whose tip is dz lower (and `back` m farther from the button along the tip's forward direction) than
    the tip of `pwm` (solution nearest the original angles)."""
    angles = {s['name']: ag.pwm_angle(arm, s['servo'], pwm[s['servo']]) for s in arm['joints']}
    tip, R, _, _ = ag.fk(arm, angles)
    fwd = np.array([R[0, 0], R[1, 0], 0.0])
    fwd = fwd/np.linalg.norm(fwd)
    sols = _ik_all(arm, tip-np.array([0.0, 0.0, dz])-back*fwd)
    if not sols:
        raise ValueError(f'tip {tip.round(3).tolist()} cannot be lowered by {dz} m')
    best = min(sols, key=lambda a: sum((a[k]-angles[k])**2 for k in angles))
    out = ag.to_pwm(arm, best)
    lo, hi = arm['pwm_limits_us']
    if not all(lo <= v <= hi for v in out.values()):
        raise ValueError(f'lowered pose outside PWM limits: {out}')
    out['000'], out['003'] = pwm['000'], pwm['003']    # base yaw / wrist roll stay forward as taught
    return out


def add_low_cycles(sp, arm, dz, base_cycles, standoff_m=0.08):
    """Register one low-row cycle per button kind in the loaded servo_protocol module `sp` (in memory only):
    press_ready -> low_standoff (standoff_m back, button height) -> low_press -> hold -> low_standoff -> press_ready
    -> home. Returns {'call': n, 'destination': m}."""
    added = {}
    for key in ('call', 'destination'):
        base = int(base_cycles[key])
        cycle = sp.PRESS_CYCLES[base]
        hold = next(pose for pose, _, send in cycle if not send)
        hold_ms = next(ms for pose, ms, send in cycle if not send)
        press_ms = next(ms for pose, ms, send in cycle if send and pose == hold)
        number = max(sp.PRESS_CYCLES)+1
        press = sp.POSE_TABLES[base][hold]
        table = {name: dict(sp.POSE_TABLES[base][name]) for name in KEEP}
        table['low_standoff'] = lowered_pose(arm, press, float(dz[key]), back=standoff_m)
        table['low_press'] = lowered_pose(arm, press, float(dz[key]))
        sp.POSE_TABLES[number] = table
        sp.PRESS_CYCLES[number] = (('press_ready', 2000, True), ('low_standoff', 1500, True), ('low_press', press_ms, True),
                                   ('low_press', hold_ms, False), ('low_standoff', 800, True), ('press_ready', 1500, True),
                                   ('home', 1500, True))
        sp.CYCLE_LABELS[number] = f'{sp.CYCLE_LABELS.get(base, base)}_low_row_sim_ik'
        added[key] = number
    return added


def cycle_for(label, kind, arm, low):
    key = 'call' if kind == 'hall' else 'destination'
    return low[key] if str(label) in LOW_LABELS else int(arm['press_cycle'][key])


def make_classroom_arm(SimArm):
    class ClassroomArm(SimArm):
        low = {}

        def press(self, button):
            number = cycle_for(button.get('label'), button.get('kind'), self.P['arm'], self.low)
            out = self.a.press_cycle(number)
            return {'camera_confirmation': {'label': button.get('label'), 'lit': False, 'source': 'none (no lamp)'},
                    'done': bool(out['completed']), 'cycle': number, 'label': out.get('label'),
                    'steps': [{k: row.get(k) for k in ('pose', 'reached', 'tip_map')} for row in out['steps']]}
    return ClassroomArm
