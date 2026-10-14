"""Assumed 4-DOF arm geometry shared by the robot side (arm_lift / perception) and the world side (sim_elevator button
contact). Every number comes from world_params.yaml `arm` ([가정값]: joint axes, link lengths, PWM->angle).

Joints: 000 base yaw (z), 001 shoulder pitch (y, +down), 002 elbow pitch (y, +down), 003 wrist roll (x).
The arm-tip camera looks along the tip +x axis (image right = tip -y, image down = tip -z).
"""
import importlib.util
import math
from pathlib import Path

import numpy as np


def rot(axis, angle):
    x, y, z = np.asarray(axis, float)/np.linalg.norm(axis)
    c, s, C = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    return np.array([[c+x*x*C, x*y*C-z*s, x*z*C+y*s], [y*x*C+z*s, c+y*y*C, y*z*C-x*s], [z*x*C-y*s, z*y*C+x*s, c+z*z*C]])


def fk(arm, angles, base=(0.0, 0.0, 0.0, 0.0)):
    """Tip and camera poses for joint angles {name: rad} with the robot base at (x, y, z, yaw).
    Returns (tip_xyz, tip_R, cam_xyz, cam_R)."""
    x, y, z, yaw = base
    R = rot([0, 0, 1], yaw)
    p = np.array([x, y, z])+R@np.array(arm['mount_xyz'], float)
    for spec in arm['joints']:
        p = p+R@np.array(spec['parent_offset'], float)
        R = R@rot(spec['axis'], angles[spec['name']])
    tip_p, tip_R = p+R@np.array(arm['tip_offset'], float), R
    cam = arm.get('camera', {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]})
    cam_p, cam_R = tip_p+tip_R@np.array(cam['xyz'], float), tip_R
    for axis, angle in zip(([1, 0, 0], [0, 1, 0], [0, 0, 1]), cam['rpy']):
        cam_R = cam_R@rot(axis, angle)
    return tip_p, tip_R, cam_p, cam_R


def pwm_angle(arm, servo, pwm):
    """Joint angle for a servo PWM under the assumed mapping: sign*(pwm-center)*rad_per_us + offset (zero offset)."""
    conv = arm['pwm_to_angle']
    return conv['sign'][servo]*(pwm-conv['center_us'])*conv['rad_per_us']+float(conv.get('offset_rad', {}).get(servo, 0.0))


def limits(arm):
    """Joint angle range implied by the servo PWM limits and the assumed PWM->angle mapping (the generated URDF
    uses the same range, so every legal PWM is reachable in the simulation)."""
    out = {}
    for spec in arm['joints']:
        a = [pwm_angle(arm, spec['servo'], pwm) for pwm in arm['pwm_limits_us']]
        out[spec['name']] = (min(a), max(a))
    return out


def to_pwm(arm, angles):
    conv = arm['pwm_to_angle']
    offset = conv.get('offset_rad', {})
    return {spec['servo']: int(round(conv['center_us']+(angles[spec['name']]-float(offset.get(spec['servo'], 0.0)))
                                     / (conv['sign'][spec['servo']]*conv['rad_per_us'])))
            for spec in arm['joints']}


def names(arm):
    return [spec['name'] for spec in arm['joints']]


def ik_tip(arm, target):
    """Joint angles putting the tip at `target` (robot base frame). Planar 2-link (shoulder, elbow+wrist+tip)
    after base yaw; returns None when out of reach or outside the joint limits. Elbow-up solution first."""
    n = names(arm)
    lim = limits(arm)
    j = arm['joints']
    mount = np.array(arm['mount_xyz'], float)+np.array(j[0]['parent_offset'], float)+np.array(j[1]['parent_offset'], float)
    l1 = float(j[2]['parent_offset'][0])
    l2 = float(j[3]['parent_offset'][0])+float(arm['tip_offset'][0])
    d = np.asarray(target, float)-mount
    yaw = math.atan2(d[1], d[0])
    r, h = math.hypot(d[0], d[1]), d[2]
    c2 = (r*r+h*h-l1*l1-l2*l2)/(2*l1*l2)
    if abs(c2) > 1:
        return None
    for phi2 in (math.acos(c2), -math.acos(c2)):            # phi = upward angles; theta (+down) = -phi
        phi1 = math.atan2(h, r)-math.atan2(l2*math.sin(phi2), l1+l2*math.cos(phi2))
        angles = {n[0]: yaw, n[1]: -phi1, n[2]: -phi2, n[3]: 0.0}
        if all(lim[k][0] <= v <= lim[k][1] for k, v in angles.items()):
            return angles
    return None


def aim(arm, target, base=(0.0, 0.0, 0.0, 0.0), dist=(0.25, 2.5), step=.05, max_forward=None, min_tip_z=None):
    """Joint angles pointing the camera axis at `target` (same frame as `base`): grid over shoulder/elbow,
    base yaw from the bearing. Optional compact-posture limits keep the arm like a raised head instead of reaching
    out: tip at most `max_forward` ahead of the mount (along the pan direction) and at least `min_tip_z` high.
    Returns (angles, off_axis_rad) or (None, inf)."""
    n = names(arm)
    lim = limits(arm)
    x, y, z, byaw = base
    mount = np.array([x, y, z])+rot([0, 0, 1], byaw)@np.array(arm['mount_xyz'], float)
    tgt = np.asarray(target, float)
    yaw = math.atan2(tgt[1]-mount[1], tgt[0]-mount[0])-byaw
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    yaw = min(max(yaw, lim[n[0]][0]), lim[n[0]][1])
    best, best_err = None, math.inf
    for t1 in np.arange(lim[n[1]][0], lim[n[1]][1]+1e-9, step):
        for t2 in np.arange(lim[n[2]][0], lim[n[2]][1]+1e-9, step):
            angles = {n[0]: yaw, n[1]: float(t1), n[2]: float(t2), n[3]: 0.0}
            tip, _, cp, cR = fk(arm, angles, base)
            if min_tip_z is not None and tip[2] < min_tip_z:
                continue
            if max_forward is not None:
                ahead = (tip[0]-mount[0])*math.cos(byaw+yaw)+(tip[1]-mount[1])*math.sin(byaw+yaw)
                if ahead > max_forward:
                    continue
            v = tgt-cp
            dv = np.linalg.norm(v)
            if not dist[0] <= dv <= dist[1]:
                continue
            err = math.acos(max(-1.0, min(1.0, float(cR[:, 0]@v)/dv)))
            if err < best_err-1e-9:
                best, best_err = angles, err
    return best, best_err


def servo_protocol(root):
    """The field servo_protocol.py (poses and press cycles that the Jetson runs), loaded read only without ROS."""
    path = Path(root)/'src/robot_arm_pkg/robot_arm_pkg/servo_protocol.py'
    spec = importlib.util.spec_from_file_location('field_servo_protocol', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def taught_press(arm, sp, cycle):
    """Tip of the taught press pose of a cycle (its hold step, where the button is held down) in the robot frame:
    {'xy', 'z', 'yaw' (horizontal pointing direction of the tip), 'pose', 'pwm'}. Under the assumed geometry and
    PWM->angle mapping of world_params arm (U-H04): a wrong mapping moves this point."""
    hold = next(pose for pose, _, send in sp.PRESS_CYCLES[cycle] if not send)
    pwm = sp.POSE_TABLES[cycle][hold]
    angles = {spec['name']: pwm_angle(arm, spec['servo'], pwm[spec['servo']]) for spec in arm['joints']}
    tip, R, _, _ = fk(arm, angles)
    return {'xy': [round(float(tip[0]), 4), round(float(tip[1]), 4)], 'z': round(float(tip[2]), 4),
            'yaw': round(math.atan2(float(R[1, 0]), float(R[0, 0])), 4), 'pose': hold, 'pwm': dict(pwm)}
