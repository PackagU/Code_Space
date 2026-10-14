"""Test 3 — parcel pickup with the lift (fork).

Preflight the lift before any motion. Go to the parcel area (fixed approach pose; the mission's Nav2 route usually
brings the robot there already), switch to PARCEL_PICKUP_STATE, turn right 90 deg, lower the fork to the pocket
height and scan the parcels with the arm camera, turning the base left/centre/right so the real camera view changes.
With several parcels take the right-most one (nearest the entrance) first. Drive to a pre-dock pose, look again at
the pallet front face (twice), dock the fork into the pocket, pick up, and raise the lift only until the load stays
below the arm (ultrasonic stop, or a small fixed lift capped by the allowed load-top height) so it does not hide the
arm or its camera. The robot counts as loaded only after the lift confirms the raise.
"""
import math

from .geometry import pallet_dock, rightmost, rotate, to_frame
from .machine import Machine


class ParcelPickup(Machine):
    name = 'parcel_pickup'

    def steps(self):
        io, c = self.io, self.cfg
        self.enter('PARCEL_PICKUP_STATE')
        io.base.stop()
        self.stow()
        ready = io.lift.ready()
        self.note(lift=ready)
        self.require(ready.get('ok'), 'lift not ready (interface/feedback)', lift=ready)
        if c.get('approach_pose_map'):
            target = io.base.map_to_odom(tuple(c['approach_pose_map']))
            x, y, _ = io.base.pose()
            err = math.hypot(target[0]-x, target[1]-y)
            self.enter('ARRIVE_PARCEL_AREA', error_m=round(err, 3))
            if err > c['approach_tol_m']:
                self.drive(target)
        self.enter('TURN_RIGHT_90', purpose='face the parcel area')
        turn = io.base.turn_by(-math.pi/2)
        self.require(turn.get('status') == 'REACHED', 'turn failed', result=turn)
        self.enter('LOWER_FORK')
        lowered = io.lift.lower()
        self.note(lift=lowered)
        self.require(lowered.get('reached'), 'fork not at the pocket height', lift=lowered)
        ref = io.base.pose()
        self.enter('SCAN_PARCELS', headings_rad=c['scan_yaws_rad'])
        found = []
        for dyaw in c['scan_yaws_rad']:
            self.drive((ref[0], ref[1], math.atan2(math.sin(ref[2]+dyaw), math.cos(ref[2]+dyaw))))
            x, y, yaw = io.base.pose()
            sx, sy = rotate((c['scan_dist_m'], 0.0), yaw)
            self.view('parcel_view', (x+sx, y+sy, c['scan_z_m']))
            for p in io.vision.parcels():
                if all(math.dist(p['xyz'][:2], q['xyz'][:2]) > c['same_parcel_m'] for q in found):
                    found.append(p)
            self.stow()
        self.note(parcels=[[round(v, 3) for v in p['xyz'][:2]]+[round(to_frame(ref, p['xyz'])[1], 3)] for p in found])
        self.require(found, 'no parcel seen by the arm camera')
        target = rightmost([p['xyz'] for p in found], ref)
        x, y, _ = io.base.pose()
        axis = (target[0]-x, target[1]-y)
        norm = math.hypot(*axis)
        axis = (axis[0]/norm, axis[1]/norm)
        face = None
        for look in range(int(c['face_looks'])):
            centre_guess = face['centre'] if face else target[:2]
            normal = face['normal'] if face else axis
            pre = (centre_guess[0]-normal[0]*c['predock_m'], centre_guess[1]-normal[1]*c['predock_m'],
                   math.atan2(normal[1], normal[0]))
            self.enter('GO_TO_PREDOCK', look=look, target=[round(v, 3) for v in pre])
            self.stow()
            self.drive(pre)
            self.enter('LOOK_PALLET_FACE', look=look)
            fx = (centre_guess[0]-normal[0]*c['pallet_depth_m']/2, centre_guess[1]-normal[1]*c['pallet_depth_m']/2)
            self.view('pallet_view', (fx[0], fx[1], c['face_z_m']))
            seen = io.vision.pallet_face(fx)
            self.require(seen is not None, 'pallet front face not found by the arm camera')
            lo, hi = c['pallet_face_width_m']
            self.require(lo <= seen['width_m'] <= hi, 'pallet face width implausible', width_m=seen['width_m'])
            centre, dock = pallet_dock(seen['mid'], seen['normal'], c['pallet_depth_m'], c['fork_centre_ahead_m'])
            face = {'centre': centre, 'normal': seen['normal'], 'dock': dock}
            self.note(face_width_m=round(seen['width_m'], 3), centre=[round(v, 3) for v in centre],
                      yaw=round(dock[2], 4))
        self.enter('DOCK', target=[round(v, 3) for v in face['dock']])
        self.stow()
        self.drive(face['dock'], max_v=c['dock_v'], tol=.01, yaw_tol=.015)
        self.enter('PICK_UP')
        attach = io.load.attach()
        self.note(attach=attach)
        self.require(attach.get('ok'), 'fork not engaged with the pallet', attach=attach)
        self.enter('LIFT_LOAD')
        lifted = io.lift.raise_for_carry()
        self.note(lift=lifted)
        self.require(lifted.get('reached'), 'lift did not reach a carry height', lift=lifted)
        top = lifted.get('load_top_m')
        self.require(top is not None and top <= c['arm_clear_top_m'], 'load would hide the arm/camera',
                     load_top_m=top, limit_m=c['arm_clear_top_m'])
        io.base.set_loaded(True)
        # Leave the pickup wall in a straight line before Nav2 turns with the
        # larger loaded footprint. Rear clearance remains checked by the base.
        distance = float(c['undock_m'])
        self.require(distance > 0, 'loaded undock distance must be positive')
        self.enter('BACK_OFF_WITH_LOAD', distance_m=distance)
        x, y, yaw = io.base.pose()
        dx, dy = rotate((-distance, 0.0), yaw)
        self.stow()
        self.drive((x+dx, y+dy, yaw), reverse=True, max_v=c['dock_v'], final_yaw=False)
        self.enter('DONE', load_top_m=top)
