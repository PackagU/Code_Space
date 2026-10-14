#!/usr/bin/env python3
"""Offline tests of the shared skill-test code (no ROS, no hardware): state machines with fake adapters, the drive
controller on a kinematic model, LiDAR judgements, arm status contract, floor-frame freshness, lift cap, geometry,
image processing.

  python3 tools/jetson_skill_tests/tests/test_machines.py
"""
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skilltests import scan_core as sc                 # noqa: E402
from skilltests.control import drive_to                # noqa: E402
from skilltests.elevator_entry import ElevatorEntry    # noqa: E402
from skilltests.elevator_ride import ElevatorRide      # noqa: E402
from skilltests.geometry import door_axis, face_wall_stance, from_frame, pallet_dock, rightmost, to_frame   # noqa
from skilltests.machine import TestAborted             # noqa: E402
from skilltests.parcel_pickup import ParcelPickup      # noqa: E402

TIPS = {'call': {'xy': [0.7, 0.0], 'z': .8, 'yaw': 0.0}, 'destination': {'xy': [0.75, 0.2], 'z': .8, 'yaw': 0.0}}
COMMON = dict(press_tip=TIPS, press_push_m=.01, press_height_tol_m=.03, press_envelope_m=[.012, .02, .06], reaim_m=.02, recheck_max=[.20, .10, .2],
              press_view={'call': 'hall_press_view', 'destination': 'cabin_press_view'},
              confirm_by={'call': ['lamp', 'door'], 'destination': ['lamp']},
              front_stop_m=.02, line_up_m=.25, press_retries=1, back_off_m=.3, slow_v=.05,
              cabin_v=.08, door_timeout_s=30, call_retries=1, floor_confirm_frames=3, floor_reconfirm_frames=2,
              floor_period_s=.5, floor_timeout_s=20, door_open_margin_m=.6, door_align_m=.7, door_through_m=.55, door_width_m=[.7, 1.4], door_centre_tol_m=.03, door_lateral_tol_m=.12, door_jamb_range_diff_m=.35, door_match_m=.45, door_match_rad=.45, door_confirm_margin_m=.3, depth_adjust_max_m=.05, display_ahead_m=1.2,
              look_back_steps=2, look_back_m=.1, confirm_window_s=0)


class World:
    def __init__(self):
        self.t, self.pose, self.door_open_at, self.pressed = 0.0, (0.0, 0.0, 0.0), None, []
        self.lit_after = 1           # the lamp lights after this many presses
        self.floors, self.stops, self.turns, self.drives = [], 0, [], []
        self.block_next = 0          # number of front-stop results for guarded drives
        self.block_entry = 0         # front stops only on the drive through the door (door closing)
        self.range_unknown = False
        self.stow_ok = True
        self.loaded = False
        self.view_ok = True
        self.view_out = 0           # views that reach the posture but leave the target out of the picture
        self.doors = [((1.0, 1.0), (1.0, 0.0)), ((0.0, 1.0), (0.0, 1.0))]   # entry / ride test doors
        self.door_close_at = None    # hold-open timer: the door closes again at this time

    def door_is_open(self):
        return (self.door_open_at is not None and self.t >= self.door_open_at
                and (self.door_close_at is None or self.t < self.door_close_at))


class Clock:
    def __init__(self, w):
        self.w = w

    def now(self):
        return self.w.t

    def sleep(self, s):
        self.w.t += s


class Base:
    def __init__(self, w):
        self.w = w

    def pose(self):
        return self.w.pose

    def map_to_odom(self, p):
        return tuple(p)

    def go_to(self, target, **kw):
        self.w.drives.append(tuple(round(v, 3) for v in target))
        if kw.get('min_front') is not None and (self.w.block_next or (self.w.block_entry and target[0] > 1.4)):
            if target[0] > 1.4 and self.w.block_entry:
                self.w.block_entry -= 1
            else:
                self.w.block_next -= 1
            self.w.t += 2
            return {'status': 'FRONT_STOP'}
        self.w.pose = tuple(target)
        self.w.t += 2
        return {'status': 'REACHED'}

    def turn_by(self, a):
        x, y, yaw = self.w.pose
        self.w.turns.append(round(a, 3))
        self.w.pose = (x, y, math.atan2(math.sin(yaw+a), math.cos(yaw+a)))
        return {'status': 'REACHED'}

    def stop(self):
        self.w.stops += 1

    def scan(self):
        """Synthetic LiDAR: each test door is a wall line with a 1.0 m opening (open after door_open_at)."""
        x, y, yaw = self.w.pose
        opened = self.w.door_is_open()
        ranges = []
        for i in range(720):
            a = yaw-math.pi+2*math.pi*i/720
            dx, dy = math.cos(a), math.sin(a)
            best = float('inf')
            for (cx, cy), (nx, ny) in self.w.doors:
                den = dx*nx+dy*ny
                if abs(den) < 1e-6:
                    continue
                t = ((cx-x)*nx+(cy-y)*ny)/den
                if t <= .15:
                    continue
                lateral = (x+dx*t-cx)*-ny+(y+dy*t-cy)*nx
                if opened and abs(lateral) < .5:
                    continue
                best = min(best, t)
            ranges.append(best if best <= 8.0 else float('inf'))     # beyond range_max: no return
        return sc.Scan(-math.pi, 2*math.pi/720, ranges, .15, 8.0)

    def front_extent(self):
        return .60

    def geometry(self):
        return {'front_extent': .60, 'rear_extent': .33, 'half_width': .22}

    def set_loaded(self, flag):
        self.w.loaded = flag

    def range_at(self, bearing):
        if self.w.range_unknown:
            return None
        return 3.0 if self.w.door_is_open() else 1.0


class Arm:
    def __init__(self, w):
        self.w = w

    def posture(self, name):
        return {'reached': self.w.stow_ok}

    def view(self, name, target):
        if self.w.view_out:
            self.w.view_out -= 1
            return {'reached': False, 'in_view': False, 'view': name}
        return {'reached': self.w.view_ok, 'view': name}

    def press(self, button):
        self.w.pressed.append(button['label'])
        if button['kind'] == 'hall' or self.w.door_open_at is None:
            self.w.door_open_at = self.w.t+20       # the car arrives / the door reopens some time after the press
        return {'done': True}


class Vision:
    def __init__(self, w, button=True, parcels=None, face=None):
        self.w, self.button, self.p, self.face = w, button, parcels or [], face

    def find_button(self, kind, label, prior, hint=None):
        if not self.button:
            return None
        return {'xyz': [1.0, 0.0, .8], 'normal': [-1.0, 0.0], 'kind': kind, 'label': label}

    def button_lit(self, button):
        return len(self.w.pressed) >= self.w.lit_after

    def parcels(self):
        return [{'xyz': list(p)} for p in self.p]

    def pallet_face(self, near_xy):
        return self.face

    def floor(self):
        return self.w.floors.pop(0) if self.w.floors else None


def io_for(w, lift_ok=True, **vision):
    return SimpleNamespace(clock=Clock(w), base=Base(w), arm=Arm(w), vision=Vision(w, **vision),
                           lift=SimpleNamespace(ready=lambda: {'ok': lift_ok},
                                                raise_for_carry=lambda: {'reached': True, 'load_top_m': .52},
                                                lower=lambda: {'reached': True}),
                           load=SimpleNamespace(attach=lambda: {'ok': True}, detach=lambda: {'ok': True}))


def states(machine):
    return [e['state'] for e in machine.events]


class SelectedButtonLightTests(unittest.TestCase):
    def test_only_the_requested_cap_patch_counts(self):
        from skilltests.vision_core import button_light
        image = np.zeros((80,120,3),np.uint8)
        image[25:46,75:96] = [0,155,255]
        amber = [[[10,180,100],[26,255,255]]]
        self.assertFalse(button_light(image,(30,35),amber))
        self.assertTrue(button_light(image,(85,35),amber))
        self.assertFalse(button_light(None,(85,35),amber))
        self.assertFalse(button_light(image,(-1,35),amber))

    def test_rendered_yellow_cap_is_recognized_at_selected_pixel(self):
        from skilltests.vision_core import button_light
        image = np.zeros((60,80,3),np.uint8)
        image[15:36,25:46] = [0,255,255]
        self.assertTrue(button_light(image,(35,25),[[[10,201,150],[38,255,255]]]))
        self.assertFalse(button_light(image,(65,25),[[[10,201,150],[38,255,255]]]))

    def test_reacquired_label_ignores_old_projection_other_button_and_display(self):
        import cv2
        from skilltests import vision_core as vc
        hsv = {'board': [[[0,0,0],[180,255,45]]], 'button': [[[0,0,110],[180,70,255]]],
               'lamp': [[[10,201,150],[38,255,255]]]}
        image = np.full((240,320,3), 80, np.uint8)
        image[60:145,100:195] = 0
        for pixel in ((120,80),(170,80),(120,120),(170,120)):
            cv2.circle(image,pixel,9,(255,255,255),-1)
        cv2.circle(image,(170,80),9,(0,255,255),-1)  # button 2 is on
        image[15:30,230:270] = (0,255,255)          # unrelated display
        def requested_lit():
            caps = vc.button_caps(image,hsv,150,8)
            cap = vc.selected_cap(caps,'car','1',(120,105))  # old xyz projected 25 px away
            self.assertIsNotNone(cap)
            self.assertLess(math.dist((cap['u'],cap['v']),(120,80)),1)
            return vc.button_light(image,(cap['u'],cap['v']),hsv['lamp'],radius=8)
        self.assertFalse(requested_lit())
        cv2.circle(image,(120,80),9,(0,255,255),-1)
        self.assertTrue(requested_lit())

class EntryTests(unittest.TestCase):
    cfg = dict(COMMON, floor='F1', call_label='UP', call_button_prior_map=[1.0, 0.0, .8],
               button_view_pose_map=[0.0, 0.0, 0.0], cabin_centre_map=[2.0, 1.0], door_centre_map=[1.0, 1.0],
               door_normal_map=[1.0, 0.0])

    def test_happy_path(self):
        w = World()
        m = ElevatorEntry(io_for(w), self.cfg, echo=None)
        m.run()
        order = ['ELEVATOR_ENTRY_STATE', 'GO_TO_BUTTON_AREA', 'LOOK_FOR_BUTTON', 'APPROACH_BUTTON', 'PRESS_BUTTON',
                 'BACK_OFF', 'CONFIRM_PRESS', 'RETURN_TO_DOOR', 'WAIT_DOOR_OPEN', 'ENTER_CABIN', 'GO_TO_CABIN_CENTRE',
                 'DONE']
        self.assertEqual([x for x in states(m) if x in order], order)
        self.assertEqual(w.pressed, ['UP'])
        self.assertAlmostEqual(w.pose[0], 2.0)
        self.assertAlmostEqual(w.pose[2], 0.0)          # entered facing into the cabin

    def test_retry_until_lit(self):
        w = World()
        w.lit_after = 2
        ElevatorEntry(io_for(w), self.cfg, echo=None).run()
        self.assertEqual(w.pressed, ['UP', 'UP'])

    def test_entry_blocked_then_called_again(self):
        w = World()
        w.block_entry = 1           # the door closes in front of the robot on the first entry
        m = ElevatorEntry(io_for(w), self.cfg, echo=None)
        m.run()
        self.assertIn('ENTRY_BLOCKED', states(m))
        self.assertEqual(states(m)[-1], 'DONE')
        self.assertEqual(w.pressed, ['UP', 'UP'])      # called again after backing out

    def test_small_heading_and_depth_miss_is_corrected_before_pressing(self):
        w = World()
        io = io_for(w)
        original = io.base.go_to
        injected = []
        def drift(target, **kw):
            out = original(target, **kw)
            if kw.get('min_front') is not None and kw.get('final_yaw') is False and not injected:
                x,y,yaw=w.pose
                w.pose=(x-.03,y,yaw+.065)
                injected.append(True)
            return out
        io.base.go_to=drift
        m=ElevatorEntry(io,self.cfg,echo=None)
        m.run()
        self.assertEqual(w.pressed,['UP'])
        self.assertIn('SQUARE_TO_BUTTON',states(m))
        self.assertIn('ADJUST_DEPTH',states(m))
        self.assertLess(states(m).index('SQUARE_TO_BUTTON'),states(m).index('PRESS_BUTTON'))

    def test_early_front_stop_is_not_pressed(self):
        w = World()
        w.block_next = 1            # the approach stops far from the stance (unexpected obstacle)
        m = ElevatorEntry(io_for(w), self.cfg, echo=None)
        m.run()
        s = states(m)
        self.assertLess(s.index('STANCE_OUT_OF_ENVELOPE'), s.index('PRESS_BUTTON'))
        self.assertEqual(w.pressed, ['UP'])            # pressed only from a verified stance

    def test_second_look_moves_the_stance(self):
        w = World()
        io = io_for(w)
        looks = []

        def find_button(kind, label, prior, hint=None):
            looks.append(prior)
            y = 0.0 if len(looks) == 1 else 0.03         # the second look sees the button 3 cm to the side
            return {'xyz': [1.0, y, .8], 'normal': [-1.0, 0.0], 'kind': kind, 'label': label}
        io.vision.find_button = find_button
        m = ElevatorEntry(io, self.cfg, echo=None)
        m.run()
        recheck = [e for e in m.events if e['state'] == 'RECHECK_BUTTON'][0]
        self.assertAlmostEqual(recheck['shift_m'], .03, places=3)
        s = states(m)
        self.assertLess(s.index('RECHECK_BUTTON'), s.index('PRESS_BUTTON'))
        self.assertIn('LINE_UP_AGAIN', s)                            # 3 cm > reaim 2 cm: square up again first
        # taught call tip 0.70 m ahead on the centre line, 1 cm past the face: stance x = 1.0 + 0.01 - 0.70
        self.assertIn((.31, .03, 0.0), w.drives)
        self.assertLess(s.index('LINE_UP_AGAIN'), s.index('PRESS_BUTTON'))

    def test_after_lining_up_again_it_looks_again_before_driving_in(self):
        # codex_flat_wrist_20261014_151831: the turn of the second line-up drifted the odometry heading by 0.04 rad
        # after the last look -> the tip missed the cabin button by 3 cm. The press is planned from a look taken
        # where the robot drives in from.
        w = World()
        io = io_for(w)
        seen = iter([0.0, 0.03, 0.045])                  # the third look (after LINE_UP_AGAIN) sees the drift
        last = [0.0]

        def find_button(kind, label, prior, hint=None):
            last[0] = next(seen, last[0])
            return {'xyz': [1.0, last[0], .8], 'normal': [-1.0, 0.0], 'kind': kind, 'label': label}
        io.vision.find_button = find_button
        m = ElevatorEntry(io, self.cfg, echo=None)
        m.run()
        s = states(m)
        first_press = s.index('PRESS_BUTTON')
        self.assertEqual(s[:first_press].count('RECHECK_BUTTON'), 2)
        self.assertGreater(s.index('RECHECK_BUTTON', s.index('LINE_UP_AGAIN')), s.index('LINE_UP_AGAIN'))
        self.assertIn((.31, .045, 0.0), w.drives)                   # stance from the latest look

    def test_second_look_far_away_is_not_the_same_button(self):
        w = World()
        io = io_for(w)
        looks = []

        def find_button(kind, label, prior, hint=None):
            looks.append(prior)
            x = 1.0 if len(looks) == 1 else 5.0          # the second look latches onto something 4 m away
            return {'xyz': [x, 0.0, .8], 'normal': [-1.0, 0.0], 'kind': kind, 'label': label}
        io.vision.find_button = find_button
        m = ElevatorEntry(io, self.cfg, echo=None)
        m.run()
        recheck = [e for e in m.events if e['state'] == 'RECHECK_BUTTON'][0]
        self.assertIn('rejected', recheck)
        self.assertFalse(any(d[0] > 2.0 for d in w.drives))      # never drove toward the far detection

    def test_too_close_to_see_backs_off_and_looks_again(self):
        w = World()
        w.view_out = 2              # the camera cannot see the button from the first two spots
        m = ElevatorEntry(io_for(w), self.cfg, echo=None)
        m.run()
        s = states(m)
        self.assertEqual(s.count('BACK_OFF_TO_SEE'), 2)
        self.assertLess(s.index('BACK_OFF_TO_SEE'), s.index('APPROACH_BUTTON'))
        self.assertEqual(s[-1], 'DONE')

    def test_view_posture_failure_never_drives(self):
        w = World()
        w.view_ok = False           # the arm itself did not reach the view posture
        m = ElevatorEntry(io_for(w), self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertNotIn('BACK_OFF_TO_SEE', states(m))
        self.assertNotIn('APPROACH_BUTTON', states(m))

    def test_button_not_found_stops(self):
        w = World()
        m = ElevatorEntry(io_for(w, button=False), self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertEqual(m.state, 'FAILED')
        self.assertGreaterEqual(w.stops, 2)

    def test_rotated_arm_press_is_refused_before_motion_or_press(self):
        w = World()
        tips = dict(TIPS, call=dict(TIPS['call'], yaw=.628))
        m = ElevatorEntry(io_for(w), dict(self.cfg, press_tip=tips), echo=None)
        with self.assertRaises(TestAborted) as err:
            m.run()
        self.assertIn('horizontal rotation', str(err.exception))
        self.assertNotIn('APPROACH_BUTTON', states(m))
        self.assertEqual(w.pressed, [])

    def test_button_height_outside_the_taught_press_is_refused(self):
        tips = dict(TIPS, call=dict(TIPS['call'], z=1.0))       # the cycle presses 0.2 m above the button
        m = ElevatorEntry(io_for(World()), dict(self.cfg, press_tip=tips), echo=None)
        with self.assertRaises(TestAborted) as err:
            m.run()
        self.assertIn('height', str(err.exception))
        self.assertNotIn('APPROACH_BUTTON', states(m))

    def test_only_confirmed_retry_saves_the_actual_press_pose(self):
        w = World()
        w.lit_after = 2
        io = io_for(w)
        candidates, saved = [], []
        def capture(floor, kind, label, button, pose, tip):
            record = {'floor': floor, 'kind': kind, 'label': label, 'press_pose': pose}
            candidates.append(record)
            return record
        io.press_memory = SimpleNamespace(lookup=lambda *a: None, capture=capture, commit=saved.append)
        cfg = dict(self.cfg, confirm_by={'call': ['lamp'], 'destination': ['lamp']})
        m = ElevatorEntry(io, cfg, echo=None)
        m.run()
        self.assertEqual(len(candidates), 2)
        self.assertEqual(saved, [candidates[1]])
        self.assertEqual(saved[0]['label'], 'UP')
        self.assertNotEqual(saved[0]['press_pose'], w.pose)

    def remembered_io(self, w, press_pose, button_xyz=(1.0, 0.0, .8)):
        io = io_for(w)
        entry = {'floor': 'F1', 'kind': 'hall', 'label': 'UP', 'frame': 'map', 'source': 'arm_camera',
                 'button_xyz': list(button_xyz), 'press_pose': list(press_pose), 'run': 'earlier'}
        looked = []
        io.press_memory = SimpleNamespace(lookup=lambda floor, kind, label: entry if (floor, kind, label) ==
                                          ('F1', 'hall', 'UP') else None,
                                          capture=lambda *a: dict(entry), commit=lambda e: None)
        view = io.arm.view
        io.arm.view = lambda name, target: looked.append(name) or view(name, target)
        return io, looked

    def test_remembered_press_goes_straight_to_its_line_up(self):
        # stance for the fake button (x=1.0, facing -x) with tip 0.7 m ahead and 1 cm push: x=0.31
        w = World()
        io, looked = self.remembered_io(w, (0.31, 0.0, 0.0))
        m = ElevatorEntry(io, self.cfg, echo=None)
        m.run()
        seq = states(m)
        self.assertNotIn('GO_TO_BUTTON_AREA', seq)
        self.assertNotIn('LOOK_FOR_BUTTON', seq)
        self.assertNotIn('RECHECK_BUTTON', seq)          # the camera just looked from this very line-up
        self.assertEqual(seq[1:4], ['GO_TO_REMEMBERED_LINE_UP', 'APPROACH_BUTTON', 'PRESS_BUTTON'])
        self.assertEqual(w.drives[0], (0.06, 0.0, 0.0))  # 0.25 m behind the remembered press pose
        self.assertEqual(looked[0], 'hall_press_view')
        self.assertEqual(w.pressed, ['UP'])
        self.assertEqual(seq[-1], 'DONE')

    def test_remembered_press_falls_back_to_the_search_when_the_camera_misses(self):
        w = World()
        io, looked = self.remembered_io(w, (0.31, 0.0, 0.0))
        found = io.vision.find_button
        misses = [1]
        def find_button(kind, label, prior, hint=None):
            if misses:                                   # not seen from the remembered line-up
                misses.pop()
                return None
            return found(kind, label, prior, hint)
        io.vision.find_button = find_button
        m = ElevatorEntry(io, self.cfg, echo=None)
        m.run()
        seq = states(m)
        self.assertEqual(seq[1:3], ['GO_TO_REMEMBERED_LINE_UP', 'LOOK_FOR_BUTTON'])
        self.assertIn('RECHECK_BUTTON', seq)
        self.assertEqual(w.pressed, ['UP'])

    def test_remembered_press_of_another_button_is_ignored(self):
        w = World()
        io, _ = self.remembered_io(w, (0.31, 0.0, 0.0), button_xyz=(3.0, 0.0, .8))   # 2 m from the taught prior
        m = ElevatorEntry(io, self.cfg, echo=None)
        m.run()
        self.assertEqual(states(m)[1:3], ['GO_TO_BUTTON_AREA', 'LOOK_FOR_BUTTON'])

    def test_press_that_needs_the_front_past_the_wall_is_refused(self):
        tips = dict(TIPS, call=dict(TIPS['call'], xy=[0.5, 0.0]))   # tip 0.5 m ahead, fork front 0.6 m
        m = ElevatorEntry(io_for(World()), dict(self.cfg, press_tip=tips), echo=None)
        with self.assertRaises(TestAborted) as err:
            m.run()
        self.assertIn('front stop', str(err.exception))

    def test_lamp_free_button_is_accepted_after_the_cycle(self):
        w = World()
        w.lit_after = 99                                     # no lamp ever lights (demo buttons)
        m = ElevatorEntry(io_for(w), dict(self.cfg, confirm_by={'call': [], 'destination': []}), echo=None)
        m.run()
        self.assertEqual(w.pressed, ['UP'])
        self.assertEqual(states(m)[-1], 'DONE')

    def test_post_press_camera_miss_retries_instead_of_accepting_cycle(self):
        w = World()
        io = io_for(w)
        original = io.vision.find_button
        calls = [0]
        def find(*args, **kw):
            calls[0] += 1
            return None if calls[0] == 3 else original(*args, **kw)
        io.vision.find_button = find
        m = ElevatorEntry(io, dict(self.cfg, press_visual_required=True), echo=None)
        m.run()
        self.assertEqual(w.pressed, ['UP', 'UP'])
        self.assertEqual(states(m).count('POST_PRESS_LOOK'), 2)

    def test_no_post_press_camera_evidence_exhausts_retry_and_stops(self):
        w = World()
        io = io_for(w)
        original = io.vision.find_button
        io.vision.find_button = lambda *a, **kw: None if w.pressed else original(*a, **kw)
        m = ElevatorEntry(io, dict(self.cfg, press_visual_required=True), echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertEqual(len(w.pressed), 2)
        self.assertNotIn('ENTER_CABIN', states(m))
        self.assertGreater(w.stops, 0)

    def test_failed_stow_blocks_driving(self):
        w = World()
        w.stow_ok = False
        m = ElevatorEntry(io_for(w), self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertEqual(w.drives, [])                  # never drove with an unstowed arm

    def test_unknown_lidar_never_counts_as_open(self):
        w = World()
        w.range_unknown = True
        m = ElevatorEntry(io_for(w), dict(self.cfg, door_timeout_s=5), echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertNotIn('ENTER_CABIN', states(m))


class RideTests(unittest.TestCase):
    cfg = dict(COMMON, floor='F1', target_floor='F2', target_label='2', target_button_prior_map=[1.0, 0.0, .76],
               door_centre_map=[0.0, 1.0], door_normal_map=[0.0, 1.0], cabin_centre_map=[0.0, 0.0],
               display_prior_map=[0.0, 1.0, 1.13], exit_display_prior_map=[0.0, 1.0, 1.13],
               exit_pose_map=[0.0, 3.0, 1.57], exit_door_centre_map=[0.0, 1.0], exit_door_normal_map=[0.0, 1.0],
               exit_cabin_centre_map=[0.0, 0.0], exit_button_prior_map=[1.0, 0.0, .76])

    def test_floor_display_cannot_confirm_a_missing_lamp_free_car_button(self):
        w = World()
        w.floors = ['F2']*20
        io = io_for(w)
        original = io.vision.find_button
        io.vision.find_button = lambda *a, **kw: None if w.pressed else original(*a, **kw)
        cfg = dict(self.cfg, press_visual_required=True, confirm_by={'call': [], 'destination': []})
        m = ElevatorRide(io, cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run('press')
        self.assertEqual(w.pressed, ['2', '2'])
        self.assertEqual(w.floors, ['F2']*20)
        self.assertEqual(states(m).count('POST_PRESS_LOOK'), 2)

    def test_press_then_exit(self):
        w = World()
        w.door_open_at = 0.0
        m = ElevatorRide(io_for(w), self.cfg, echo=None)
        m.run('press')
        self.assertEqual(w.turns, [round(-math.pi/2, 3), round(-math.pi/2, 3)])   # right 90 twice
        self.assertEqual(w.pressed, ['2'])
        w.floors = [None, 'F1', 'F2', 'F2', 'F2', 'F2', 'F2']
        checked = []
        m.run('exit', before_exit=lambda: checked.append(True))
        s = states(m)
        self.assertEqual(s[-1], 'DONE')
        self.assertTrue(checked)
        self.assertLess(s.index('BEFORE_EXIT_CHECK'), s.index('RECONFIRM_FLOOR'))
        self.assertLess(s.index('RECONFIRM_FLOOR'), s.index('WAIT_DOOR_OPEN'))
        self.assertLess(s.index('WAIT_DOOR_OPEN'), s.index('EXIT_CABIN'))

    def test_exit_alone_takes_display_posture(self):
        w = World()
        w.door_open_at = 0.0
        w.floors = ['F2']*6
        m = ElevatorRide(io_for(w), self.cfg, echo=None)
        m.run('exit')
        self.assertEqual(states(m)[0], 'DISPLAY_ARM_POSTURE')

    def test_wrong_floor_never_exits(self):
        w = World()
        w.door_open_at = 0.0
        w.floors = ['F1']*100
        m = ElevatorRide(io_for(w), self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run('exit')
        self.assertNotIn('EXIT_CABIN', states(m))

    def test_floor_changed_before_leaving(self):
        w = World()
        w.door_open_at = 0.0
        w.floors = ['F2', 'F2', 'F2']+['F1']*100      # confirmed, then the car moved on
        m = ElevatorRide(io_for(w), self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run('exit')
        self.assertNotIn('EXIT_CABIN', states(m))

    def test_interrupted_streak_resets(self):
        w = World()
        w.door_open_at = 0.0
        w.floors = ['F2', 'F2', None, 'F2', 'F2', 'F2', 'F2', 'F2']
        m = ElevatorRide(io_for(w), self.cfg, echo=None)
        m.run('exit')
        wait = next(e for e in m.events if e['state'] == 'WAIT_TARGET_FLOOR')
        self.assertEqual(wait['streak'], 3)

    def test_door_closing_during_the_exit_line_up_is_not_driven_into(self):
        # codex_flat_wrist_20261014_150631: 0.21 m off the opening's axis, the line-up onto it outlasted the hold-open
        # time and the robot drove straight out into the closed door (front stop, 5 mm)
        w = World()
        w.floors = ['F2']*40
        w.door_open_at = 0.0
        w.pose = (0.2, 0.0, math.pi/2)
        io = io_for(w)
        def press(button):
            w.pressed.append(button['label'])
            w.door_open_at, w.door_close_at = w.t+2, None   # this floor's button reopens the door
            return {'done': True}
        io.arm.press = press
        outs = []
        go_to = io.base.go_to
        def guarded(target, **kw):
            if kw.get('min_front') is not None and target[1] > .5:
                outs.append(w.door_is_open())                # every drive out through the door
            result = go_to(target, **kw)
            if 'EXIT_CABIN' in states(m) and not w.pressed and w.door_close_at is None:
                w.door_close_at = w.t                        # the hold-open time ran out during the line-up
            return result
        io.base.go_to = guarded
        m = ElevatorRide(io, self.cfg, echo=None)
        m.run('exit')
        self.assertTrue(outs)
        self.assertTrue(all(outs), outs)
        self.assertIn('REOPEN_DOOR', states(m))
        self.assertEqual(states(m)[-1], 'DONE')

    def test_exit_reopens_closed_door(self):
        w = World()
        w.floors = ['F2']*20            # door closed until this floor's button is pressed again
        m = ElevatorRide(io_for(w), self.cfg, echo=None)
        m.run('exit')
        self.assertIn('REOPEN_DOOR', states(m))
        self.assertEqual(w.pressed, ['2'])
        self.assertEqual(states(m)[-1], 'DONE')



class PickupTests(unittest.TestCase):
    cfg = dict(scan_dist_m=.95, scan_yaws_rad=[0.0, .35, -.35], scan_z_m=.2, same_parcel_m=.25, predock_m=.85,
               face_looks=2, face_z_m=.05, pallet_face_width_m=[.3, .5], pallet_depth_m=.4, fork_centre_ahead_m=.37,
               dock_v=.05, undock_m=.3, arm_clear_top_m=.6, approach_tol_m=.3, approach_pose_map=[0.0, 0.0, 0.0])

    def test_rightmost_parcel_and_dock(self):
        w = World()
        face = {'mid': [-.5, -.75], 'normal': [0.0, -1.0], 'width_m': .4}
        io = io_for(w, parcels=[(.4, -.95, .2), (-.5, -.95, .2)], face=face)
        m = ParcelPickup(io, self.cfg, echo=None)
        m.run()
        self.assertEqual(m.state, 'DONE')
        dock = next(e for e in m.events if e['state'] == 'DOCK')['target']
        self.assertAlmostEqual(dock[0], -.5, places=3)          # right-most (entrance side) parcel chosen
        self.assertAlmostEqual(dock[1], -.95+.37, places=3)
        self.assertAlmostEqual(dock[2], -math.pi/2, places=3)
        self.assertTrue(w.loaded)
        last = w.drives[-1]
        self.assertAlmostEqual(last[0], dock[0], places=3)
        self.assertAlmostEqual(last[1], dock[1]+self.cfg['undock_m'], places=3)
        self.assertAlmostEqual(last[2], dock[2], places=3)
        scan_headings = [d[2] for d in w.drives[:3]]             # the base turned for the three scan views
        self.assertEqual(len(set(round(h, 2) for h in scan_headings)), 3)

    def test_loaded_undock_blocked_rear_stops_before_done(self):
        w = World()
        io = io_for(w, parcels=[(0.0, -.95, .2)], face={'mid': [0.0, -.75], 'normal': [0.0, -1.0], 'width_m': .4})
        original = io.base.go_to
        def guarded(target, **kw):
            if w.loaded and kw.get('reverse') is True:
                return {'status': 'REAR_STOP'}
            return original(target, **kw)
        io.base.go_to = guarded
        m = ParcelPickup(io, self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertIn('BACK_OFF_WITH_LOAD', states(m))
        self.assertNotIn('DONE', states(m))
        self.assertGreater(w.stops, 0)

    def test_lift_not_ready_means_no_motion(self):
        w = World()
        m = ParcelPickup(io_for(w, lift_ok=False), self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertEqual(w.drives, [])
        self.assertEqual(w.turns, [])

    def test_load_hiding_the_arm_fails(self):
        w = World()
        io = io_for(w, parcels=[(0.0, -.95, .2)], face={'mid': [0.0, -.75], 'normal': [0.0, -1.0], 'width_m': .4})
        io.lift.raise_for_carry = lambda: {'reached': True, 'load_top_m': .75}
        m = ParcelPickup(io, self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run()
        self.assertFalse(w.loaded)

    def test_no_parcel(self):
        m = ParcelPickup(io_for(World()), self.cfg, echo=None)
        with self.assertRaises(TestAborted):
            m.run()


class PressMemoryTests(unittest.TestCase):
    def test_floor_and_button_keys_survive_restart_without_crossing_revisions(self):
        from skilltests.press_memory import PressMemory
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'presses.json'
            memory = PressMemory(path, 'geometry-v1')
            for floor, label in (('F1', '2'), ('F2', '1')):
                memory.commit({'floor': floor, 'kind': 'car', 'label': label, 'frame': 'map',
                               'source': 'arm_camera', 'button_xyz': [1., 2., .68], 'press_pose': [.5, 2.1, 0.]})
            loaded = PressMemory(path, 'geometry-v1')
            self.assertIsNotNone(loaded.lookup('F1', 'car', '2'))
            self.assertIsNone(loaded.lookup('F2', 'car', '2'))
            self.assertIsNone(PressMemory(path, 'geometry-v2').lookup('F1', 'car', '2'))

    def test_nonfinite_pose_is_not_saved(self):
        from skilltests.press_memory import PressMemory
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'presses.json'
            memory = PressMemory(path, 'v1')
            with self.assertRaises(ValueError):
                memory.commit({'floor': 'F1', 'kind': 'car', 'label': '2', 'frame': 'map',
                               'source': 'arm_camera', 'button_xyz': [1., 2., .68],
                               'press_pose': [float('nan'), 2.1, 0.]})
            self.assertFalse(path.exists())


class ControllerTests(unittest.TestCase):
    """drive_to on a unicycle model."""

    def run_drive(self, target, start=(0.0, 0.0, 0.0), check=None, stale_after=None, slip=0.0, accel=None, delay=0.0,
                  **kw):
        """accel=(m/s^2, rad/s^2), delay=s: the wheels follow the command late (safety gate, bridge, MCU) and
        acceleration-limited (max_wheel_accel_rpm_s), so the base coasts on after a zero command."""
        state = {'p': list(start), 't': 0.0, 'cmds': [], 'av': 0.0, 'aw': 0.0, 'late': []}

        def pose():
            if stale_after is not None and state['t'] > stale_after:
                return None
            return tuple(state['p'])

        def send(v, w):
            state['cmds'].append((v, w))
            state['v'], state['w'] = v, w

        def sleep(dt):
            v, w = state.get('v', 0.0), state.get('w', 0.0)
            if delay:
                state['late'].append((state['t']+delay, v, w))
                v, w = state.get('applied', (0.0, 0.0))
                while state['late'] and state['late'][0][0] <= state['t']+1e-9:
                    _, v, w = state['late'].pop(0)
                state['applied'] = (v, w)
            if accel is not None:
                state['av'] += max(-accel[0]*dt, min(accel[0]*dt, v-state['av']))
                state['aw'] += max(-accel[1]*dt, min(accel[1]*dt, w-state['aw']))
                v, w = state['av'], state['aw']
            x, y, yaw = state['p']
            state['p'] = [x+v*math.cos(yaw)*dt, y+v*math.sin(yaw)*dt+slip*dt*abs(v), yaw+w*dt]
            state['t'] += dt
        result = drive_to(pose, send, sleep, lambda: state['t'], target, check=check, **kw)
        return result, state

    def test_reaches_target(self):
        result, _ = self.run_drive((1.0, 0.5, math.pi/2))
        self.assertEqual(result['status'], 'REACHED')
        self.assertLess(result['pos_err_m'], .03)

    def test_coasting_past_a_tight_line_up_is_corrected(self):
        # codex_flat_wrist_20261014_145133: the base coasted ~2.3 cm past the button line-up point while the final
        # turn had already started -> POSITION_ERROR 3.2 cm against arrive_tol 1.5 cm
        lineup = dict(max_v=.05, tol=.005, reaim_tol=.01, arrive_tol=.015, yaw_tol=.006, yaw_arrive_tol=.012)
        for target in ((0.33, -0.05, -0.4), (0.30, 0.08, 0.35)):
            for delay, accel in ((.6, (.05, .3)), (.8, (.1, .5)), (.2, (.1, .5))):
                result, _ = self.run_drive(target, reverse='auto', accel=accel, delay=delay, **lineup)
                self.assertEqual(result['status'], 'REACHED', (target, delay, result))
                self.assertLessEqual(result['pos_err_m'], .015)

    def test_auto_reverse_backs_up_to_a_target_behind(self):
        # target 0.2 m behind-left: back up with a small turn instead of turning ~130 deg to face it
        result, state = self.run_drive((-0.13, 0.15, 0.0), reverse='auto')
        self.assertEqual(result['status'], 'REACHED')
        self.assertTrue(any(v < 0 for v, _ in state['cmds']))
        self.assertFalse(any(v > 0 for v, _ in state['cmds']))
        turned = sum(abs(w) for _, w in state['cmds'])*.1
        self.assertLess(turned, 2.0)                    # ~50 deg there and back, not 130 deg there and back

    def test_blocked_short_turn_goes_the_long_way(self):
        result, state = self.run_drive((0.0, 0.0, -1.0), check=lambda v, w: 'SWEEP_STOP' if w < 0 else None)
        self.assertEqual(result['status'], 'REACHED')                # turned +5.28 rad instead of -1.0
        self.assertTrue(all(w >= 0 for _, w in state['cmds']))

    def test_blocked_heading_correction_drives_straight_on(self):
        # the turn part of every curve is "blocked": the robot still goes straight on, and only turns where it
        # stands still (not blocked there in this check)
        result, state = self.run_drive((1.0, 0.0, 0.0), start=(0.0, 0.0, 0.05),
                                       check=lambda v, w: 'SWEEP_STOP' if abs(v) > 1e-3 and abs(w) > 1e-3 else None)
        self.assertIn(result['status'], ('REACHED', 'POSITION_ERROR'))
        self.assertTrue(all(abs(w) < 1e-9 for v, w in state['cmds'] if abs(v) > 1e-3))

    def test_small_lateral_miss_does_not_turn_around(self):
        # 2 cm off to the side: accept (within reaim_tol) instead of a 90 deg turn to creep sideways
        result, state = self.run_drive((0.0, 0.02, 0.0))
        self.assertEqual(result['status'], 'REACHED')
        self.assertFalse(any(abs(w) > 1e-6 for _, w in state['cmds']))

    def test_lateral_drift_is_corrected_or_reported(self):
        result, _ = self.run_drive((1.0, 0.0, 0.0), slip=.2)
        self.assertIn(result['status'], ('REACHED', 'POSITION_ERROR'))
        if result['status'] == 'REACHED':
            self.assertLessEqual(result['pos_err_m'], .06)

    def test_large_miss_is_never_reached(self):
        result, _ = self.run_drive((1.0, 0.0, 0.0), slip=2.0)
        self.assertIn(result['status'], ('POSITION_ERROR', 'TIMEOUT'))

    def test_check_stops_immediately(self):
        def check(v, w):
            return 'FRONT_STOP' if v > 0 else None
        result, state = self.run_drive((1.0, 0.0, 0.0), check=check)
        self.assertEqual(result['status'], 'FRONT_STOP')
        self.assertTrue(all(abs(v) < 1e-9 and abs(w) < 1e-9 for v, w in state['cmds']))   # nothing but zeros

    def test_stale_pose_stops(self):
        result, _ = self.run_drive((1.0, 0.0, 0.0), stale_after=1.0)
        self.assertEqual(result['status'], 'STALE_POSE')


def make_scan(ranges_fn, n=360):
    return sc.Scan(-math.pi, 2*math.pi/n, [ranges_fn(-math.pi+2*math.pi*i/n) for i in range(n)], .15, 8.0)


class ScanTests(unittest.TestCase):
    geom = {'front_extent': .6, 'rear_extent': .33, 'half_width': .22}

    def test_nan_scan_is_unknown_not_open(self):
        s = make_scan(lambda a: float('nan'))
        self.assertIsNone(s.range_at(0.0))
        self.assertIsNone(s.front_clearance(.6, .25))
        self.assertEqual(sc.motion_check(s, .05, 0.0, self.geom, {'front': .05, 'rear': .05, 'sweep': .03}),
                         'SCAN_INVALID')

    def test_inf_is_valid_no_return(self):
        s = make_scan(lambda a: float('inf'))
        self.assertEqual(s.range_at(0.0), 8.0)
        self.assertEqual(s.front_clearance(.6, .25), math.inf)

    def test_front_wall(self):
        s = make_scan(lambda a: 1.0/math.cos(a) if abs(a) < 1.2 else float('inf'))   # wall x = 1
        self.assertAlmostEqual(s.front_clearance(.6, .25), .4, places=2)
        self.assertEqual(sc.motion_check(s, .05, 0.0, self.geom, {'front': .45, 'rear': .05, 'sweep': .03}),
                         'FRONT_STOP')

    def test_rotation_near_side_wall_allowed_but_close_wall_blocks(self):
        margins = {'front': .05, 'rear': .05, 'sweep': .03}
        side = make_scan(lambda a: .55/math.sin(a) if .3 < a < 2.8 else float('inf'))   # wall y = +0.55
        self.assertIsNone(sc.motion_check(side, 0.0, -.3, self.geom, margins))
        close = make_scan(lambda a: .30/math.sin(a) if .3 < a < 2.8 else float('inf'))  # wall y = +0.30
        self.assertEqual(sc.motion_check(close, 0.0, .3, self.geom, margins), 'SWEEP_STOP')

    def test_turning_away_from_a_close_post_is_allowed(self):
        margins = {'front': .05, 'rear': .05, 'sweep': .03}
        geom = {'front_extent': .6, 'rear_extent': .33, 'half_width': .22}
        bearing, dist = math.atan2(.24, .62), math.hypot(.62, .24)      # post 2.8 cm off the front-left corner
        post = make_scan(lambda a: dist if abs(a-bearing) < math.radians(1) else float('inf'), n=720)
        turns = [sc.motion_check(post, 0.0, w, geom, margins) for w in (.3, -.3)]
        self.assertEqual(turns, ['SWEEP_STOP', None])     # left swings the corner into it, right moves away

    def test_return_entering_the_body_within_one_period_is_blocked(self):
        # Codex 3rd review counterexample: a return just outside the front-left corner, w = 0.3
        geom = {'front_extent': .36, 'rear_extent': .327, 'half_width': .219}
        bearing, dist = math.atan2(.22, .30), math.hypot(.30, .22)
        s = make_scan(lambda a: dist if abs(a-bearing) < math.radians(1) else float('inf'), n=720)
        self.assertEqual(sc.motion_check(s, 0.0, .3, geom, {'front': .05, 'rear': .05, 'sweep': .03}), 'SWEEP_STOP')

    def test_curve_into_a_return_beside_the_path_is_blocked(self):
        # Codex 3rd review counterexample: (0.352, 0.282) clear for pure rotation, hit along the v=0.08, w=0.3 arc
        geom = {'front_extent': .36, 'rear_extent': .327, 'half_width': .219}
        bearing, dist = math.atan2(.282, .352), math.hypot(.352, .282)
        s = make_scan(lambda a: dist if abs(a-bearing) < math.radians(1) else float('inf'), n=720)
        self.assertEqual(sc.motion_check(s, .08, .3, geom, {'front': .05, 'rear': .05, 'sweep': .03}), 'SWEEP_STOP')

    def test_blind_sector_blocks_rotation(self):
        s = make_scan(lambda a: float('nan') if abs(a-1.0) < math.radians(25) else float('inf'))
        self.assertEqual(sc.motion_check(s, 0.0, .3, self.geom, {'front': .05, 'rear': .05, 'sweep': .03}),
                         'SCAN_INVALID')

    def test_person_is_not_a_jamb(self):
        def scene(a):                                      # open door (x = 0.7, |y| <= 0.5) and a person at
            if abs(a+.25) < math.radians(6):               # (0.45, -0.12), nearer than the right jamb
                return .47
            y = .7*math.tan(a) if abs(a) < 1.4 else None
            return float('inf') if y is None or -.5 <= y <= .5 else .7/math.cos(a)
        w = World()
        io = io_for(w)
        io.base.scan = lambda: make_scan(scene, n=720)
        self.assertIsNone(ElevatorEntry(io, EntryTests.cfg, echo=None).door_from_scan(1.0))

    def test_opening_independent_of_scan_origin(self):
        def door(a):
            y = .7*math.tan(a) if abs(a) < 1.4 else None
            return float('inf') if y is None or -.4 <= y <= .6 else .7/math.cos(a)
        n = 720
        a = sc.Scan(-math.pi, 2*math.pi/n, [door(-math.pi+2*math.pi*i/n) for i in range(n)], .15, 8.0).opening(1.0)
        b = sc.Scan(0.0, 2*math.pi/n, [door(math.atan2(math.sin(2*math.pi*i/n), math.cos(2*math.pi*i/n)))
                                        for i in range(n)], .15, 8.0).opening(1.0)
        self.assertAlmostEqual(a['centre_y'], b['centre_y'], delta=.005)

    def test_curved_motion_near_a_side_wall_is_swept(self):
        margins = {'front': .05, 'rear': .05, 'sweep': .03}
        side = make_scan(lambda a: .26/math.sin(a) if .3 < a < 2.8 else float('inf'))   # wall y = +0.26
        for v in (.05, -.05):
            self.assertEqual(sc.motion_check(side, v, .3, self.geom, margins), 'SWEEP_STOP')

    def test_blind_gap_behind_is_unknown_not_clear(self):
        def ranges(a):
            if abs(abs(a)-math.pi) < math.radians(15):
                return float('nan')                      # 30 deg of invalid beams straight behind
            return float('inf')
        s = make_scan(ranges)
        self.assertIsNone(s.rear_clearance(.33, .22))
        self.assertEqual(sc.motion_check(s, -.05, 0.0, self.geom, {'front': .05, 'rear': .05, 'sweep': .03}),
                         'SCAN_INVALID')

    def test_wall_fit_ignores_the_adjacent_wall_at_a_corner(self):
        # button wall x = 0.8 ahead; a side wall y = +0.35 meets it inside the search sector (cabin corner)
        def corner(a):
            front = .8/math.cos(a) if math.cos(a) > 1e-6 else math.inf
            side = .35/math.sin(a) if math.sin(a) > 1e-6 else math.inf
            return min(front, side)+.008*math.sin(37*a)            # ~1 cm LiDAR noise
        q, n, info = make_scan(corner, n=720).wall(.2, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(float(n[0]), -1.0, places=2)
        self.assertAlmostEqual(float(q[0]), .8, delta=.01)
        q, n, _ = make_scan(corner, n=720).wall(.2, (1.0, 2.0, math.pi/2))   # robot at (1, 2) facing +y
        self.assertAlmostEqual(float(q[1]), 2.8, delta=.01)
        self.assertAlmostEqual(float(n[1]), -1.0, places=2)

    def test_wall_fit_refuses_a_ray_right_at_the_corner(self):
        def corner(a):
            front = .8/math.cos(a) if math.cos(a) > 1e-6 else math.inf
            side = .35/math.sin(a) if math.sin(a) > 1e-6 else math.inf
            return min(front, side)+.008*math.sin(37*a)
        res = make_scan(corner, n=720).wall(.40, (0.0, 0.0, 0.0))          # hits x=0.8 about 1 cm from the corner
        self.assertTrue(res is None or abs(float(res[1][0])+1.0) < .02)    # never the adjacent y wall

    def test_door_opening_centre_from_the_jambs(self):
        def door(a):                                       # wall x = 0.7 with a 1.0 m opening centred at y = +0.1
            y = .7*math.tan(a) if abs(a) < 1.4 else None
            if y is None:
                return float('inf')
            return float('inf') if -.4 <= y <= .6 else .7/math.cos(a)
        gap = make_scan(door, n=720).opening(1.0)
        self.assertAlmostEqual(gap['width'], 1.0, delta=.02)
        self.assertAlmostEqual(gap['centre_y'], .1, delta=.01)
        self.assertIsNone(make_scan(lambda a: 1.0/math.cos(a) if abs(a) < 1.4 else float('inf')).opening(1.2))

    def test_door_from_scan_in_odom(self):
        def door(a):
            y = .7*math.tan(a) if abs(a) < 1.4 else None
            return float('inf') if y is None or -.4 <= y <= .6 else .7/math.cos(a)
        w = World()
        w.pose = (1.0, 2.0, math.pi/2)                    # facing +y
        io = io_for(w)
        io.base.scan = lambda: make_scan(door, n=720)
        seen = ElevatorEntry(io, EntryTests.cfg, echo=None).door_from_scan(1.0)
        self.assertAlmostEqual(seen['door'][0], .9, delta=.01)    # opening centre 0.1 m to the robot's left = -x
        self.assertAlmostEqual(seen['door'][1], 2.7, delta=.01)
        self.assertAlmostEqual(seen['n'][1], 1.0, delta=.01)       # through the door = the robot's forward (+y)

    def test_wall_fit_needs_a_return_on_the_ray(self):
        s = make_scan(lambda a: .8/math.cos(a) if a < 0 else float('inf'))
        self.assertIsNone(s.wall(.3, (0.0, 0.0, 0.0)))


class FreshTests(unittest.TestCase):
    def test_fresh_allows_small_clock_skew_only(self):
        self.assertTrue(sc.fresh(10.05, 10.0, .5))         # sensor stamp slightly ahead of a 10 Hz clock
        self.assertFalse(sc.fresh(11.0, 10.0, .5))         # far in the future: not trusted
        self.assertFalse(sc.fresh(9.0, 10.0, .5))          # too old
        self.assertFalse(sc.fresh(float('nan'), 10.0, .5))


class ContractTests(unittest.TestCase):
    def test_arm_status_verdict(self):
        from skilltests.ros_adapters import arm_status_verdict
        ok = {'request_id': 'r1', 'event': 'completed', 'action': 'view', 'hardware_connected': True}
        self.assertEqual(arm_status_verdict(ok, 'r1', 'view', True), 'done')
        self.assertEqual(arm_status_verdict(dict(ok, simulation_mode=True), 'r1', 'view', True), 'failed')
        self.assertEqual(arm_status_verdict(dict(ok, hardware_connected=False), 'r1', 'view', True), 'failed')
        self.assertEqual(arm_status_verdict(dict(ok, event='cancelled'), 'r1', 'view', True), 'failed')
        self.assertEqual(arm_status_verdict(dict(ok, action='press'), 'r1', 'view', True), 'failed')
        self.assertIsNone(arm_status_verdict(dict(ok, request_id='r2'), 'r1', 'view', True))

    def test_floor_frames_strictly_fresh(self):
        from skilltests.ros_adapters import FloorFrames
        f = FloorFrames()

        def obs(seq, t, floor='F2', s='a'):
            return {'valid': True, 'stream_id': s, 'frame_seq': seq, 'captured_at': t, 'floor': floor}
        self.assertIsNone(f.accept(obs(10, 1.0)))          # first frame of a stream only sets the reference
        self.assertEqual(f.accept(obs(11, 1.1)), 'F2')
        self.assertIsNone(f.accept(obs(10, 1.2)))          # older sequence
        self.assertIsNone(f.accept(obs(11, 1.3)))          # repeated sequence
        self.assertIsNone(f.accept(obs(12, 1.4, s='b')))   # stream change resets
        f.cutoff = 5.0
        self.assertIsNone(f.accept(obs(13, 1.5, s='b')))   # captured before the posture settled

    def test_lift_step_plan_caps(self):
        from skilltests.ros_adapters import lift_step_plan
        self.assertAlmostEqual(lift_step_plan(.10, .10, .01, .30), .11)
        self.assertIsNone(lift_step_plan(.30, .30, .01, .305))


class GeometryTests(unittest.TestCase):
    def test_press_stance_lands_the_taught_tip_square_on_the_button(self):
        from skilltests.geometry import press_error, press_stance
        tip, tip_yaw = (0.388, 0.215), 0.628             # a cycle that presses forward-left
        pose = press_stance((2.0, 1.0), (0.0, -1.0), tip, tip_yaw, .01)   # wall below, facing -y
        err = press_error(pose, (2.0, 1.0), (0.0, -1.0), tip, tip_yaw)
        self.assertAlmostEqual(err['depth_m'], .01, places=6)
        self.assertAlmostEqual(err['lateral_m'], 0.0, places=6)
        self.assertAlmostEqual(err['yaw_err_rad'], 0.0, places=6)
        self.assertAlmostEqual(math.sin(pose[2]+tip_yaw), 1.0, places=6)    # tip points straight at the wall (+y)

    def test_face_wall_stance_aligns_mount(self):
        pose, reach = face_wall_stance((1.0, 2.0), (-1.0, 0.0), (.12, .15), .62)
        mount = from_frame(pose, (.12, .15))
        self.assertAlmostEqual(mount[1], 2.0)
        self.assertAlmostEqual(1.0-mount[0], .50)
        self.assertAlmostEqual(reach, .50)
        self.assertAlmostEqual(pose[2], 0.0)

    def test_pallet_dock(self):
        centre, dock = pallet_dock((0.0, 1.0), (0.0, 1.0), .4, .37)
        self.assertEqual(centre, (0.0, 1.2))
        self.assertAlmostEqual(dock[1], 1.2-.37)
        self.assertAlmostEqual(dock[2], math.pi/2)

    def test_door_axis_square_to_door(self):
        n, through, outside = door_axis((0.0, 0.0), (0.3, 1.2), .7, normal=(0.0, -1.0), through_m=.55)
        self.assertEqual(n, (0.0, 1.0))                    # normal flipped toward the cabin
        self.assertEqual(through, (0.0, .55))
        self.assertEqual(outside, (0.0, -.7))

    def test_rightmost(self):
        self.assertEqual(rightmost([(1, 1), (1, -1), (1, 0)], (0.0, 0.0, 0.0)), (1, -1))
        self.assertAlmostEqual(to_frame((0, 0, math.pi/2), (0, 1))[0], 1.0)


class VisionCoreTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest('OpenCV unavailable')

    def test_projection_round_trip(self):
        from skilltests import vision_core as vc
        pose = vc.camera_pose((0.0, 0.0, .3), {'xyz': [.2, .15, .85], 'yaw_rad': -.4, 'pitch_rad': .5})
        fx = vc.fx_from_hfov(320, 1.13)
        p = (1.2, -.3, .05)
        u, v = vc.project(p, (320, 240), fx, pose)
        back = vc.on_plane_z(u, v, (320, 240), fx, pose, .05)
        self.assertLess(float(np.linalg.norm(back-np.array(p))), 1e-6)

    def test_seven_segment(self):
        from skilltests import vision_core as vc
        red = [[[0, 150, 120], [6, 255, 255]]]
        img = np.zeros((120, 120, 3), np.uint8)
        img[20:100, 55:63] = (0, 0, 255)
        self.assertEqual(vc.seven_segment(img, red, 15, .4)[0], '1')
        img[20:28, 30:90] = (0, 0, 255)
        img[56:64, 30:90] = (0, 0, 255)
        self.assertEqual(vc.seven_segment(img, red, 15, .4)[0], '2')


if __name__ == '__main__':
    unittest.main(verbosity=1)
