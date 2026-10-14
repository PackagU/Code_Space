"""Find a wall button with the arm camera and press it with a TAUGHT press cycle (fixed servo angles, as on the
Jetson): the robot stands so that the tip of the cycle's press pose lands on the button, square to the wall.

With a remembered confirmed press of this button (press_memory): drive straight to its line-up point and look from
there with the press posture; seen -> plan from that sighting and skip the separate line-up and second look, not
seen -> the search below from where the robot stands. The camera always confirms the button before a press.
Look (back off and look again if it is out of the picture) -> plan the stance from the cycle's press tip (robot
frame xy, height, pointing direction; `press_tip` per button kind) -> drive to a line-up point square to the wall
-> look again from there (the first estimate carries the odometry drift of the turns on the way; re-line-up if the
plan moved) -> drive straight in -> check where the tip will land (press envelope) -> press -> back off -> confirm
(`confirm_by`: the camera sees the lit button and/or the LiDAR sees the car door opening; an empty list accepts the
completed cycle, for buttons without a lamp) -> retry.
"""
import math

from .geometry import press_error, press_stance, rotate, to_frame, wall_gap


class ButtonMixin:
    def remembered_press(self, kind, label, prior_map_xyz, floor=None):
        """The last confirmed press of this button (map frame), if one is near the taught prior."""
        memory = getattr(self.io, 'press_memory', None)
        entry = memory.lookup(floor or self.cfg.get('floor'), kind, label) if memory else None
        return entry if entry and math.dist(entry['button_xyz'], prior_map_xyz) <= .6 else None

    def press_button(self, kind, label, prior_map_xyz, view, door_map=None, confirm_by=None, memory_floor=None):
        io, c = self.io, self.cfg
        door_odom = None if door_map is None else io.base.map_to_odom((door_map[0], door_map[1], 0.0))[:2]
        key = 'call' if kind == 'hall' else 'destination'
        tip = c['press_tip'][key]
        self.require(abs(tip['yaw']) <= .01, 'arm horizontal rotation is forbidden; teach a forward-only press')
        memory = getattr(io, 'press_memory', None)
        floor = memory_floor or c.get('floor')
        remembered = self.remembered_press(kind, label, prior_map_xyz, floor)
        if remembered:
            prior_map_xyz = remembered['button_xyz']
            self.note(remembered_press=remembered, camera_recheck_required=True)
        px, py, _ = io.base.map_to_odom((prior_map_xyz[0], prior_map_xyz[1], 0.0))
        prior = (px, py, prior_map_xyz[2])
        hint = {}
        if door_map is not None:
            hint['door_xy'] = io.base.map_to_odom((door_map[0], door_map[1], 0.0))[:2]

        def line_up_of(stance):
            back = rotate((-c['line_up_m'], 0.0), stance[2])
            return (stance[0]+back[0], stance[1]+back[1], stance[2])
        lineup_drive = dict(max_v=c['slow_v'], tol=.005, reaim_tol=.01, arrive_tol=.015, yaw_tol=.006,
                            yaw_arrive_tol=.012)
        button = lined_up = None
        if remembered:
            # straight to where this button was last pressed and confirmed; the camera must find it again from there
            target = line_up_of(io.base.map_to_odom(tuple(remembered['press_pose'])))
            self.enter('GO_TO_REMEMBERED_LINE_UP', target=[round(v, 3) for v in target], run=remembered.get('run'))
            self.stow()
            result = io.base.go_to(target, reverse='auto', **lineup_drive)
            self.note(drive=result)
            if result.get('status') in ('REACHED', 'POSITION_ERROR', 'YAW_ERROR'):   # near enough to look
                seen = io.arm.view(c['press_view'][key], prior)
                if seen.get('reached'):
                    self.current_view = c['press_view'][key]
                    button = io.vision.find_button(kind, label, prior, hint)
                elif seen.get('in_view') is not False:   # the posture itself failed: never drive on
                    self.require(False, 'arm view posture not reached', view=c['press_view'][key], arm=seen)
            lined_up = io.base.pose() if button is not None else None
            self.note(seen=button is not None)
        steps = int(c['look_back_steps'])
        for step in range(steps+1 if button is None else 0):
            if step == 0:
                self.enter('LOOK_FOR_BUTTON', kind=kind, label=label, arm_view=view)
            seen = io.arm.view(view, prior)
            if seen.get('reached'):
                self.current_view = view
                button = io.vision.find_button(kind, label, prior, hint)
                if button is not None:
                    break
            elif seen.get('in_view') is not False:      # the posture itself failed: never drive on
                self.require(False, 'arm view posture not reached', view=view, arm=seen)
            if step < steps:                             # out of the picture or not found: back off, look again
                self.enter('BACK_OFF_TO_SEE', step=step, arm=seen, distance_m=c['look_back_m'])
                if not self.back_off(c['look_back_m'], blocked_ok=True):
                    self.note(stopped='no room to back off further')
                    break
                self.enter('LOOK_FOR_BUTTON', kind=kind, label=label, arm_view=view, look=step+1)
        self.require(button is not None, 'the arm camera did not find the button', looks=steps+1)
        self.note(button={k: button[k] for k in button if k in ('xyz', 'normal', 'seen', 'wall_fit', 'pixel')})
        g = io.base.geometry()
        outline = [(g['front_extent'], g['half_width']), (g['front_extent'], -g['half_width']),
                   (-g['rear_extent'], g['half_width']), (-g['rear_extent'], -g['half_width'])]

        def plan(b):
            """Stance for the taught press; refuses a button the cycle cannot reach (height, or the robot front
            would have to be closer to the wall than the front stop)."""
            dz = b['xyz'][2]-tip['z']
            self.require(abs(dz) <= c['press_height_tol_m'], 'button height outside the taught press',
                         button_z=round(b['xyz'][2], 3), tip_z=tip['z'])
            stance = press_stance(b['xyz'][:2], b['normal'], tip['xy'], tip['yaw'], c['press_push_m'])
            gap = wall_gap(stance, outline, b['xyz'][:2], b['normal'])
            self.require(gap >= c['front_stop_m'], 'the taught press needs the robot closer to the wall than the '
                         'front stop allows', gap_m=round(gap, 3), front_stop_m=c['front_stop_m'])
            return stance, gap
        stance, gap = plan(button)
        for attempt in range(1+int(c['press_retries'])):
            line_up = line_up_of(stance)
            self.enter('APPROACH_BUTTON', attempt=attempt, stance=[round(v, 3) for v in stance], wall_gap_m=round(gap, 3),
                       cycle_tip=tip)
            self.stow()
            here = lined_up if attempt == 0 else None
            if here is not None and math.hypot(here[0]-line_up[0], here[1]-line_up[1]) <= c['reaim_m'] and \
                    abs(math.atan2(math.sin(here[2]-line_up[2]), math.cos(here[2]-line_up[2]))) <= \
                    c['press_envelope_m'][2]:
                self.note(line_up='already there: planned from the sighting at the remembered line-up')
            else:
                # a few mrad of heading left at the line-up point are taken care of by the re-look below and the
                # squaring/envelope check before the press (same as LINE_UP_AGAIN); a 0.0121 rad finish against the
                # 0.012 rad arrival tolerance aborted a whole classroom run (2026-10-14)
                lined = self.drive(line_up, reverse='auto', accept=('REACHED', 'YAW_ERROR'), **lineup_drive)
                self.require(lined.get('status') == 'REACHED' or abs(lined.get('yaw_err_rad', 1.0)) <=
                             c.get('lineup_yaw_slack_rad', .03), 'line-up heading error too large', result=lined)
                # Look again from the line-up point: the estimate no longer carries the odometry drift of the turns on
                # the way here. From the stance itself the arm-tip camera is too close to see the button. After a
                # re-line-up look once more: its turns drift the odometry heading too (0.04 rad = 2 cm at the tip).
                for look in range(3):
                    self.enter('RECHECK_BUTTON', attempt=attempt, look=look)
                    again_view = c['press_view'][key]               # fixed posture along the press direction
                    seen = io.arm.view(again_view, tuple(button['xyz']))
                    again = io.vision.find_button(kind, label, tuple(button['xyz']), hint) if seen.get('reached') \
                        else None
                    if again is not None:
                        shift = math.hypot(again['xyz'][0]-button['xyz'][0], again['xyz'][1]-button['xyz'][1])
                        turn = math.acos(max(-1.0, min(1.0, again['normal'][0]*button['normal'][0]
                                                       + again['normal'][1]*button['normal'][1])))
                        self.note(shift_m=round(shift, 3), turn_rad=round(turn, 3),
                                  xyz=[round(v, 3) for v in again['xyz']])
                        lim = c['recheck_max']
                        if shift > lim[0] or abs(again['xyz'][2]-button['xyz'][2]) > lim[1] or turn > lim[2]:
                            self.note(rejected='not the same button (moved too far)')   # keep the last estimate
                            again = None
                    if again is None:
                        self.note(seen=False, arm=seen)   # keep the last estimate; the envelope check still applies
                        break
                    button = again
                    stance, gap = plan(button)
                    moved = line_up_of(stance)
                    if (math.hypot(moved[0]-line_up[0], moved[1]-line_up[1]) <= c['reaim_m'] and
                            abs(math.atan2(math.sin(moved[2]-line_up[2]), math.cos(moved[2]-line_up[2]))) <=
                            c['press_envelope_m'][2]) or look == 2:
                        break
                    self.enter('LINE_UP_AGAIN', attempt=attempt, line_up=[round(v, 3) for v in moved])
                    self.stow()
                    # turns happen here, away from the wall; a blocked last few deg are left to the envelope check
                    self.note(drive=self.drive(moved, reverse='auto', accept=('REACHED', 'SWEEP_STOP'),
                                               **lineup_drive))
                    line_up = moved
            self.stow()
            self.drive(stance, accept=('REACHED', 'FRONT_STOP', 'SWEEP_STOP'), max_v=c['slow_v'],
                       max_w=.08, tol=.003, reaim_tol=.006, arrive_tol=.01, final_yaw=False,
                       min_front=c['front_stop_m'])
            envelope = self.stance_error(button, tip)
            self.note(envelope=envelope)
            env = c['press_envelope_m']
            # A few degrees of heading error moves the fixed forward tip
            # sideways by several cm. Square the BASE (arm stays stowed and
            # yaw-locked) before making axial depth corrections.
            for correction in range(2):
                if abs(envelope['yaw_err_rad']) <= .01 or abs(envelope['yaw_err_rad']) > .12:
                    break
                x, y, _ = io.base.pose()
                heading = math.atan2(-button['normal'][1], -button['normal'][0])-tip['yaw']
                self.enter('SQUARE_TO_BUTTON', attempt=attempt, correction=correction)
                result = self.drive((x,y,heading), accept=('REACHED','SWEEP_STOP','YAW_ERROR'),
                                    max_w=.06, tol=.003, arrive_tol=.015, yaw_tol=.004, yaw_arrive_tol=.008)
                envelope = self.stance_error(button, tip)
                self.note(envelope=envelope, square_drive=result)
            # moving d along the heading changes the depth by d*cos(a) and the lateral offset by d*sin(a), with a the
            # angle between the heading and the wall's inward normal
            correction_total = 0.0
            for correction in range(3):
                a = math.atan2(math.sin(io.base.pose()[2]-math.atan2(-button['normal'][1], -button['normal'][0])),
                               math.cos(io.base.pose()[2]-math.atan2(-button['normal'][1], -button['normal'][0])))
                delta = (envelope['depth_m']-c['press_push_m'])/max(math.cos(a), .3)
                lateral_after = envelope['lateral_m']-delta*math.sin(a)
                if envelope['ok'] or abs(lateral_after) > env[1] or abs(envelope['yaw_err_rad']) > env[2]                         or abs(delta) > c['depth_adjust_max_m']:
                    break
                # Closed-loop small axial corrections; do not turn during a cm-scale
                # correction or accept a loose 6 cm arrival tolerance as alignment.
                delta = max(-.02, min(.02, delta))
                correction_total += abs(delta)
                if correction_total > c['depth_adjust_max_m']:
                    break
                self.enter('ADJUST_DEPTH', attempt=attempt, correction=correction, move_m=round(-delta, 3))
                x, y, yaw = io.base.pose()
                result = self.drive((x-math.cos(yaw)*delta, y-math.sin(yaw)*delta, yaw), reverse=delta > 0,
                           accept=('REACHED', 'FRONT_STOP', 'POSITION_ERROR'), max_v=.012, tol=.003,
                           reaim_tol=0.0, final_yaw=False, retries=0, arrive_tol=.008,
                           min_front=c['front_stop_m'])
                envelope = self.stance_error(button, tip)
                self.note(envelope=envelope, correction_drive=result)
            if not envelope['ok']:
                self.enter('STANCE_OUT_OF_ENVELOPE', attempt=attempt, envelope=envelope)
                self.back_off()
                continue
            self.enter('PRESS_BUTTON', attempt=attempt)
            self._press_pose = tuple(io.base.pose())
            candidate = memory.capture(floor, kind, label, button, self._press_pose, tip) if memory else None
            pressed = io.arm.press(button)
            self.note(press=pressed)
            self.require(pressed.get('done'), 'arm press motion failed', press=pressed)
            self.enter('BACK_OFF', distance_m=c['back_off_m'])
            self.back_off()
            if self.confirm(kind, view, button, attempt, door_odom, confirm_by, pressed.get('camera_confirmation')):
                if candidate is not None:
                    memory.commit(candidate)
                    self.note(press_position_saved=candidate)
                return button
        self.require(False, 'button press not confirmed', attempts=1+int(c['press_retries']))

    def confirm(self, kind, view, button, attempt, door=None, by=None, camera=None):
        """Confirm the requested cap: lamp image, optional door response, or configured cap/alignment recheck.
        The floor display is used only later for arrival; it cannot confirm this press.
        Lamp-free alignment evidence is an estimate, not force or switch feedback.
        """
        io, c = self.io, self.cfg
        by = list(c['confirm_by']['call' if kind == 'hall' else 'destination'] if by is None else by)
        self.enter('CONFIRM_PRESS', attempt=attempt, by=by)
        if 'lamp' in by and isinstance(camera, dict) and camera.get('source') == 'arm_camera' and camera.get('lit') is True and str(camera.get('label')) == str(button['label']):
            self.note(lit=True, evidence=camera)
            return True
        if c.get('press_visual_required', False):
            self.enter('POST_PRESS_LOOK', attempt=attempt, camera='existing arm-tip camera')
            seen = io.arm.view(c['press_view']['call' if kind == 'hall' else 'destination'], tuple(button['xyz']))
            again = io.vision.find_button(kind, button['label'], tuple(button['xyz'])) if seen.get('reached') else None
            if again is None:
                self.note(camera_verified=False, why='requested cap not reacquired after backing off')
                return False
            tip = c['press_tip']['call' if kind == 'hall' else 'destination']
            err = press_error(self._press_pose, again['xyz'][:2], again['normal'], tip['xy'], tip['yaw'])
            env = c['press_envelope_m']
            aligned = (abs(err['depth_m']-c['press_push_m']) <= env[0]
                       and abs(err['lateral_m']) <= env[1] and abs(err['yaw_err_rad']) <= env[2])
            self.note(camera_verified=aligned, actual_press_pose=list(self._press_pose), error=err,
                      evidence='reacquired cap + recorded press pose (alignment estimate, not force feedback)')
            if not aligned:
                return False
        if not by:
            self.note(accepted='press cycle completed (no lamp to check)')
            return True
        responded = 'door' in by and self.door_responding(door)
        lit, looking = False, False
        if not responded and 'lamp' in by:
            # look along the press direction (the stance may be turned for the taught cycle)
            seen = io.arm.view(c['press_view']['call' if kind == 'hall' else 'destination'], tuple(button['xyz']))
            if seen.get('reached'):
                looking = True
            elif seen.get('in_view') is not False:
                self.require(False, 'arm view posture not reached', arm=seen)
            else:
                self.note(lamp_out_of_view=seen)
        end = io.clock.now()+c['confirm_window_s']
        while not responded:
            lit = looking and io.vision.button_lit(button)
            responded = 'door' in by and self.door_responding(door)
            if lit or responded or io.clock.now() >= end:
                break
            io.clock.sleep(.5)
        self.note(lit=lit, door_responding=responded)
        return bool(lit or responded)

    def back_off(self, distance=None, blocked_ok=False):
        """Reverse straight back; True if the whole distance was driven (a blocked rear is a failure unless
        blocked_ok, then False)."""
        self.stow()
        x, y, yaw = self.io.base.pose()
        back = rotate((-(self.cfg['back_off_m'] if distance is None else distance), 0.0), yaw)
        result = self.drive((x+back[0], y+back[1], yaw), reverse=True, max_v=self.cfg['cabin_v'],
                            accept=('REACHED', 'REAR_STOP') if blocked_ok else ('REACHED',))
        return result.get('status') == 'REACHED'

    def stance_error(self, button, tip):
        """Where the taught press tip will land from the stopped pose: depth past the button face against the
        planned push, lateral offset on the wall and heading error, against `press_envelope_m`."""
        c = self.cfg
        err = press_error(self.io.base.pose(), button['xyz'][:2], button['normal'], tip['xy'], tip['yaw'])
        env = c['press_envelope_m']
        ok = (abs(err['depth_m']-c['press_push_m']) <= env[0] and abs(err['lateral_m']) <= env[1]
              and abs(err['yaw_err_rad']) <= env[2])
        return {'ok': ok, **{k: round(v, 3) for k, v in err.items()}}

    def door_responding(self, door=None):
        """LiDAR sees past the door centre (odom `door`, default this floor's door from the config)."""
        io = self.io
        if door is None:
            centre = self.cfg.get('door_centre_map')
            if centre is None:
                return False
            door = io.base.map_to_odom((centre[0], centre[1], 0.0))[:2]
        scan, pose = io.base.scan(), io.base.pose()
        if scan is None or pose is None:
            return False
        # bearing and expected distance from the LiDAR origin (the scan's ranges are measured from there)
        fx, fy = to_frame(pose, door)
        lx, ly = fx-scan.lx, fy-scan.ly
        r = scan.range_at(math.atan2(ly, lx))
        # seen from beside the door the open car is shallow at that angle: a smaller margin than at the door
        return r is not None and r >= math.hypot(lx, ly)+self.cfg['door_confirm_margin_m']
