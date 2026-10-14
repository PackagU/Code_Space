"""Test 2 — floor button inside the cabin and exit.

Phase 'press' (right after entering, robot at the cabin centre facing in): switch to ELEVATOR_RIDE_STATE, turn right
90 deg to face the panel wall, take the in-cabin arm posture, find the target-floor button with the camera, drive as
close as possible, press it with the arm, confirm the lit button, reverse to the cabin centre, turn right 90 deg to
face the door and raise the arm so the camera sees the floor display above the entrance.
Phase 'exit' (also usable alone): (re)take the display posture, read the display until the target floor is seen in
N consecutive fresh frames, run `before_exit` (target-floor map switch + localisation check), face the door, and
right before leaving confirm the floor again, wait for the LiDAR to see the door open, stow the arm and drive out
along the door normal. If the door stays closed, press this floor's button again to reopen it.
"""
import math

from .button import ButtonMixin
from .machine import Machine


class ElevatorRide(ButtonMixin, Machine):
    name = 'elevator_ride'

    def press(self):
        io, c = self.io, self.cfg
        self.enter('ELEVATOR_RIDE_STATE', floor=c['floor'], target_floor=c['target_floor'])
        io.base.stop()
        self.stow()
        self.enter('TURN_RIGHT_90', purpose='face the button wall')
        turn = io.base.turn_by(-math.pi/2)
        self.require(turn.get('status') == 'REACHED', 'turn failed', result=turn)
        centre_x, centre_y, wall_heading = io.base.pose()   # where the entry left it (centred), facing the wall
        self.press_button('car', c['target_label'], c['target_button_prior_map'], 'cabin_view',
                          door_map=c['door_centre_map'])
        cx, cy = centre_x, centre_y
        self.enter('BACK_TO_CABIN_CENTRE', target=[round(cx, 3), round(cy, 3)])
        self.stow()
        self.drive((cx, cy, wall_heading), reverse='auto', max_v=c['slow_v'], reaim_tol=.05, arrive_tol=.05)
        self.enter('TURN_RIGHT_90', purpose='face the door to get ready to exit')
        turn = io.base.turn_by(-math.pi/2)
        self.require(turn.get('status') == 'REACHED', 'turn failed', result=turn)
        self.display_posture('display_prior_map')

    def display_posture(self, key):
        """Raise the arm so the camera sees the floor display. `key` names the display position in the map that is
        active now (departure floor before the map switch, arrival floor after it)."""
        c = self.cfg
        # the display is above the door the robot now faces: straight ahead, at the taught height (a map prior
        # would be wrong once the car has moved before the map switch)
        x, y, yaw = self.io.base.pose()
        d = c['display_ahead_m']
        self.enter('DISPLAY_ARM_POSTURE', arm='raised to see the floor display')
        self.view('display_view', (x+math.cos(yaw)*d, y+math.sin(yaw)*d, c[key][2]))

    def confirm_floor(self, frames, timeout):
        """True once the target floor is read in `frames` consecutive fresh frames (any other reading resets)."""
        io, c = self.io, self.cfg
        streak, last, seen = 0, None, []
        end = io.clock.now()+timeout
        while io.clock.now() < end and streak < frames:
            last = io.vision.floor()
            seen.append(last)
            streak = streak+1 if last == c['target_floor'] else 0
            io.clock.sleep(c['floor_period_s'])
        self.note(last_reading=last, streak=streak, readings=seen[-12:])
        return streak >= frames

    def exit(self, before_exit=None):
        io, c = self.io, self.cfg
        if self.current_view != 'display_view':           # exit used on its own: take the posture first
            self.display_posture('display_prior_map')
        self.enter('WAIT_TARGET_FLOOR', target_floor=c['target_floor'], frames=c['floor_confirm_frames'])
        self.require(self.confirm_floor(int(c['floor_confirm_frames']), c['floor_timeout_s']),
                     'target floor not confirmed by the camera')
        if before_exit is not None:
            self.enter('BEFORE_EXIT_CHECK', check='target-floor map and localisation')
            before_exit()
        n, through, outside, cabin = self.door_path(c['exit_door_centre_map'], c['exit_cabin_centre_map'],
                                                    c.get('exit_door_normal_map'))
        out_heading = math.atan2(-n[1], -n[0])
        door = io.base.map_to_odom((c['exit_door_centre_map'][0], c['exit_door_centre_map'][1], 0.0))
        reopens = int(c['call_retries'])
        for attempt in range(reopens+1):
            x, y, yaw = io.base.pose()
            if abs(math.atan2(math.sin(out_heading-yaw), math.cos(out_heading-yaw))) > .05:
                self.enter('FACE_DOOR', attempt=attempt)
                self.stow()                              # never turn the base with the arm raised
                self.drive((x, y, out_heading))
                self.display_posture('exit_display_prior_map')
            self.enter('RECONFIRM_FLOOR', attempt=attempt)
            if not self.confirm_floor(int(c['floor_reconfirm_frames']), c['floor_timeout_s']):
                self.require(False, 'floor changed or unreadable before leaving the cabin')
            x, y, _ = io.base.pose()
            opened = math.hypot(door[0]-x, door[1]-y)+c['door_open_margin_m']
            self.enter('WAIT_DOOR_OPEN', attempt=attempt, timeout_s=c['door_timeout_s'], open_range_m=round(opened, 3))
            door_o = (door[:2], (-n[0], -n[1]))             # facing out: through the door = out of the cabin
            self._door_streak = 0
            if self.wait_for(lambda: self.door_open(opened, door_o), c['door_timeout_s']):
                seen = self.door_from_scan(opened, door_o)  # the opening as the LiDAR sees it on this floor
                if seen is not None:
                    d, m = seen['door'], seen['n']          # m points out of the cabin
                    n = (-m[0], -m[1])
                    outside = (d[0]+m[0]*c['door_align_m'], d[1]+m[1]*c['door_align_m'])
                    through = (d[0]-m[0]*c['door_through_m'], d[1]-m[1]*c['door_through_m'])
                    out_heading = math.atan2(m[1], m[0])
                    self.note(door_seen={'width_m': round(seen['width'], 3)})
                self.enter('EXIT_CABIN', attempt=attempt, target=[round(v, 3) for v in outside])
                self.stow()
                x, y, _ = io.base.pose()
                lateral = (x-outside[0])*-math.sin(out_heading)+(y-outside[1])*math.cos(out_heading)
                result = {'status': 'REACHED'}
                if abs(lateral) > c['door_lateral_tol_m']:          # well off the opening's axis: onto it first
                    result = io.base.go_to((through[0], through[1], out_heading), max_v=c['slow_v'])
                    self.note(onto_axis=result, lateral_m=round(lateral, 3))
                    x, y, _ = io.base.pose()
                if result.get('status') == 'REACHED':
                    # straight out along the robot's own track parallel to the door axis (no diagonal crossing)
                    ux, uy = math.cos(out_heading), math.sin(out_heading)
                    along = (outside[0]-x)*ux+(outside[1]-y)*uy
                    result = io.base.go_to((x+ux*along, y+uy*along, out_heading), max_v=c['cabin_v'],
                                           min_front=c['front_stop_m'])
                self.note(drive=result)
                if result.get('status') == 'REACHED':
                    exit_pose = io.base.map_to_odom(tuple(c['exit_pose_map']))
                    self.enter('GO_TO_EXIT_POSE', target=[round(v, 3) for v in exit_pose])
                    # out of the cabin already: get clear of the door and take the taught exit heading (Nav2
                    # starts from it); a turn blocked both ways next to the landing walls is left to Nav2
                    self.note(drive=self.drive(exit_pose, max_v=c['cabin_v'],
                                               accept=('REACHED', 'SWEEP_STOP', 'FRONT_STOP')))
                    self.enter('DONE')
                    return
                self.enter('EXIT_BLOCKED', drive=result)
                self.drive((cabin[0], cabin[1], out_heading), reverse=True, max_v=c['slow_v'])
            if attempt == reopens:
                break
            # door closed with the robot inside: press this floor's button again (reopens at the current floor);
            # the panel is on the wall to the left when facing the door
            self.enter('REOPEN_DOOR', attempt=attempt)
            self.stow()
            turn = io.base.turn_by(math.pi/2)
            self.require(turn.get('status') == 'REACHED', 'turn to the button wall failed', result=turn)
            # confirmation as configured; the door wait that follows (facing the door again) is the real check
            self.press_button('car', c['target_label'], c['exit_button_prior_map'], 'cabin_view',
                              door_map=c['exit_door_centre_map'], memory_floor=c['target_floor'])
            self.enter('BACK_TO_CABIN_CENTRE', target=[round(v, 3) for v in cabin])
            self.stow()
            self.drive((cabin[0], cabin[1], out_heading))
            self.display_posture('exit_display_prior_map')
        self.require(False, 'could not leave the cabin', reopen_presses=reopens)

    def steps(self, before_exit=None):
        self.press()
        self.exit(before_exit=before_exit)
