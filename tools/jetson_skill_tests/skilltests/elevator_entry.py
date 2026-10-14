"""Test 1 — elevator entry.

At the elevator entrance: switch to ELEVATOR_ENTRY_STATE, drive to the call-button area, find the button with the
arm camera, press it with the arm and confirm the press, then stand in front of the entrance on the door centre line
(square to the door) and enter as soon as the LiDAR sees the door open (robot ends at the cabin centre facing into
the cabin). If the door is not open in time, or closes in front of the robot while entering (front LiDAR stop), the
robot backs to the waiting spot and calls again (up to call_retries). Unknown LiDAR data never counts as open.
"""
import math

from .button import ButtonMixin
from .machine import Machine


class ElevatorEntry(ButtonMixin, Machine):
    name = 'elevator_entry'

    def steps(self):
        io, c = self.io, self.cfg
        self.enter('ELEVATOR_ENTRY_STATE', floor=c['floor'], call_button=c['call_label'])
        io.base.stop()
        for call in range(1+int(c['call_retries'])):
            area = io.base.map_to_odom(tuple(c['button_view_pose_map']))
            self.enter('GO_TO_BUTTON_AREA', call=call, target=[round(v, 3) for v in area])
            self.stow()
            self.drive(area, reverse='auto')            # back up if the spot is behind: less turning
            self.press_button('hall', c['call_label'], c['call_button_prior_map'], 'hall_view')
            n, through, outside, cabin = self.door_path(c['door_centre_map'], c['cabin_centre_map'],
                                                        c.get('door_normal_map'))
            heading = math.atan2(n[1], n[0])
            self.enter('RETURN_TO_DOOR', call=call, target=[round(v, 3) for v in outside])
            self.stow()
            self.drive((outside[0], outside[1], heading))
            opened = c['door_align_m']+c['door_open_margin_m']
            self.enter('WAIT_DOOR_OPEN', call=call, timeout_s=c['door_timeout_s'], open_range_m=round(opened, 3))
            # open = a plausible door-wide opening near the taught door, seen twice in a row (a far straight
            # range alone is not enough: people, glass, a skewed approach)
            door_o = ((through[0]-n[0]*c['door_through_m'], through[1]-n[1]*c['door_through_m']), n)
            self._door_streak = 0
            if not self.wait_for(lambda: self.door_open(opened, door_o), c['door_timeout_s']):
                self.note(door_range_m=round(self.range_ahead(), 3), result='not opened')
                continue
            self.note(door_range_m=round(self.range_ahead(), 3))
            # square up with the opening the LiDAR actually sees (map door axis: up to 0.2 m / 9 deg off near a lift)
            seen = self.door_from_scan(opened, door_o)      # jambs: closer than the open-door range
            if seen is not None:
                depth = (cabin[0]-through[0])*n[0]+(cabin[1]-through[1])*n[1]+c['door_through_m']
                d, n = seen['door'], seen['n']
                outside = (d[0]-n[0]*c['door_align_m'], d[1]-n[1]*c['door_align_m'])
                through = (d[0]+n[0]*c['door_through_m'], d[1]+n[1]*c['door_through_m'])
                cabin = (d[0]+n[0]*depth, d[1]+n[1]*depth)
                x, y, yaw = io.base.pose()
                new_heading = math.atan2(n[1], n[0])
                off = math.hypot(outside[0]-x, outside[1]-y)
                turn = math.atan2(math.sin(new_heading-yaw), math.cos(new_heading-yaw))
                heading = new_heading
                self.note(door_seen={'width_m': round(seen['width'], 3), 'offset_m': round(off, 3), 'turn_rad': round(turn, 3)})
                if off > c['door_centre_tol_m'] or abs(turn) > .03:
                    self.enter('ALIGN_TO_DOOR', offset_m=round(off, 3), turn_rad=round(turn, 3))
                    self.drive((outside[0], outside[1], heading), reverse='auto', max_v=c['slow_v'])
            self.enter('ENTER_CABIN', call=call, target=[round(v, 3) for v in through])
            result = io.base.go_to((through[0], through[1], heading), max_v=c['cabin_v'], min_front=c['front_stop_m'])
            self.note(drive=result)
            if result.get('status') == 'REACHED':
                self.go_to_cabin_centre(n, through, cabin, heading)
                self.enter('DONE')
                return
            self.enter('ENTRY_BLOCKED', call=call, drive=result)
            self.drive((outside[0], outside[1], heading), reverse=True, max_v=c['slow_v'])
        self.require(False, 'could not enter the cabin', calls=1+int(c['call_retries']))
