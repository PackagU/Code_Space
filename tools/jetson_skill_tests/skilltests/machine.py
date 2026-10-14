"""State-machine base for the skill tests. The same machine runs in the simulation (sim/real_maps/sim_adapters.py)
and on the Jetson (skilltests/ros_adapters.py); only the adapters differ.

Adapter contract (duck typed; poses in the odometry frame unless noted):
  base.pose() -> (x, y, yaw) | None (stale)      base.map_to_odom((x, y, yaw)) -> (x, y, yaw)
  base.go_to((x, y, yaw), min_front=None, **kw) -> {'status', ...}   (every command is safety checked)
  base.turn_by(rad) -> {...}   base.stop()   base.front_extent() -> m   base.set_loaded(bool)
  base.range_at(bearing_rad) -> m | None (unknown)   base.wall(bearing_rad) -> (point, normal, info) | None
  arm.posture('stow') -> {'reached'}             arm.view(name, (x, y, z)) -> {'reached', ...}
                                                 (simulation aims at the point; the robot uses the taught view `name`)
  arm.press(button) -> {'done', ...}
  vision.find_button(kind, label, prior_xyz, hint) -> {'xyz', 'normal', ...} | None
  vision.button_lit(button) -> bool              vision.parcels() -> [{'xyz', ...}]
  vision.pallet_face(near_xy) -> {'mid', 'normal', 'width_m'} | None
  vision.floor() -> 'F1' | 'F2' | ... | None     (one fresh reading; repeated or stale frames return None)
  lift.ready() -> {'ok', ...}   lift.lower() -> {'reached'}   lift.raise_for_carry() -> {'reached', 'load_top_m'}
  load.attach() -> {'ok', ...}   load.detach() -> {'ok', ...}
  clock.now() -> s   clock.sleep(s)
"""
import json
import math
import time


class TestAborted(RuntimeError):
    pass


class Machine:
    name = 'machine'

    def __init__(self, io, cfg, log_path=None, echo=print):
        self.io, self.cfg, self.log_path, self.echo = io, cfg, log_path, echo
        self.state, self.events, self._open = 'IDLE', [], None
        self.current_view = None

    # ------------------------------------------------------------ records
    def _flush(self):
        """Write the previous state record once its evidence (note()) is complete."""
        if self._open is not None and self.log_path:
            with open(self.log_path, 'a', encoding='utf-8') as handle:
                handle.write(json.dumps(self._open, default=str, ensure_ascii=False)+'\n')
        self._open = None

    def enter(self, state, **data):
        self._flush()
        self.state = state
        event = {'test': self.name, 'state': state, 't': round(self.io.clock.now(), 3),
                 'wall': round(time.time(), 3), **data}
        self.events.append(event)
        self._open = event
        if self.echo:
            self.echo('STATE '+json.dumps(event, default=str, ensure_ascii=False))

    def note(self, **data):
        self.events[-1].update(data)

    def require(self, ok, why, **data):
        if not ok:
            raise TestAborted(f'{self.state}: {why} {json.dumps(data, default=str, ensure_ascii=False)}'.strip())

    # ------------------------------------------------------------ guarded actions
    def drive(self, target, accept=('REACHED',), **kw):
        result = self.io.base.go_to(target, **kw)
        self.require(result.get('status') in accept, 'drive did not reach the target', result=result)
        return result

    def stow(self):
        """The arm must be in its verified safe posture before any base motion."""
        result = self.io.arm.posture('stow')
        self.require(result.get('reached'), 'arm not stowed', arm=result)
        self.current_view = None
        return result

    def view(self, name, target):
        result = self.io.arm.view(name, target)
        self.require(result.get('reached'), 'arm view posture not reached', view=name, arm=result)
        self.current_view = name
        return result

    def range_ahead(self):
        r = self.io.base.range_at(0.0)
        return -1.0 if r is None else r          # unknown counts as blocked / not open

    def run(self, phase='steps', **kw):
        try:
            getattr(self, phase)(**kw)
        except TestAborted as exc:
            self.io.base.stop()
            self.enter('FAILED', error=str(exc))
            raise
        except Exception as exc:     # keep the robot stopped on any unexpected error
            self.io.base.stop()
            self.enter('FAILED', error=f'{type(exc).__name__}: {exc}')
            raise
        finally:
            self._flush()
        return self.result()

    def result(self):
        return {'test': self.name, 'final_state': self.state, 'events': self.events}

    def door_path(self, door_map, cabin_map, normal_map=None):
        """door_axis() in the odometry frame: map door centre / cabin point / door normal ->
        (n, through point, outside point, cabin stand point). The stand point is the cabin point's depth ON the door
        axis: the robot goes straight in and out along the door normal instead of turning next to the door jambs."""
        from .geometry import door_axis
        base = self.io.base
        door = base.map_to_odom((door_map[0], door_map[1], 0.0))
        cabin = base.map_to_odom((cabin_map[0], cabin_map[1], 0.0))
        normal = None
        if normal_map is not None:
            tip = base.map_to_odom((door_map[0]+normal_map[0], door_map[1]+normal_map[1], 0.0))
            normal = (tip[0]-door[0], tip[1]-door[1])
        n, through, outside = door_axis(door[:2], cabin[:2], self.cfg['door_align_m'], normal,
                                        self.cfg['door_through_m'])
        depth = max(self.cfg['door_through_m'], (cabin[0]-door[0])*n[0]+(cabin[1]-door[1])*n[1])
        return n, through, outside, (door[0]+n[0]*depth, door[1]+n[1]*depth)

    def door_from_scan(self, min_range, expect=None):
        """The open door ahead as the LiDAR sees it (the map door can be 0.3 m and 18 deg off near a lift):
        {'door' (odom centre of the opening), 'n' (unit, through the door away from the robot), 'width'} or None
        unless the opening is plausible: width in c['door_width_m'], little missing data inside it, both jambs at
        similar range (a nearer person is not a jamb), and - with `expect` = (odom door centre, unit normal) - near
        the taught door. A glass door is invisible to the LiDAR: such a site needs another door check."""
        scan, pose = self.io.base.scan(), self.io.base.pose()
        gap = None if scan is None or pose is None else scan.opening(min_range)
        c = self.cfg
        lo, hi = c['door_width_m']
        if gap is None or not lo <= gap['width'] <= hi or gap['invalid_fraction'] > .1 \
                or abs(gap['jamb_ranges'][0]-gap['jamb_ranges'][1]) > c['door_jamb_range_diff_m']:
            return None
        (lx, ly), (rx, ry) = gap['left'], gap['right']
        u = ((lx-rx)/gap['width'], (ly-ry)/gap['width'])          # along the door line, right jamb -> left jamb
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = (lx+rx)/2, (ly+ry)/2
        door, n = (x+c*mx-s*my, y+s*mx+c*my), (c*u[1]+s*u[0], s*u[1]-c*u[0])
        if expect is not None:
            (ex, ey), (enx, eny) = expect
            if math.hypot(door[0]-ex, door[1]-ey) > self.cfg['door_match_m'] or \
                    abs(n[0]*enx+n[1]*eny) < math.cos(self.cfg['door_match_rad']):
                return None
        return {'door': door, 'n': n, 'width': gap['width']}

    def door_open(self, min_range, expect, frames=2):
        """True once the door is seen open (door_from_scan) in `frames` consecutive checks."""
        self._door_streak = self._door_streak+1 if self.door_from_scan(min_range, expect) is not None else 0
        return self._door_streak >= frames

    _door_streak = 0

    def go_to_cabin_centre(self, n, through, stand, heading):
        """Straight on along the door axis (already lined up with the opening seen by the LiDAR) to the cabin depth.
        No sideways move inside: a 0.7 m long loaded robot would have to turn next to the walls to shift. The side
        ranges are recorded (the right turn that follows is sweep-checked)."""
        io = self.io
        self.enter('GO_TO_CABIN_CENTRE', target=[round(v, 3) for v in stand])
        self.drive((stand[0], stand[1], heading), max_v=self.cfg['slow_v'])
        self.note(side_ranges_m=[None if r is None else round(r, 3)
                                 for r in (io.base.range_at(math.pi/2), io.base.range_at(-math.pi/2))])

    def wait_for(self, predicate, timeout, period=.5):
        end = self.io.clock.now()+timeout
        while self.io.clock.now() < end:
            if predicate():
                return True
            self.io.clock.sleep(period)
        return bool(predicate())
