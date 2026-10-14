"""LiDAR judgements shared by the simulation and the Jetson tests (pure Python + NumPy, no ROS types needed).

A scan is (angle_min, angle_increment, ranges, range_min, range_max) in the LiDAR frame; `lidar` is the LiDAR pose
in the robot frame (x, y, yaw). NaN or out-of-[range_min, range_max] finite values are INVALID; +inf is a valid
"no return within range_max". Every judgement needs enough valid beams in its sector, otherwise it returns None
(unknown), which callers must treat as unsafe / not confirmed.
"""
import math

import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Scan:
    def __init__(self, angle_min, angle_increment, ranges, range_min, range_max, lidar=(0.0, 0.0, 0.0)):
        r = np.asarray(ranges, float)
        self.angles = angle_min+np.arange(r.size)*angle_increment+lidar[2]
        self.increment = abs(float(angle_increment))
        self.range_max = float(range_max)
        no_return = np.isposinf(r)
        valid = (np.isfinite(r) & (r >= range_min) & (r <= range_max)) | no_return
        self.valid = valid
        self.r = np.where(no_return, np.inf, r)
        self.lx, self.ly = float(lidar[0]), float(lidar[1])

    def sector(self, bearing, half):
        d = np.abs(np.arctan2(np.sin(self.angles-bearing), np.cos(self.angles-bearing)))
        return d <= half

    def points(self):
        """Valid finite returns in the robot frame."""
        m = self.valid & np.isfinite(self.r)
        return (self.lx+self.r[m]*np.cos(self.angles[m]), self.ly+self.r[m]*np.sin(self.angles[m]))

    def coverage(self, mask):
        n = int(mask.sum())
        return float((self.valid & mask).sum())/n if n else 0.0

    def range_at(self, bearing, half=math.radians(4), min_coverage=.6):
        """Median range around a bearing (from the LiDAR); inf if no return; None if too few valid beams."""
        mask = self.sector(bearing, half)
        if self.coverage(mask) < min_coverage:
            return None
        vals = self.r[mask & self.valid]
        vals = np.where(np.isinf(vals), self.range_max, vals)
        return float(np.median(vals))

    def max_gap(self, mask):
        """Widest run of invalid beams (rad) inside a sector mask (beams in scan order)."""
        beams = self.valid[mask]
        run = best = 0
        for valid in np.concatenate([beams, beams]):   # doubled: a sector across +-pi wraps around the array
            run = 0 if valid else run+1
            best = max(best, run)
        return min(best, beams.size)*self.increment

    def _corridor(self, front, half_width, min_coverage, max_gap=math.radians(8)):
        """Clearance along +x (front=True) or -x beyond the given extent, inside |y| <= half_width. Unknown (None)
        if the cone is poorly covered or the central travel sector has a blind gap an obstacle could hide in."""
        bearing = 0.0 if front else math.pi
        cone = self.sector(bearing, math.radians(70))
        if self.coverage(cone) < min_coverage:
            return None
        if self.max_gap(self.sector(bearing, math.radians(45))) > max_gap:
            return None
        x, y = self.points()
        sel = (np.abs(y) <= half_width) & ((x > 0) if front else (x < 0))
        if not sel.any():
            return math.inf
        return float(np.min(np.abs(x[sel])))

    def front_clearance(self, front_extent, half_width, min_coverage=.6):
        d = self._corridor(True, half_width, min_coverage)
        return None if d is None else d-front_extent

    def rear_clearance(self, rear_extent, half_width, min_coverage=.6):
        d = self._corridor(False, half_width, min_coverage)
        return None if d is None else d-rear_extent

    def rotation_clearance(self, front, rear, half_width, direction, lookahead=math.radians(10), steps=5,
                           min_coverage=.6):
        """Smallest gap between LiDAR returns and the robot rectangle (incl. a carried load: x in [-rear, front],
        |y| <= half_width) over the next `lookahead` of an in-place rotation in `direction` (+1 left, -1 right).
        Only the area the body actually sweeps counts (a full circle would forbid turning near side walls)."""
        if self.coverage(np.ones_like(self.valid)) < min_coverage:
            return None
        x, y = self.points()
        if x.size == 0:
            return math.inf
        best = math.inf
        for k in range(1, steps+1):                     # the poses the turn moves through (not where it is now)
            a = -direction*lookahead*k/steps        # points seen from the body after it turned by direction*angle
            c, s = math.cos(a), math.sin(a)
            px, py = c*x-s*y, s*x+c*y
            dx = np.maximum(np.maximum(-rear-px, px-front), 0.0)
            dy = np.maximum(np.abs(py)-half_width, 0.0)
            best = min(best, float(np.min(np.hypot(dx, dy))))
        return best

    @staticmethod
    def _rect_gap(px, py, front, rear, half_width):
        dx = np.maximum(np.maximum(-rear-px, px-front), 0.0)
        dy = np.maximum(np.abs(py)-half_width, 0.0)
        inside = (dx == 0) & (dy == 0)
        return np.where(inside, -1.0, np.hypot(dx, dy))   # negative = inside the body

    def sweep_violation(self, v, w, front, rear, half_width, margin, horizon, steps=6, min_coverage=.6,
                        max_blind=math.radians(15)):
        """Per-return check of the body (incl. load) swept along the commanded arc (v, w) for `horizon` s:
        None if clear, 'SCAN_INVALID' for poor coverage or a blind gap wide enough to hide an obstacle, else
        'SWEEP_STOP' when a return would end inside the body, or inside `margin` while getting closer than it is
        now (a robot already next to a wall may still move away from it)."""
        everything = np.ones_like(self.valid)
        if self.coverage(everything) < min_coverage or self.max_gap(everything) > max_blind:
            return 'SCAN_INVALID'
        x, y = self.points()
        if x.size == 0:
            return None
        now = self._rect_gap(x, y, front, rear, half_width)
        worst = np.full(x.shape, np.inf)
        for k in range(1, steps+1):
            t = horizon*k/steps
            th = w*t
            if abs(w) > 1e-6:
                ox, oy = v/w*math.sin(th), v/w*(1-math.cos(th))
            else:
                ox, oy = v*t, 0.0
            c, s = math.cos(-th), math.sin(-th)
            px, py = c*(x-ox)-s*(y-oy), s*(x-ox)+c*(y-oy)
            worst = np.minimum(worst, self._rect_gap(px, py, front, rear, half_width))
        if np.any(worst < 0) or np.any((worst < margin) & (worst < now-1e-4)):
            return 'SWEEP_STOP'
        return None

    def opening(self, min_range, half=math.radians(60)):
        """The open gap ahead (an open door): from the open beam nearest bearing 0 (within half/2), beams stay open
        (valid and at least min_range) until the first valid closer return on each side = the jambs. -> {'left', 'right' (robot-frame
        points), 'centre_y', 'width'} or None (ahead not open, or a side has no jamb within `half`)."""
        idx = np.where(self.sector(0.0, half))[0]
        if idx.size == 0:
            return None
        rel = np.arctan2(np.sin(self.angles[idx]), np.cos(self.angles[idx]))   # wrapped: any angle_min/yaw
        idx, rel = idx[np.argsort(rel)], np.sort(rel)
        off = np.abs(rel)

        def is_open(i):
            return bool(self.valid[i]) and self.r[i] >= min_range
        opened = [k for k in range(idx.size) if is_open(idx[k]) and off[k] <= half/2]
        if not opened:                                    # nothing open near the heading (door closed)
            return None
        mid = min(opened, key=lambda k: off[k])           # the open beam nearest the heading (robot may be skewed)
        edges, edge_k = [], []
        for step in (1, -1):                              # left (increasing angle), then right
            k, edge = mid, None
            while 0 <= k+step < idx.size:
                k += step
                i = idx[k]
                if self.valid[i] and self.r[i] < min_range:
                    edge = (self.lx+self.r[i]*math.cos(self.angles[i]), self.ly+self.r[i]*math.sin(self.angles[i]))
                    break
            if edge is None:
                return None
            edges.append(edge)
            edge_k.append(k)
        (lx, ly), (rx, ry) = edges
        gap = idx[min(edge_k):max(edge_k)+1]
        return {'left': (lx, ly), 'right': (rx, ry), 'centre_y': (ly+ry)/2, 'width': math.hypot(lx-rx, ly-ry),
                'invalid_fraction': float(1-self.valid[gap].mean()) if gap.size else 1.0,
                'jamb_ranges': (math.hypot(lx-self.lx, ly-self.ly), math.hypot(rx-self.lx, ry-self.ly))}

    def wall(self, bearing, pose, half=math.radians(25), max_range=2.5, min_points=8, inlier_m=.03, min_span=.25,
             ray_tol=math.radians(3)):
        """Line fit (odom frame) to the wall the bearing ray hits -> (point, unit normal toward the robot, info), or
        None. Robust to corners (a plain fit over a sector that holds two walls gives a diagonal 'wall'): every
        candidate line passes through the return nearest the ray, the one with most returns within inlier_m wins,
        then a least-squares refit on its inliers (twice). None if the ray has no return, or the inliers are too few
        or span less than min_span along the wall (the normal would be unreliable)."""
        x, y = self.points()
        b = np.arctan2(y, x)
        off = np.abs(np.arctan2(np.sin(b-bearing), np.cos(b-bearing)))
        sel = (np.hypot(x, y) < max_range) & (off <= half)
        if int(sel.sum()) < min_points:
            return None
        pts = np.stack([x[sel], y[sel]], axis=1)
        k = int(np.argmin(off[sel]))
        if off[sel][k] > ray_tol:
            return None
        anchor = pts[k]
        candidates = []
        for other in pts:
            d = other-anchor
            length = math.hypot(d[0], d[1])
            if length < .1:
                continue
            normal = np.array([-d[1], d[0]])/length
            candidates.append((int((np.abs((pts-anchor)@normal) <= inlier_m).sum()), normal))
        if not candidates:
            return None
        best = max(candidates, key=lambda c: c[0])
        # a corner right at the ray: another wall direction with comparable support -> ambiguous, refuse
        rival = max((c[0] for c in candidates if abs(float(c[1]@best[1])) < math.cos(math.radians(30))), default=0)
        if rival >= .5*best[0]:
            return None
        n, centre = best[1], anchor
        for _ in range(2):
            inl = pts[np.abs((pts-centre)@n) <= inlier_m]
            if len(inl) < min_points:
                return None
            centre = inl.mean(axis=0)
            n = np.linalg.svd(inl-centre)[2][1]
        if abs(float((anchor-centre)@n)) > inlier_m:   # the refit drifted off the wall the ray hits
            return None
        along = (inl-centre)@np.array([-n[1], n[0]])
        span = float(along.max()-along.min())
        if span < min_span:
            return None
        residual = float(np.abs((inl-centre)@n).max())
        c, s = math.cos(pose[2]), math.sin(pose[2])
        q = np.array([pose[0]+c*centre[0]-s*centre[1], pose[1]+s*centre[0]+c*centre[1]])
        n = np.array([c*n[0]-s*n[1], s*n[0]+c*n[1]])
        if n@(np.array(pose[:2])-q) < 0:
            n = -n
        return q, n, {'points': int(len(inl)), 'of': int(sel.sum()), 'span_m': round(span, 3),
                      'max_residual_m': round(residual, 4)}


def motion_check(scan, v, w, geom, margins):
    """None if the commanded motion is clear, else a stop reason. geom: front_extent, rear_extent, half_width
    (incl. a carried load). margins: front, rear, sweep (m). Unknown (insufficient valid data) blocks motion."""
    if scan is None:
        return 'NO_SCAN'
    if v > 1e-3:
        c = scan.front_clearance(geom['front_extent'], geom['half_width'])
        if c is None:
            return 'SCAN_INVALID'
        if c <= margins['front']:
            return 'FRONT_STOP'
    if v < -1e-3:
        c = scan.rear_clearance(geom['rear_extent'], geom['half_width'])
        if c is None:
            return 'SCAN_INVALID'
        if c <= margins['rear']:
            return 'REAR_STOP'
    if abs(w) > 1e-3:
        # any turning command, also while driving (curves): the body swept along the commanded arc. Horizon: about
        # 0.6 s, at most 10 deg and at least 0.5 deg of rotation.
        horizon = min(.6, math.radians(10)/abs(w)) if abs(v) > 1e-3 else \
            min(math.radians(10), max(math.radians(.5), abs(w)*.6))/abs(w)
        reason = scan.sweep_violation(v, w, geom['front_extent'], geom['rear_extent'], geom['half_width'],
                                      margins['sweep'], horizon, max_blind=margins.get('blind', math.radians(15)))
        if reason:
            return reason
    return None


def fresh(stamp, now, max_age, skew=.25):
    """Age within max_age. A slightly negative age is clock skew, not staleness (the simulation /clock is published
    at 10 Hz so sensor stamps can be newer than 'now'; machines on a LAN differ by small offsets)."""
    return stamp is not None and math.isfinite(stamp) and -skew <= now-stamp <= max_age
