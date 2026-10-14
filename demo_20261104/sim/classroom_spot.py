"""Find the taped drop spot (empty pallet place at A) with the arm camera: robot-side data only.

The parcel goes back where it was taken from. AMCL near wall A drifted 0.1-0.2 m between pickup and drop in the
simulation (2026-10-14: 0.11 m placement error), so the stance is taken from the floor tape around the empty spot
instead: yellow tape pixels are projected onto the floor (camera pose from joints + encoder odometry), the near
edge gives the distance, the two side edges give the lateral centre. Heading comes from the LiDAR wall ahead.
Pure numpy/OpenCV + skilltests.vision_core, so the same function can run on the robot's webcam frame.
"""
import math

import cv2
import numpy as np

TAPE_HSV = [[[18, 120, 120], [40, 255, 255]]]          # yellow floor tape (sim colour '1 .85 .1')


def tape_points(bgr, size, fx, pose, ray_fn, step=2):
    """Floor points (odom x, y) of the yellow tape pixels (every `step`-th pixel)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in TAPE_HSV:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    ys, xs = np.nonzero(m[::step, ::step])
    pts = []
    for u, v in zip(xs*step, ys*step):
        p = ray_fn(u+.5, v+.5, size, fx, pose, 0.0)
        if p is not None:
            pts.append((float(p[0]), float(p[1])))
    return np.array(pts) if pts else np.zeros((0, 2))


def find_spot(points, expect_xy, heading, length, width, window=(.45, .55), cell=.015, wall=None, wall_band=.08):
    """Centre of the taped rectangle (`length` along the robot heading, `width` across) from its tape points.
    The points are rasterised in (along, lateral) cells and split into connected outlines; the outline about
    width wide and at least length-0.1 long whose centre is nearest the prior wins (a neighbouring taped spot and the
    area boundary are separate outlines). Near edge (closest to the robot) -> distance, side edges -> lateral centre.
    expect_xy: prior centre (odom). Returns {'centre', 'near_edge_m', 'sides', 'points', 'candidates'} or None."""
    if len(points) == 0:
        return None
    if wall is not None:                                   # (point, unit normal toward the robot) of the wall ahead:
        q, n = np.asarray(wall[0], float), np.asarray(wall[1], float)
        points = points[(points-q)@n > wall_band]          # drop the far edge + boundary tape along the wall base
        if len(points) == 0:
            return None
    f = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([-f[1], f[0]])
    rel = points-np.asarray(expect_xy)
    a, l = rel@f, rel@left                                 # along the heading (away from the robot), lateral (left)
    A, L = length/2+window[0], width/2+window[1]
    keep = (np.abs(a) <= A) & (np.abs(l) <= L)
    a, l = a[keep], l[keep]
    if len(a) < 20:
        return None
    na, nl = int(2*A/cell)+1, int(2*L/cell)+1
    ia, il = ((a+A)/cell).astype(int), ((l+L)/cell).astype(int)
    grid = np.zeros((na, nl), np.uint8)
    grid[ia, il] = 255
    grid = cv2.dilate(grid, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(grid)
    cands = []
    for k in range(1, n):
        sel = lab[ia, il] == k
        if sel.sum() < 15:
            continue
        ak, lk = a[sel], l[sel]
        near = float(np.percentile(ak, 2))
        side = (ak > near+.04) & (ak < near+length-.04)
        if side.sum() < 10:
            continue
        lo, hi = float(np.percentile(lk[side], 3)), float(np.percentile(lk[side], 97))
        along_extent = float(np.percentile(ak, 98))-near
        if abs((hi-lo)-width) > .12 or along_extent < length-(.1 if wall is None else .1+wall_band):
            continue
        cands.append((math.hypot(near+length/2, (lo+hi)/2), near, lo, hi, int(sel.sum())))
    if not cands:
        return None
    _, near, lo, hi, count = min(cands)
    centre = np.asarray(expect_xy)+(near+length/2)*f+((lo+hi)/2)*left
    return {'centre': [float(centre[0]), float(centre[1])], 'near_edge_m': round(near, 3),
            'sides': [round(lo, 3), round(hi, 3)], 'points': count, 'candidates': len(cands)}
