"""Image processing and projection shared by the simulation and the Jetson tests (OpenCV + NumPy, no ROS).

A camera pose is (position xyz, 3x3 rotation) in the odometry frame with the optical axis along its +x
(image right = -y, image down = -z), focal length fx in pixels (square pixels, principal point at the centre).
"""
import math

import cv2
import numpy as np


def fx_from_hfov(width, hfov):
    return (width/2)/math.tan(hfov/2)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def camera_pose(robot_pose, view):
    """Calibrated fixed view on the robot: view = {xyz (robot frame), yaw_rad, pitch_rad (+down)}."""
    x, y, yaw = robot_pose
    R = rot_z(yaw)
    p = np.array([x, y, 0.0])+R@np.array(view['xyz'], float)
    return p, R@rot_z(view['yaw_rad'])@rot_y(view['pitch_rad'])


def mask(bgr, ranges):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def blobs(bgr, ranges, min_area):
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask(bgr, ranges))
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area >= min_area:
            out.append({'area': int(area), 'u': float(cents[i][0]), 'v': float(cents[i][1]),
                        'bbox': [int(x), int(y), int(w), int(h)], 'label': i})
    out.sort(key=lambda b: -b['area'])
    return out, labels


def ray(u, v, size, fx, pose):
    w, h = size
    cp, cR = pose
    d = np.array([1.0, -(u-w/2)/fx, -(v-h/2)/fx])
    return cp, cR@(d/np.linalg.norm(d))


def on_plane_z(u, v, size, fx, pose, z):
    cp, d = ray(u, v, size, fx, pose)
    if abs(d[2]) < 1e-6:
        return None
    t = (z-cp[2])/d[2]
    return None if t <= 0 else cp+t*d


def on_wall(u, v, size, fx, pose, wall):
    """Intersection with the vertical plane through point q with unit horizontal normal n."""
    cp, d = ray(u, v, size, fx, pose)
    q, n = np.asarray(wall[0], float), np.asarray(wall[1], float)
    den = float(d[:2]@n)
    if abs(den) < 1e-6:
        return None
    t = float((q-cp[:2])@n)/den
    return None if t <= 0 else cp+t*d


def project(xyz, size, fx, pose):
    cp, cR = pose
    v = cR.T@(np.asarray(xyz, float)-cp)
    if v[0] <= .02:
        return None
    return (size[0]/2-fx*v[1]/v[0], size[1]/2-fx*v[2]/v[0])


def parcels(bgr, ranges, min_area, fx, pose, mid_z):
    """Parcel blobs -> centre estimates on the parcel mid-height plane."""
    found, _ = blobs(bgr, ranges, min_area)
    size = (bgr.shape[1], bgr.shape[0])
    out = []
    for b in found:
        xyz = on_plane_z(b['u'], b['v'], size, fx, pose, mid_z)
        if xyz is not None:
            out.append({'xyz': [float(c) for c in xyz], 'blob': b})
    return out


def pallet_face(bgr, ranges, min_area, fx, pose, near_uv=None):
    """Front face of a 2-way pallet from its two edge stringers standing on the floor: in the outer columns of the
    pallet blob, the lowest pixels nearest the camera are the front-bottom corners (an oblique view also shows a
    side face whose bottom edge recedes). Returns face midpoint, unit normal into the pallet and the width."""
    found, labels = blobs(bgr, ranges, min_area)
    if not found:
        return None
    if near_uv is not None:
        found.sort(key=lambda b: math.hypot(b['u']-near_uv[0], b['v']-near_uv[1]))
    b = found[0]
    x, y, w, h = b['bbox']
    sel = labels[y:y+h, x:x+w] == b['label']
    cols = [c for c in range(w) if sel[:, c].any()]
    if len(cols) < 6:
        return None
    size = (bgr.shape[1], bgr.shape[0])
    k = max(3, len(cols)//6)
    cam = pose[0][:2]
    ends = []
    for group in (cols[:k], cols[-k:]):
        pts = []
        for c in group:
            v = y+int(np.flatnonzero(sel[:, c]).max())
            p = on_plane_z(x+c+.5, v+.5, size, fx, pose, 0.0)
            if p is not None:
                pts.append(p[:2])
        if not pts:
            return None
        pts.sort(key=lambda q: float(np.linalg.norm(q-cam)))
        ends.append(np.median(np.array(pts[:3]), axis=0))
    left, right = ends
    mid, along = (left+right)/2, right-left
    width = float(np.linalg.norm(along))
    n = np.array([-along[1], along[0]])/max(width, 1e-9)
    if n@(mid-cam) < 0:
        n = -n
    return {'mid': [float(c) for c in mid], 'normal': [float(c) for c in n], 'width_m': width, 'blob': b}


def seven_segment(bgr, ranges, min_pixels, one_max_aspect):
    """'1' (thin vertical bar) or '2' (wide) from the lit segments; None if too few pixels."""
    m = mask(bgr, ranges)
    ys, xs = np.nonzero(m)
    if len(xs) < min_pixels:
        return None, None
    w, h = xs.max()-xs.min()+1, ys.max()-ys.min()+1
    ratio = float(w)/float(h)
    return ('1' if ratio < one_max_aspect else '2'), {'aspect': round(ratio, 3), 'pixels': int(len(xs))}


def button_caps(bgr, hsv, board_min_px, button_min_px):
    """Cap blobs grouped by their dark board, retaining image coordinates for re-acquisition."""
    board = cv2.morphologyEx(mask(bgr, hsv['board']), cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(board)
    white = mask(bgr, hsv['button']) | mask(bgr, hsv['lamp'])
    value = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
    caps = []
    for i in range(1, n):
        if stats[i][4] < board_min_px:
            continue
        x, y, w, h = [int(v) for v in stats[i][:4]]
        bright = (value[y:y+h, x:x+w] >= .8*float(np.percentile(value[y:y+h, x:x+w], 99))).astype(np.uint8)*255
        roi = cv2.morphologyEx(white[y:y+h, x:x+w] & bright, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        k, _, st, cen = cv2.connectedComponentsWithStats(roi)
        for j in range(1, k):
            cx, cy, cw, ch, area = st[j]
            if area >= button_min_px and cw < w and ch < h:
                caps.append({'u': float(cen[j][0]+x), 'v': float(cen[j][1]+y), 'area': int(area),
                             'bbox': [int(cx+x), int(cy+y), int(cw), int(ch)], 'board_bbox': [x, y, w, h]})
    return caps


def selected_cap(caps, kind, label, near=None, max_distance=80):
    """Select the requested labelled slot on one detected demo board, never a display or another cap."""
    groups = {}
    for cap in caps:
        groups.setdefault(tuple(cap['board_bbox']), []).append(cap)
    selected = []
    for group in groups.values():
        count = 2 if kind == 'hall' else 4
        if len(group) < count:
            continue
        rows = sorted(sorted(group, key=lambda b: -b['area'])[:count], key=lambda b: b['v'])
        if kind == 'hall' and label in ('UP', 'DOWN'):
            cap = rows[0 if label == 'UP' else 1]
        elif kind == 'car' and label in ('1', '2', '3', '4'):
            gap = (rows[2]['v']+rows[3]['v']-rows[0]['v']-rows[1]['v'])/2
            if gap <= 2 or max(rows[1]['v']-rows[0]['v'], rows[3]['v']-rows[2]['v']) > gap*.5:
                continue
            pair = sorted(rows[:2] if label in ('1', '2') else rows[2:], key=lambda b: b['u'])
            if pair[1]['u']-pair[0]['u'] <= 2:
                continue
            cap = pair[0 if label in ('1', '3') else 1]
        else:
            continue
        distance = 0 if near is None else math.hypot(cap['u']-near[0], cap['v']-near[1])
        if distance <= max_distance:
            selected.append((distance, cap))
    return min(selected, key=lambda item: item[0])[1] if selected else None


def button_light(bgr, pixel, hsv_ranges, radius=8, min_fraction=.3):
    """Light evidence from the selected button patch, excluding other buttons and floor digits."""
    if bgr is None or pixel is None or not all(math.isfinite(float(v)) for v in pixel):
        return False
    u, v = [int(round(c)) for c in pixel]
    if not (0 <= u < bgr.shape[1] and 0 <= v < bgr.shape[0]):
        return False
    r = int(radius)
    patch = bgr[max(0,v-r):v+r+1, max(0,u-r):u+r+1]
    return bool(patch.size and np.count_nonzero(mask(patch,hsv_ranges))/patch.shape[0]/patch.shape[1] >= min_fraction)
