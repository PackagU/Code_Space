"""Find our white round buttons and read 1-4 / UP / DOWN. OpenCV only, no model."""
import time
import cv2
import numpy as np

DIGITS = ('1', '2', '3', '4')
ARROWS = ('UP', 'DOWN')
LABELS = DIGITS + ARROWS
GLYPH_W, GLYPH_H = 48, 64
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def find_buttons(gray):
    """White caps on a dark board: bright blobs that are round and solid."""
    h, w = gray.shape
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, bright = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bright = cv2.morphologyEx(bright, cv2.MORPH_OPEN, KERNEL)  # drop thin glue strings
    contours = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    buttons = []
    for c in contours:
        area = cv2.contourArea(c)
        if not h*w*.002 <= area <= h*w*.25 or len(c) < 5:
            continue
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        perimeter = cv2.arcLength(hull, True)
        if hull_area <= 0 or area/hull_area < .85 or 4*np.pi*hull_area/perimeter**2 < .78:
            continue
        (cx, cy), (a, b), angle = cv2.fitEllipse(hull)
        if min(a, b)/max(a, b) < .55:
            continue
        # A cap sits on a dark board: the ring just outside must be much darker than the face.
        center = (round(cx), round(cy))
        face, ring = np.zeros_like(gray), np.zeros_like(gray)
        cv2.ellipse(face, (center, (a*.8, b*.8), angle), 255, -1)
        cv2.ellipse(ring, (center, (a*1.5, b*1.5), angle), 255, -1)
        cv2.ellipse(ring, (center, (a*1.15, b*1.15), angle), 0, -1)
        if cv2.countNonZero(ring) == 0 or cv2.mean(blur, ring)[0] > .6*cv2.mean(blur, face)[0]:
            continue
        buttons.append({'center': (cx, cy), 'axes': (a/2, b/2), 'angle': angle,
                        'radius': (a+b)/4})
    return buttons


def extract_glyph(gray, button, inner=.72):
    """Dark ink inside the cap face. Returns a tight binary crop or None."""
    (cx, cy), (ax, by), angle = button['center'], button['axes'], button['angle']
    r = max(ax, by)
    x1, y1 = max(0, int(cx-r)), max(0, int(cy-r))
    x2, y2 = min(gray.shape[1], int(cx+r)+1), min(gray.shape[0], int(cy+r)+1)
    roi = gray[y1:y2, x1:x2]
    face = np.zeros_like(roi)
    cv2.ellipse(face, (int(round(cx-x1)), int(round(cy-y1))),
                (max(1, int(ax*inner)), max(1, int(by*inner))), angle, 0, 360, 255, -1)
    values = roi[face > 0]
    if values.size < 50:
        return None
    k = int(values.size*.8)
    white = float(np.partition(values, k)[k])  # 80th percentile, without a full sort
    if white - float(values.min()) < 40:
        return None  # blank cap
    otsu, _ = cv2.threshold(values.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ink = np.where((roi < min(otsu, white*.75)) & (face > 0), 255, 0).astype(np.uint8)
    count, labels, stats, cents = cv2.connectedComponentsWithStats(ink)
    # Drop specks, and rim shadows near the edge; glyph strokes reach toward the center.
    valid = ((stats[:, cv2.CC_STAT_AREA] >= values.size*.004) &
             (np.hypot(cents[:, 0]-(cx-x1), cents[:, 1]-(cy-y1)) <= .6*r))
    valid[0] = False
    # A tilted cap shows its side wall as a dark crescent hugging the face edge ("1" + arc looks
    # like "3"). Glyph ink lives mostly inside; drop components that sit mostly in the outer band.
    band = max(2, int(.14*r))
    core = cv2.erode(face, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*band+1, 2*band+1)))
    inside = np.bincount(labels[(ink > 0) & (core > 0)], minlength=count)
    valid &= inside >= .5*stats[:, cv2.CC_STAT_AREA]
    keep = np.where(valid, 255, 0).astype(np.uint8)[labels]
    points = cv2.findNonZero(keep)
    if points is None:
        return None
    x, y, w, h = cv2.boundingRect(points)
    if h < 8:
        return None
    return keep[y:y+h, x:x+w]


def arrow_direction(ink):
    """Filled triangle -> UP/DOWN by where its mass sits; anything else -> None."""
    solid = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, KERNEL)
    contours = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < .85*sum(cv2.contourArea(v) for v in contours):
        return None
    hull_area = cv2.contourArea(cv2.convexHull(c))
    triangle_area = cv2.minEnclosingTriangle(c.astype(np.float32))[0]
    if hull_area <= 0 or area/hull_area < .88 or area/max(triangle_area, 1) < .75:
        return None
    m = cv2.moments(c)
    x, y, w, h = cv2.boundingRect(c)
    offset = (m['m01']/m['m00'] - (y+h/2)) / h  # +1/6 for apex-up triangle
    if offset > .07:
        return 'UP'
    if offset < -.07:
        return 'DOWN'
    return None


def normalize(ink):
    """Keep aspect ratio (a '1' stays thin) and center on a fixed canvas."""
    h, w = ink.shape
    scale = min((GLYPH_W-6)/w, (GLYPH_H-6)/h)
    small = cv2.resize(ink, (max(1, round(w*scale)), max(1, round(h*scale))),
                       interpolation=cv2.INTER_AREA)
    canvas = np.zeros((GLYPH_H, GLYPH_W), np.uint8)
    oy, ox = (GLYPH_H-small.shape[0])//2, (GLYPH_W-small.shape[1])//2
    canvas[oy:oy+small.shape[0], ox:ox+small.shape[1]] = small
    return np.where(canvas > 100, 255, 0).astype(np.uint8)


def prepare(mask):
    """Mask pixels + distance-to-ink map, computed once per template/query."""
    return (mask > 0).astype(np.float32), cv2.distanceTransform(255-mask, cv2.DIST_L2, 3)


def stack(prepared):
    """{label: [(mask, dist)]} -> (labels, masks[N,H,W], dists[N,H,W], ink counts[N])."""
    labels = [label for label, refs in prepared.items() for _ in refs]
    if not labels:
        return labels, None, None, None
    masks = np.stack([m for refs in prepared.values() for m, _ in refs])
    dists = np.stack([d for refs in prepared.values() for _, d in refs])
    return labels, masks, dists, np.maximum(masks.sum(axis=(1, 2)), 1)


def classify_digit(ink, templates, max_distance=1.5, min_ratio=1.15):
    """Nearest enrolled digit by mean symmetric chamfer distance (canvas px).
    Refuses when too far or when the runner-up digit is nearly as close."""
    labels, masks, dists, counts = templates if isinstance(templates, tuple) else stack(templates)
    if len(set(labels)) < 2:
        return 'UNKNOWN', 99., 0.
    qm, qd = prepare(normalize(ink))
    per_template = ((dists*qm).sum(axis=(1, 2))/max(qm.sum(), 1) +
                    (masks*qd).sum(axis=(1, 2))/counts) / 2
    # Width/height must roughly agree: a thin "1" can never be a wide "3" and vice versa.
    widths = np.maximum((masks.max(axis=1) > 0).sum(axis=1), 1)
    heights = np.maximum((masks.max(axis=2) > 0).sum(axis=1), 1)
    aspect = ink.shape[1]/ink.shape[0]
    per_template[np.abs(np.log(aspect*heights/widths)) > np.log(1.4)] = 99.
    best_by_label = {}
    for label, value in zip(labels, per_template.tolist()):
        best_by_label[label] = min(value, best_by_label.get(label, 99.))
    scores = sorted((value, label) for label, value in best_by_label.items())
    if len(scores) < 2:
        return 'UNKNOWN', 99., 0.
    (best, label), (second, _) = scores[0], scores[1]
    ratio = second / max(best, 1e-3)
    ok = best <= max_distance and ratio >= min_ratio
    return (label if ok else 'UNKNOWN'), best, ratio


def read_button(gray, button, templates):
    """-> (label, distance, ratio, ink). Arrows need no enrollment; digits do."""
    (cx, cy), r = button['center'], button['radius']
    if cx-r < 2 or cy-r < 2 or cx+r > gray.shape[1]-3 or cy+r > gray.shape[0]-3:
        return 'UNKNOWN', 99., 0., None  # cap cut by the frame edge: part of the glyph may be missing
    ink = extract_glyph(gray, button)
    if ink is None:
        return 'UNKNOWN', 99., 0., None
    direction = arrow_direction(ink)
    if direction:
        return direction, 0., 99., ink
    label, dist, ratio = classify_digit(ink, templates)
    return label, dist, ratio, ink


def reading_order(buttons):
    """Rows top-to-bottom, then left-to-right inside each row."""
    rest = sorted(buttons, key=lambda b: b['center'][1])
    ordered = []
    while rest:
        top = rest[0]
        row = [b for b in rest if abs(b['center'][1]-top['center'][1]) < top['radius']]
        ordered += sorted(row, key=lambda b: b['center'][0])
        rest = [b for b in rest if b not in row]
    return ordered


class Gate:
    """Consecutive fresh labels in the same place; losses clear readiness."""
    def __init__(self, required=5, max_gap=.5):
        self.required, self.max_gap = required, max_gap
        self.reset()

    def reset(self):
        self.key, self.count, self.updated = None, 0, 0.

    def update(self, key, now=None):
        now = time.monotonic() if now is None else now
        if key is None:
            self.reset()
            return False
        if key != self.key or now-self.updated > self.max_gap or now <= self.updated:
            self.count = 0
        self.key, self.updated = key, now
        self.count += 1
        return self.count >= self.required
