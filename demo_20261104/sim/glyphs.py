"""Digit glyphs for the classroom demo: one geometry definition draws the world (label and display digits) and renders
the reading templates, like registering templates from the printed font (tools/floor_reader, button_arm_test).

Two fonts:
  label   - bold block digits on the white shipping label (strokes overlap at the corners: one blob per digit)
  display - the 7-segment floor display (sim/real_maps/dressing.py SEGMENTS, segments do not touch)
Reading uses the arm-camera image only: find the white label on cardboard -> rectify its 4 corners to a front-on
rectangle (perspective warp) -> split the dark glyphs -> decide each
glyph from how much of each of its 7 segment regions is filled (cropped, resized box); the font templates' overlap is logged.
The display reading takes the red lit segments.
Pure numpy/OpenCV: the same functions run on the simulated camera and on a webcam frame.
"""
import cv2
import numpy as np

# standard 7-segment names: a top, b upper right, c lower right, d bottom, e lower left, f upper left, g middle
DIGIT_SEGMENTS = {'0': 'abcdef', '1': 'bc', '2': 'abged', '3': 'abgcd', '4': 'fgbc', '5': 'afgcd',
                  '6': 'afgedc', '7': 'abc', '8': 'abcdefg', '9': 'abcfgd'}
# dressing.SEGMENTS: (u right, v up, horizontal, vertical) in digit-height units; sizes as in dressing.digit_model
DISPLAY_SEGMENTS = {'a': (0, .5, 1, 0), 'b': (.28, .25, 0, 1), 'c': (.28, -.25, 0, 1), 'd': (0, -.5, 1, 0),
                    'e': (-.28, -.25, 0, 1), 'f': (-.28, .25, 0, 1), 'g': (0, 0, 1, 0)}
TEMPLATE_SIZE = (24, 36)          # (w, h) after cropping to the glyph box


def label_strokes(digit, w, h, t):
    """Bold block digit: [(u_centre, v_centre, du, dv)] in metres (u right, v up), strokes overlap at the corners."""
    seg = {'a': (0, h/2-t/2, w, t), 'd': (0, -h/2+t/2, w, t), 'g': (0, 0, w, t),
           'f': (-w/2+t/2, h/4, t, h/2), 'b': (w/2-t/2, h/4, t, h/2),
           'e': (-w/2+t/2, -h/4, t, h/2), 'c': (w/2-t/2, -h/4, t, h/2)}
    return [seg[s] for s in DIGIT_SEGMENTS[digit]]


def display_strokes(digit, h=1.0):
    out = []
    for s in DIGIT_SEGMENTS[digit]:
        u, v, horizontal, _ = DISPLAY_SEGMENTS[s]
        du, dv = (h*.5, h*.11) if horizontal else (h*.11, h*.45)
        out.append((u*h, v*h, du, dv))
    return out


def _render(strokes, scale=400):
    span = max(max(abs(u)+du/2, abs(v)+dv/2) for u, v, du, dv in strokes)*2.2
    n = int(span*scale)+4
    img = np.zeros((n, n), np.uint8)
    for u, v, du, dv in strokes:
        x0, x1 = n/2+(u-du/2)*scale, n/2+(u+du/2)*scale
        y0, y1 = n/2-(v+dv/2)*scale, n/2-(v-dv/2)*scale
        cv2.rectangle(img, (int(round(x0)), int(round(y0))), (int(round(x1))-1, int(round(y1))-1), 255, -1)
    return img


def normalise(mask):
    """Crop a binary glyph to its pixels and resize to TEMPLATE_SIZE (float 0..1); None if empty."""
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    crop = mask[ys.min():ys.max()+1, xs.min():xs.max()+1].astype(np.float32)/255.0
    return cv2.resize(crop, TEMPLATE_SIZE, interpolation=cv2.INTER_AREA)


def templates(font, label_cfg=None):
    out = {}
    for d in DIGIT_SEGMENTS:
        if font == 'label':
            w, h = label_cfg['glyph_m']
            strokes = label_strokes(d, w, h, float(label_cfg['stroke_m']))
        else:
            strokes = display_strokes(d)
        out[d] = normalise(_render(strokes))
    return out


# segment regions in the normalised glyph box (x0, x1, y0, y1; x left->right, y top->bottom): the middle part of each
# stroke, away from the corners where strokes meet (a mean over the region tolerates shear and blur better than a
# point); valid on/off patterns -> digit
REGION = {'a': (.28, .72, .0, .13), 'd': (.28, .72, .87, 1.0), 'g': (.28, .72, .44, .56),
          'f': (.0, .16, .14, .38), 'b': (.84, 1.0, .14, .38), 'e': (.0, .16, .62, .86), 'c': (.84, 1.0, .62, .86)}
PATTERN = {''.join(sorted(v)): d for d, v in DIGIT_SEGMENTS.items()}
ON_FILL = 0.3


def classify(mask, tmpl=None, aspect=None):
    """(digit, confidence, fills, dice) for one glyph. Decision: which of the 7 segment places are filled in the
    normalised glyph box (a thin bar, w/h < 0.35, is a '1' in both fonts). confidence = how clearly every segment
    place is on or off (0..1, worst segment). dice = overlap with the font templates (logged, not decisive)."""
    g = normalise(mask)
    if g is None:
        return None, 0.0, {}, {}
    dice = {}
    if tmpl:
        for d, t in tmpl.items():
            dice[d] = round(2*float(np.minimum(g, t).sum())/max(float(g.sum()+t.sum()), 1e-6), 3)
        dice = dict(sorted(dice.items(), key=lambda kv: -kv[1])[:3])
    if aspect is not None and aspect < .35:
        fill = float(g.mean())
        return '1', round(max(0.0, min(1.0, (fill-.5)*2)), 3), {'bar': round(fill, 3)}, dice
    w, h = TEMPLATE_SIZE
    fills = {}
    for seg, (x0, x1, y0, y1) in REGION.items():
        fills[seg] = round(float(g[int(round(y0*h)):max(int(round(y1*h)), int(round(y0*h))+1),
                                   int(round(x0*w)):max(int(round(x1*w)), int(round(x0*w))+1)].mean()), 3)
    # small, blurred glyphs render their strokes thinner (an 'on' region fills 0.5-0.7) while 'off' regions stay
    # near 0: split at ON_FILL; confidence = the worst segment's distance from the split, scaled to 0..1 each side
    on = ''.join(sorted(s for s, f in fills.items() if f >= ON_FILL))
    conf = min(((f-ON_FILL)/(1-ON_FILL) if f >= ON_FILL else (ON_FILL-f)/ON_FILL) for f in fills.values())
    return PATTERN.get(on), round(conf, 3), fills, dice


def hsv_mask(bgr, ranges):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return m


LABEL_WHITE = [[[0, 0, 165], [180, 60, 255]]]
GLYPH_DARK_V = 95


def _order(pts):
    """4 corner points -> (top-left, top-right, bottom-right, bottom-left)."""
    pts = np.asarray(pts, np.float32).reshape(4, 2)
    s, d = pts.sum(1), np.diff(pts, axis=1).ravel()
    return np.float32([pts[np.argmin(s)], pts[np.argmin(d)], pts[np.argmax(s)], pts[np.argmax(d)]])


def read_label(bgr, cardboard_hsv, tmpl, near_uv=None, min_area=60, aspect=28/18, warp_h=180):
    """Shipping label number. The white label's 4 corners are rectified to a front-on rectangle (perspective warp,
    as tools/floor_reader does with its ROI corners) so the glyphs are upright before they are split and read.
    Returns {'text', 'glyphs': [...], 'label_bbox', 'why'} (text None when unreadable)."""
    white = cv2.morphologyEx(hsv_mask(bgr, LABEL_WHITE), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    board = hsv_mask(bgr, cardboard_hsv)
    n, lab, stats, cen = cv2.connectedComponentsWithStats(white)
    H, W = white.shape
    cands = []
    ring_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area or w < 12 or h < 8 or w > .6*W or h > .6*H:   # floor/wall regions are not labels
            continue
        x0, y0, x1, y1 = max(0, x-4), max(0, y-4), min(W, x+w+4), min(H, y+h+4)
        blob = (lab[y0:y1, x0:x1] == i).astype(np.uint8)
        ring = cv2.dilate(blob, ring_kernel) & (1-blob)          # 3 px just outside the white blob
        if not ring.any() or (board[y0:y1, x0:x1][ring > 0] > 0).mean() < .15:  # a label sits on cardboard
            continue
        d = 0.0 if near_uv is None else float(np.hypot(cen[i][0]-near_uv[0], cen[i][1]-near_uv[1]))
        cands.append((d, -area, i))
    if not cands:
        return {'text': None, 'why': 'no white label on cardboard'}
    _, _, i = min(cands)
    x, y, w, h, _ = stats[i]
    contours, _ = cv2.findContours((lab == i).astype(np.uint8)*255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(contours, key=cv2.contourArea)
    approx = cv2.approxPolyDP(c, .03*cv2.arcLength(c, True), True)
    quad = _order(approx if len(approx) == 4 else cv2.boxPoints(cv2.minAreaRect(c)))
    ww = int(round(warp_h*aspect))
    dst = np.float32([[0, 0], [ww-1, 0], [ww-1, warp_h-1], [0, warp_h-1]])
    M = cv2.getPerspectiveTransform(quad, dst)
    warped = cv2.warpPerspective(bgr, M, (ww, warp_h), flags=cv2.INTER_LINEAR)
    v = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)[:, :, 2]
    dark = (v < GLYPH_DARK_V).astype(np.uint8)*255
    m = int(.04*warp_h)
    dark[:m, :], dark[-m:, :], dark[:, :m], dark[:, -m:] = 0, 0, 0, 0       # label edge pixels are not glyphs
    k, gl, gs, gc = cv2.connectedComponentsWithStats(dark, connectivity=8)
    Minv = np.linalg.inv(M)
    glyphs = []
    for j in range(1, k):
        gx, gy, gw, gh, ga = gs[j]
        if gh < .4*warp_h or ga < 30:
            continue
        digit, conf, fills, dice = classify((gl == j).astype(np.uint8)*255, tmpl, aspect=gw/max(gh, 1))
        corners = cv2.perspectiveTransform(np.float32([[[gx, gy]], [[gx+gw, gy]], [[gx+gw, gy+gh]], [[gx, gy+gh]]]), Minv)
        bx0, by0 = corners[:, 0, 0].min(), corners[:, 0, 1].min()
        bx1, by1 = corners[:, 0, 0].max(), corners[:, 0, 1].max()
        glyphs.append({'u': float(gc[j][0]), 'bbox': [int(bx0), int(by0), int(bx1-bx0), int(by1-by0)], 'digit': digit,
                       'confidence': conf, 'segments': fills, 'dice': dice})
    glyphs.sort(key=lambda g: g['u'])
    out = {'label_bbox': [int(x), int(y), int(w), int(h)], 'label_quad': quad.round(1).tolist(), 'glyphs': glyphs}
    if len(glyphs) != 3:
        return out | {'text': None, 'why': f'{len(glyphs)} glyphs (need 3)'}
    if any(g['digit'] is None for g in glyphs):
        return out | {'text': None, 'why': 'a glyph is not a digit pattern'}
    return out | {'text': ''.join(g['digit'] for g in glyphs), 'why': None}


def read_display(bgr, digit_hsv, tmpl, min_px=15):
    """Lit 7-segment digit -> {'digit', 'score', 'margin', 'bbox'}; digit None if too few lit pixels."""
    m = hsv_mask(bgr, digit_hsv)
    ys, xs = np.nonzero(m)
    if len(xs) < min_px:
        return {'digit': None, 'why': 'no lit digit', 'pixels': int(len(xs))}
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    digit, conf, fills, dice = classify(m[y0:y1+1, x0:x1+1], tmpl, aspect=(x1-x0+1)/(y1-y0+1))
    return {'digit': digit, 'confidence': conf, 'segments': fills, 'dice': dice, 'pixels': int(len(xs)),
            'bbox': [int(x0), int(y0), int(x1-x0+1), int(y1-y0+1)]}


def accept(glyph, cfg):
    return glyph.get('digit') is not None and glyph['confidence'] >= float(cfg['min_segment_confidence'])
