"""Synthetic board frames that imitate our buttons: white caps, marker glyphs, dark board."""
import cv2
import numpy as np


def _arc(cx, cy, r, start, end, n=14):
    return [(cx+r*np.cos(np.radians(a)), cy+r*np.sin(np.radians(a)))
            for a in np.linspace(start, end, n)]


# Handwriting-like strokes in a unit box (y down), shaped after the photos.
STROKES = {
    '1': [[(.25, .25), (.58, .03), (.58, .96)], [(.28, .96), (.85, .96)]],
    '2': [_arc(.5, .3, .3, 190, 400) + [(.1, .96), (.92, .96)]],
    '3': [_arc(.5, .26, .3, 200, 450) + _arc(.5, .7, .3, 270, 520)[1:]],
    '4': [[(.68, .96), (.68, .04), (.06, .68), (.96, .68)]],
}
TRIANGLES = {'UP': [(.5, .05), (.02, .92), (.98, .92)],
             'DOWN': [(.02, .08), (.98, .08), (.5, .95)]}


def draw_button(img, center, radius, label, rng, tilt=0.):
    cx, cy = center
    # Cylinder side seen at an angle, then the face, rim and a soft shadow.
    cv2.ellipse(img, (int(cx+radius*.12), int(cy+radius*.15)), (int(radius), int(radius*.95)),
                0, 0, 360, (25, 25, 28), -1)
    cv2.ellipse(img, (int(cx), int(cy+radius*tilt)), (int(radius), int(radius*.96)),
                0, 0, 360, (200, 205, 200), -1)
    cv2.ellipse(img, (int(cx), int(cy)), (int(radius), int(radius*.96)), 0, 0, 360, (236, 238, 234), -1)
    cv2.ellipse(img, (int(cx), int(cy)), (int(radius*.86), int(radius*.83)), 0, 0, 360, (205, 208, 204), 2)
    if label is None:
        return
    gh = radius*rng.uniform(.8, 1.)
    gw = gh*(.95 if label in TRIANGLES else .62)
    rot = np.radians(rng.uniform(-8, 8))
    def place(p):
        x, y = (p[0]-.5)*gw, (p[1]-.5)*gh
        return (cx+x*np.cos(rot)-y*np.sin(rot)+rng.normal(0, radius*.012),
                cy+x*np.sin(rot)+y*np.cos(rot)+rng.normal(0, radius*.012))
    ink = tuple(int(v) for v in rng.integers(40, 90, 3))
    if label in TRIANGLES:
        pts = np.array([place(p) for p in TRIANGLES[label]], np.int32)
        cv2.fillPoly(img, [pts], ink, cv2.LINE_AA)
        return
    thickness = max(2, int(gh*rng.uniform(.10, .15)))
    for stroke in STROKES[label]:
        pts = np.array([place(p) for p in stroke], np.int32)
        cv2.polylines(img, [pts], False, ink, thickness, cv2.LINE_AA)


def scene(labels=('1', '2', '3', '4'), seed=0, size=(640, 480), radius=None):
    """labels fill a 2-column grid in reading order; None leaves a slot blank."""
    rng = np.random.default_rng(seed)
    w, h = size
    img = np.full((h, w, 3), (48, 42, 44), np.uint8)
    rows = (len(labels)+1)//2
    radius = radius or int(min(w/4.6, h/(rows*2.4)) * rng.uniform(.8, 1.))
    centers = []
    for i, label in enumerate(labels):
        col, row = i % 2, i//2
        cx = w*(.27 if col == 0 else .73) + rng.normal(0, 6)
        cy = h*(row+.5)/rows + rng.normal(0, 6)
        draw_button(img, (cx, cy), radius, label, rng, tilt=rng.uniform(0, .08))
        centers.append((cx, cy))
    gradient = np.linspace(rng.uniform(.8, 1.), rng.uniform(1., 1.15), w)[None, :, None]
    img = np.clip(img*gradient + rng.normal(0, 5, img.shape), 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(img, (3, 3), 0), centers
