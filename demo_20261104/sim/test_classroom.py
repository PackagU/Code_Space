#!/usr/bin/env python3
"""Offline tests for the classroom demo (no ROS, no Gazebo):
  - label / display digit reading on synthetic oblique, blurred, noisy camera-like images
  - floor rule (first digit 2..4) and the low-row press cycle (IK) reaching 8 cm lower at the same reach
  python3 demo_20261104/sim/test_classroom.py
"""
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT/'sim/real_maps'))
import glyphs  # noqa: E402

CFG = yaml.safe_load((HERE/'classroom.yaml').read_text())
WORLD = yaml.safe_load((ROOT/'sim/real_maps/world_params.yaml').read_text())
CARDBOARD = WORLD['perception']['hsv']['parcel']
FAILS = []


def check(name, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ')+name+(f'  {detail}' if detail else ''))
    if not ok:
        FAILS.append(name)


def label_image(text, tilt_deg=35, yaw_deg=8, px_per_m=260, seed=0):
    """Cardboard face with the label, seen from above at `tilt_deg` and turned by `yaw_deg` (perspective warp)."""
    lab = CFG['parcels']['label']
    s = 1200                                          # px per metre on the flat face
    face = np.zeros((int(.20*s), int(.30*s), 3), np.uint8)
    face[:] = (91, 139, 184)                          # cardboard BGR
    lw, lh = lab['size_m']
    cx, cy = face.shape[1]//2, face.shape[0]//2
    cv2.rectangle(face, (int(cx-lw*s/2), int(cy-lh*s/2)), (int(cx+lw*s/2), int(cy+lh*s/2)), (242, 244, 244), -1)
    gw, gh = lab['glyph_m']
    for k, ch in enumerate(text):
        u0 = (k-1)*(gw+lab['gap_m'])
        for u, v, du, dv in glyphs.label_strokes(ch, gw, gh, lab['stroke_m']):
            x0, x1 = cx+(u0+u-du/2)*s, cx+(u0+u+du/2)*s
            y0, y1 = cy-(v+dv/2)*s, cy-(v-dv/2)*s
            cv2.rectangle(face, (int(x0), int(y0)), (int(x1), int(y1)), (15, 13, 13), -1)
    h, w = face.shape[:2]
    sx = px_per_m/s
    squash = math.cos(math.radians(tilt_deg))
    skew = math.tan(math.radians(yaw_deg))*.15
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst = np.float32([[0, 0], [w*sx, h*sx*skew], [w*sx*.95, h*sx*squash+h*sx*skew], [w*sx*.05, h*sx*squash]])
    warped = cv2.warpPerspective(face, cv2.getPerspectiveTransform(src, dst), (320, 240), borderValue=(60, 70, 75))
    out = np.full((240, 320, 3), (60, 70, 75), np.uint8)
    out[30:, 60:] = warped[:210, :260]
    out = cv2.GaussianBlur(out, (3, 3), .8)
    rng = np.random.default_rng(seed)
    return np.clip(out.astype(np.int16)+rng.normal(0, 4, out.shape).astype(np.int16), 0, 255).astype(np.uint8)


def display_image(digit, px=34):
    img = np.full((240, 320, 3), (20, 20, 22), np.uint8)
    cx, cy = 160, 120
    for u, v, du, dv in glyphs.display_strokes(digit, 1.0):
        x0, x1 = cx+(u-du/2)*px*2, cx+(u+du/2)*px*2
        y0, y1 = cy-(v+dv/2)*px*2, cy-(v-dv/2)*px*2
        cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), (25, 38, 255), -1)
    return cv2.GaussianBlur(img, (3, 3), .7)


def main():
    lab_t = glyphs.templates('label', CFG['parcels']['label'])
    disp_t = glyphs.templates('display')
    for text in ('402', '310', '205', '301', '418', '399'):
        for tilt, yaw, scale in ((35, 8, 260), (50, -10, 220), (20, 3, 300)):
            got = glyphs.read_label(label_image(text, tilt, yaw, scale, seed=len(text)+tilt), CARDBOARD, lab_t)
            ok = got['text'] == text and all(glyphs.accept(g, CFG['reading']) for g in got['glyphs'])
            check(f'label {text} tilt {tilt} yaw {yaw}', ok,
                  f"read={got['text']} conf={[g['confidence'] for g in got.get('glyphs', [])]} why={got.get('why')}")
    for d in '1234':
        got = glyphs.read_display(display_image(d), WORLD['perception']['hsv']['digit'], disp_t)
        check(f'display {d}', got['digit'] == d and glyphs.accept(got, CFG['reading']),
              f"read={got['digit']} conf={got.get('confidence')} dice={got.get('dice')}")
    blank = glyphs.read_label(np.full((240, 320, 3), (60, 70, 75), np.uint8), CARDBOARD, lab_t)
    check('no label -> None', blank['text'] is None, blank.get('why', ''))
    # floor rule
    valid = CFG['reading']['valid_first_digits']
    for text, floor in (('402', 4), ('310', 3), ('205', 2), ('150', None), ('42', None)):
        got = int(text[0]) if len(text) == 3 and text[0] in valid else None
        check(f'floor rule {text}', got == floor, f'-> {got}')
    # low-row press cycle
    import arm_geometry as ag
    sys.path.insert(0, str(HERE))
    from classroom_arm import add_low_cycles
    sp = ag.servo_protocol(ROOT)
    arm = WORLD['arm']
    added = add_low_cycles(sp, arm, CFG['boards']['low_row_dz_m'], WORLD['arm']['press_cycle'])
    for key, number in added.items():
        top = ag.taught_press(arm, sp, int(WORLD['arm']['press_cycle'][key]))
        low = ag.taught_press(arm, sp, number)
        dz = top['z']-low['z']
        reach = math.dist(top['xy'], low['xy'])
        check(f'low cycle {key} -> {number}', abs(dz-float(CFG['boards']['low_row_dz_m'][key])) < .004 and reach < .005
              and abs(low['yaw']) <= .01, f'dz={dz:.4f} reach_diff={reach:.4f} pwm={low["pwm"]}')
        # the way in and out is straight along the press direction at the button height (no sweep over the row above)
        ang = {s['name']: ag.pwm_angle(arm, s['servo'], sp.POSE_TABLES[number]['low_standoff'][s['servo']])
               for s in arm['joints']}
        stand = ag.fk(arm, ang)[0]
        back = low['xy'][0]-float(stand[0])
        check(f'low cycle {key} standoff', abs(float(stand[2])-low['z']) < .004 and abs(back-.08) < .004,
              f'standoff z={stand[2]:.4f} back={back:.4f}')
    # taped drop spot: P1 rectangle with P2's rectangle 0.2 m beside it and the area boundary, prior off by 0.15 m
    from classroom_spot import find_spot
    rng = np.random.default_rng(3)

    def outline(cx, cy, L, W, n=60):
        pts = []
        for t in np.linspace(-.5, .5, n):
            pts += [(cx-L/2, cy+t*W), (cx+L/2, cy+t*W), (cx+t*L, cy-W/2), (cx+t*L, cy+W/2)]
        return pts
    tape = outline(.25, 1.35, .38, .50)+outline(.25, .65, .38, .50)+[(0.0, y) for y in np.linspace(-.2, 2.2, 80)]
    tape += [(x, 2.0) for x in np.linspace(0, 1.5, 60)]
    pts = np.array(tape)+rng.normal(0, .006, (len(tape), 2))
    for prior in ((.25, 1.35), (.40, 1.47), (.12, 1.22)):
        got = find_spot(pts, prior, math.pi, .38, .50, wall=((0.0, 1.35), (1.0, 0.0)))
        err = None if got is None else math.dist(got['centre'], (.25, 1.35))
        check(f'tape spot prior {prior}', err is not None and err < .02, f'err={err} got={got and got["centre"]}')
    print('RESULT', 'FAIL '+', '.join(FAILS) if FAILS else 'ALL PASS')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
