#!/usr/bin/env python3
"""Generate the classroom demo world, its map and the scene description (demo_20261104/sim/generated/).

  classroom.world        Gazebo world: classroom walls + desks, A/B partitions, floor tape, B-wall call board (UP DOWN)
                         and floor board (1 2 / 3 4), floor display (stand + panel + digit models parked below the floor)
  classroom_map.yaml/pgm Nav2 map rasterised from the same collision boxes (no SLAM needed in simulation)
  classroom_boxes.json   collision boxes [x, y, sx, sy, yaw] for the ground-truth clearance metric
  classroom_scene.json   cap faces, board heights, display position, digit models, parcel labels, view poses, press tips

Board heights follow the taught press cycles (forward kinematics of the assumed arm, as sim/real_maps does): the top
row at the taught tip, the bottom row lower by the board's row spacing (pressed with the simulated low-row cycle).
The display centre is where the display_view camera axis meets the display plane. All values are simulation
assumptions ([가정값], classroom.yaml / world_params.yaml).
"""
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
REAL = ROOT/'sim/real_maps'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REAL))
sys.path.insert(0, str(ROOT/'tools/jetson_skill_tests'))
import arm_geometry as ag                                  # noqa: E402
import dressing as dr                                      # noqa: E402
from generate_worlds import write_world                    # noqa: E402
from skilltests import vision_core as vc                   # noqa: E402
from classroom_arm import add_low_cycles                   # noqa: E402
import glyphs                                              # noqa: E402

OUT = HERE/'generated'
CFG = yaml.safe_load((HERE/'classroom.yaml').read_text())
WORLD = yaml.safe_load((REAL/'world_params.yaml').read_text())
RES = 0.05
PARTITION = '.80 .83 .87 1'
DESK = '.55 .41 .29 1'
STAND = '.25 .26 .28 1'
TAPE = '1 .85 .1 1'                 # yellow (H ~ 26): outside the cardboard colour class


def solid_model(name, boxes, rgba):
    """Static model, one collision + visual per box (x, y, z_centre, sx, sy, sz, yaw)."""
    model, link = dr.static_model(name)
    link.set('name', 'solid')
    for i, (x, y, z, sx, sy, sz, yaw) in enumerate(boxes):
        for kind in ('collision', 'visual'):
            item = ET.SubElement(link, kind, name=f'{kind}_{i}')
            ET.SubElement(item, 'pose').text = f'{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}'
            ET.SubElement(ET.SubElement(ET.SubElement(item, 'geometry'), 'box'), 'size').text = f'{sx:.4f} {sy:.4f} {sz:.4f}'
            if kind == 'visual':
                dr.material(item, rgba)
    return model


def tape_model(area, poses, parcels, pallet):
    model, link = dr.static_model('floor_tape')
    w = .03
    ax, ay = area
    strips = [((ax/2, 0.0), (ax, w)), ((ax/2, ay), (ax, w)), ((0.0, ay/2), (w, ay)), ((ax, ay/2), (w, ay))]
    for name in ('start', 'a_approach', 'b_call_view', 'b_floor_view'):
        x, y = poses[name][:2]
        strips += [((x, y), (.20, w)), ((x, y), (w, .20))]
    margin = .05
    L, Wd = pallet['size_m'][0]+2*margin, pallet['size_m'][1]+2*margin
    for item in parcels:
        x, y = item['xy']
        strips += [((x, y-Wd/2), (L, w)), ((x, y+Wd/2), (L, w)), ((x-L/2, y), (w, Wd)), ((x+L/2, y), (w, Wd))]
    for i, ((x, y), (sx, sy)) in enumerate(strips):
        dr.add_visual(link, f'tape_{i}', (x, y, .0006), (0, 0, 0), 'box', (sx, sy, .001), TAPE)
    return model


def rasterise(boxes, x_range, y_range):
    x0, y0 = x_range[0]-.2, y_range[0]-.2
    w, h = int(math.ceil((x_range[1]-x0+.2)/RES)), int(math.ceil((y_range[1]-y0+.2)/RES))
    grid = np.full((h, w), 254, np.uint8)                   # free
    xs = x0+(np.arange(w)+.5)*RES
    ys = y0+(np.arange(h)+.5)*RES
    gx, gy = np.meshgrid(xs, ys)
    for x, y, sx, sy, yaw in boxes:
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx, ly = c*(gx-x)-s*(gy-y), s*(gx-x)+c*(gy-y)
        grid[(np.abs(lx) <= sx/2+RES/2) & (np.abs(ly) <= sy/2+RES/2)] = 0
    return np.flipud(grid), [x0, y0, 0.0]


def write_pgm(path, grid):
    with open(path, 'wb') as handle:
        handle.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())
        handle.write(grid.tobytes())


def view_angles(arm, name):
    pwm = dict(zip(('000', '001', '002', '003'), arm['view_poses'][name]))
    return {s['name']: ag.pwm_angle(arm, s['servo'], pwm[s['servo']]) for s in arm['joints']}


def project(arm, angles, base, target):
    _, _, cp, cR = ag.fk(arm, angles, base)
    cam = arm['camera']
    size = (int(CFG['camera']['width']), int(CFG['camera']['height']))
    return vc.project(target, size, vc.fx_from_hfov(size[0], float(cam['hfov_rad'])), (cp, cR)), size


def fixed_view(arm, base, target, step=.01):
    """Raised, forward (base yaw and wrist roll 1500) view posture whose camera axis passes nearest `target`, with the
    compact limits of the existing taught views (tip at most 0.2 m ahead of the mount, at least 0.95 m high)."""
    n = ag.names(arm)
    lim = ag.limits(arm)
    x, y, z, byaw = base
    mount = np.array([x, y, z])+ag.rot([0, 0, 1], byaw)@np.array(arm['mount_xyz'], float)
    best, err = None, math.inf
    for t1 in np.arange(lim[n[1]][0], lim[n[1]][1], step):
        for t2 in np.arange(lim[n[2]][0], lim[n[2]][1], step):
            a = {n[0]: 0.0, n[1]: float(t1), n[2]: float(t2), n[3]: 0.0}
            tip, _, cp, cR = ag.fk(arm, a, base)
            ahead = (tip[0]-mount[0])*math.cos(byaw)+(tip[1]-mount[1])*math.sin(byaw)
            if tip[2] < .95 or ahead > .2:
                continue
            v = np.asarray(target, float)-cp
            e = math.acos(max(-1.0, min(1.0, float(cR[:, 0]@v)/float(np.linalg.norm(v)))))
            if e < err:
                best, err = a, e
    pwm = ag.to_pwm(arm, best)
    return [pwm['000'], pwm['001'], pwm['002'], pwm['003']], math.degrees(err)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--p1-label', default=None, help='shipping label on P1 (3 digits), default classroom.yaml')
    if ap.parse_args().p1_label:
        CFG['parcels']['items'][0]['label'] = ap.parse_args().p1_label
    OUT.mkdir(parents=True, exist_ok=True)
    arm, demo, pal = WORLD['arm'], WORLD['dressing']['demo_buttons'], WORLD['parcel']['pallet']
    cls, part, poses = CFG['classroom'], CFG['partition'], CFG['poses']
    report = {'validation_scope': 'simulation geometry only (assumed dimensions)'}

    # -------- press tips (taught cycles + simulated low-row cycles) -> board heights
    sp = ag.servo_protocol(ROOT)
    low = add_low_cycles(sp, arm, CFG['boards']['low_row_dz_m'], arm['press_cycle'])
    tips = {}
    for key in ('call', 'destination'):
        tips[key] = {'top': ag.taught_press(arm, sp, int(arm['press_cycle'][key])),
                     'low': ag.taught_press(arm, sp, low[key])}
    call_cz = tips['call']['top']['z']-[c for c in demo['hall_caps'] if c[0] == 'UP'][0][2]
    floor_cz = tips['destination']['top']['z']-[c for c in demo['car_caps'] if c[0] == '1'][0][2]

    # -------- solid geometry (collisions, LiDAR, map)
    x0, x1 = cls['x_range']
    y0, y1 = cls['y_range']
    t, wh = float(cls['wall_thickness_m']), float(cls['wall_height_m'])
    walls = [((x0+x1)/2, y0-t/2, x1-x0+2*t, t), ((x0+x1)/2, y1+t/2, x1-x0+2*t, t),
             (x0-t/2, (y0+y1)/2, t, y1-y0), (x1+t/2, (y0+y1)/2, t, y1-y0)]
    py0, py1 = part['y_range']
    pt, ph = float(part['thickness_m']), float(part['height_m'])
    ax = CFG['area'][0]
    partitions = [(-pt/2, (py0+py1)/2, pt, py1-py0), (ax+pt/2, (py0+py1)/2, pt, py1-py0)]
    desks = [tuple(d) for d in cls['desks']]
    disp = CFG['display']

    # display centre: display_view camera axis meets the display plane from the nominal viewing stance
    base = (float(disp['nominal_robot_x']), float(disp['nominal_robot_y']), 0.0, -math.pi/2)
    _, _, cp, cR = ag.fk(arm, view_angles(arm, 'display_view'), base)
    axis = cR[:, 0]
    yf = float(disp['y_face'])
    s = (yf-cp[1])/axis[1]
    hit = cp+s*axis
    dcx, dcz = float(hit[0]), float(hit[2])
    report['display'] = {'centre': [round(dcx, 3), yf, round(dcz, 3)], 'camera_distance_m': round(float(s), 3),
                         'from': 'display_view camera axis at robot (%.2f, %.2f) facing -y' % base[:2]}
    dw, dh = disp['size_m']
    sw, sd = disp['stand_m']
    stand_h = dcz-dh/2
    stand = (dcx, yf-.02-sd/2, sw, sd)

    solids = [solid_model('classroom_walls', [(x, y, wh/2, sx, sy, wh, 0.0) for x, y, sx, sy in walls], dr.WALL),
              solid_model('partitions', [(x, y, ph/2, sx, sy, ph, 0.0) for x, y, sx, sy in partitions], PARTITION),
              solid_model('desks', [(x, y, cls['desk_height_m']/2, sx, sy, cls['desk_height_m'], 0.0)
                                    for x, y, sx, sy in desks], DESK),
              solid_model('display_stand', [(stand[0], stand[1], stand_h/2, sw, sd, stand_h, 0.0)], STAND)]
    boxes = [[x, y, sx, sy, 0.0] for x, y, sx, sy in walls+partitions+desks+[stand]]

    # -------- visual-only dressing: tape, boards, display panel + digits
    models = list(solids)
    models.append(tape_model(CFG['area'], poses, CFG['parcels']['items'], pal))
    board_model, link = dr.static_model('b_wall_buttons')
    caps = {}
    for label, face in dr.demo_board(link, 'call_', ax, CFG['boards']['call_y'], math.pi, call_cz,
                                      demo['hall_board_m'], demo['hall_caps'], demo).items():
        caps[label] = {'kind': 'hall', 'xyz': [round(v, 4) for v in face], 'normal': math.pi}
    for label, face in dr.demo_board(link, 'floor_', ax, CFG['boards']['floor_y'], math.pi, floor_cz,
                                      demo['car_board_m'], demo['car_caps'], demo).items():
        caps[label] = {'kind': 'car', 'xyz': [round(v, 4) for v in face], 'normal': math.pi}
    models.append(board_model)
    panel, plink = dr.static_model('display_panel')
    dr.add_visual(plink, 'panel', (dcx, yf-.01, dcz), (0, 0, 0), 'box', (dw, .02, dh), dr.BLACK)
    models.append(panel)
    digits = {}
    for d in '1234':
        model, entry = dr.digit_model(f'display_digit_{d}', d, dcx, yf+.004, dcz, math.pi/2, [0.0, 0.0, 0.0],
                                      height=float(disp['digit_height_m']))
        models.append(model)
        digits[d] = {'name': f'display_digit_{d}', 'on': entry['on']}

    write_world(OUT/'classroom.world', models, 60.0, dressed=True)
    # robot URDF with the classroom camera resolution (same generator as sim/real_maps, camera size patched)
    import shutil
    import generate_worlds as gw
    cam = gw.PARAMS['arm']['camera']
    saved = dict(cam)
    cam.update(width=int(CFG['camera']['width']), height=int(CFG['camera']['height']),
               update_rate_hz=float(CFG['camera']['update_rate_hz']))
    try:
        gw.robot(gw.PARAMS['lidar']['noise_enabled'], 'robot_arm_cam_classroom.urdf', arm=True, camera=True)
    finally:
        cam.clear()
        cam.update(saved)
    shutil.move(str(gw.OUT/'robot_arm_cam_classroom.urdf'), str(OUT/'robot_arm_cam_classroom.urdf'))
    grid, origin = rasterise(boxes, (x0-t, x1+t), (y0-t, y1+t))
    write_pgm(OUT/'classroom_map.pgm', grid)
    (OUT/'classroom_map.yaml').write_text(yaml.safe_dump({'image': 'classroom_map.pgm', 'mode': 'trinary',
        'resolution': RES, 'origin': origin, 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .19}))
    (OUT/'classroom_boxes.json').write_text(json.dumps(boxes))

    # -------- label view: the pallet_view posture from the predock stance, else a computed fixed view
    lab = CFG['parcels']['label']
    p1 = CFG['parcels']['items'][0]
    parcel = pal['parcel']
    deck = pal['size_m'][2]
    label_xyz = (p1['xy'][0]+parcel['size_m'][0]/2, p1['xy'][1], deck+parcel['size_m'][2]/2)
    predock = (p1['xy'][0]+float(WORLD['perception']['predock_m']), p1['xy'][1], 0.0, math.pi)
    px, size = project(arm, view_angles(arm, 'pallet_view'), predock, label_xyz)
    size = (int(CFG['camera']['width']), int(CFG['camera']['height']))
    m = float(WORLD['perception']['view_margin_frac'])
    inside = px is not None and m*size[0] <= px[0] <= (1-m)*size[0] and m*size[1] <= px[1] <= (1-m)*size[1]
    if inside:
        label_view = {'name': 'pallet_view', 'pwm': arm['view_poses']['pallet_view'], 'pixel': [round(v, 1) for v in px]}
    else:
        pwm, err = fixed_view(arm, predock, label_xyz)
        angles = {s['name']: ag.pwm_angle(arm, s['servo'], v) for s, v in zip(arm['joints'], pwm)}
        px2, _ = project(arm, angles, predock, label_xyz)
        label_view = {'name': 'label_view', 'pwm': pwm, 'axis_error_deg': round(err, 2),
                      'pixel': None if px2 is None else [round(v, 1) for v in px2],
                      'pallet_view_pixel': None if px is None else [round(v, 1) for v in px],
                      'note': '[시뮬 가정] 송장 보기 자세. 실물 팔은 티칭 필요'}

    scene = {'validation_scope': 'simulation', 'area': CFG['area'], 'poses': poses, 'caps': caps,
             'boards': {'call': {'y': CFG['boards']['call_y'], 'centre_z': round(call_cz, 4)},
                        'floor': {'y': CFG['boards']['floor_y'], 'centre_z': round(floor_cz, 4)}},
             'press_tips': tips, 'low_cycles': low,
             'low_cycle_pwm': {key: sp.POSE_TABLES[n] for key, n in low.items()},
             'display': {'centre': [dcx, yf, dcz], 'normal': math.pi/2, 'digits': digits, 'hidden_z': dr.HIDDEN_Z},
             'parcels': {'pallet_yaw': CFG['parcels']['pallet_yaw'], 'items': CFG['parcels']['items'], 'label': lab,
                         'label_xyz_p1': [round(v, 4) for v in label_xyz]},
             'label_view': label_view, 'boxes': 'classroom_boxes.json', 'map': 'classroom_map.yaml'}
    (OUT/'classroom_scene.json').write_text(json.dumps(scene, indent=1))
    report.update({'caps': caps, 'board_centre_z': scene['boards'], 'label_view': label_view,
                   'tips_z': {k: {r: v[r]['z'] for r in v} for k, v in tips.items()},
                   'map_cells': list(grid.shape), 'boxes': len(boxes)})
    print(json.dumps(report, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
