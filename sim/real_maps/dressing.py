"""Simulation-only VISUAL dressing: elevator cabin/landing, pickup marking, pallet and parcel looks (--dressed, --load, --parcels).

Every element here is a <visual> of a static model: no <collision>, no mass. Gazebo's 'ray' LiDAR and the
physics only see collision geometry, so Nav2 inputs, contacts and judgement metrics are those of
building.world (generate_worlds.py checks the two worlds' collisions are identical). Positions and sizes
marked [가정값] in world_params.yaml `dressing` are illustrative, not measured (U-E01, U-E07).
"""
import math
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import numpy as np
import yaml

import arm_geometry as ag

# Lift carrier geometry: mast x of the arm-run URDF (world_params lift.mast_x_m, [사진 추정] override of the source
# URDF 0.165) and lift_joint origin z; shared with parcel_mock and sim_adapters.
LIFT_BASE_X = float(yaml.safe_load((Path(__file__).resolve().parent/'world_params.yaml').read_text())['lift']['mast_x_m'])
LIFT_ORIGIN_Z = 0.05
MAST_HALF = 0.04         # URDF lead_screw_base box 0.08 m deep: the load rests ahead of the mast front face


def pallet_center_ahead(pallet):
    """Pallet centre ahead of base_footprint when the carrier sits in its pocket (rear face rear_gap ahead of the
    mast front face)."""
    return LIFT_BASE_X+MAST_HALF+float(pallet['rear_gap_m'])+pallet['size_m'][0]/2


def pickup_pallet_pose(stop, pallet):
    """Pallet bottom-centre pose: to the right of the f1_locker stop pose (entry direction), pocket facing it."""
    x, y, yaw = stop[:3]
    right, ahead = float(pallet['pickup_right_m']), float(pallet['pickup_ahead_m'])
    return [x+math.cos(yaw)*ahead+math.sin(yaw)*right, y+math.sin(yaw)*ahead-math.cos(yaw)*right, yaw-math.pi/2]


STEEL = '.70 .72 .75 1'
STEEL_DARK = '.52 .54 .57 1'
GRANITE = '.24 .24 .26 1'
WALL = '.93 .92 .89 1'
FLOOR = '.80 .79 .76 1'
TAPE = '.78 .66 .45 1'
WHITE = '.97 .97 .95 1'
BLACK = '.05 .05 .06 1'
BUTTON = '.30 .60 1 1'      # blue LED-ring button (sim colour class for the arm camera)
AMBER = '1 .62 .12 1'


def material(element, rgba, emissive=None, specular=None):
    mat = ET.SubElement(element, 'material')
    ET.SubElement(mat, 'ambient').text = rgba
    ET.SubElement(mat, 'diffuse').text = rgba
    if specular:
        ET.SubElement(mat, 'specular').text = specular
    if emissive:
        ET.SubElement(mat, 'emissive').text = emissive


def add_visual(link, name, xyz, rpy, shape, dims, rgba, emissive=None, specular=None):
    vis = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(vis, 'cast_shadows').text = 'false'
    ET.SubElement(vis, 'pose').text = ' '.join(f'{v:.5f}' for v in (*xyz, *rpy))
    geom = ET.SubElement(ET.SubElement(vis, 'geometry'), shape)
    if shape == 'box':
        ET.SubElement(geom, 'size').text = ' '.join(f'{v:.5f}' for v in dims)
    else:
        ET.SubElement(geom, 'radius').text = f'{dims[0]:.5f}'
        ET.SubElement(geom, 'length').text = f'{dims[1]:.5f}'
    material(vis, rgba, emissive, specular)
    return vis


def static_model(name, x=0.0, y=0.0, z=0.0, yaw=0.0):
    model = ET.Element('model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.5f}'
    return model, ET.SubElement(model, 'link', name='visual_only')


def rgba(color):
    return ' '.join(f'{c:.3f}' for c in color)+' 1'


# ------------------------------------------------------------------ parcels
def parcel_visuals(link, size, color, prefix='', xyz=(0.0, 0.0, 0.0), yaw=0.0):
    """Cardboard box centred at xyz: body, packing tape (top and two end faces), shipping label + barcode."""
    sx, sy, sz = size
    c, s = math.cos(yaw), math.sin(yaw)

    def put(name, local, dims, col):
        lx, ly, lz = local
        add_visual(link, prefix+name, (xyz[0]+c*lx-s*ly, xyz[1]+s*lx+c*ly, xyz[2]+lz), (0, 0, yaw), 'box', dims, col)

    put('body', (0, 0, 0), (sx, sy, sz), color)
    put('tape_top', (0, 0, sz/2+.0008), (sx+.002, .048, .0016), TAPE)
    drop = min(.08, sz*.45)
    put('tape_front', (sx/2+.0008, 0, sz/2-drop/2), (.0016, .048, drop), TAPE)
    put('tape_back', (-sx/2-.0008, 0, sz/2-drop/2), (.0016, .048, drop), TAPE)
    lw, lh = min(.12, sx*.5), min(.08, sz*.6)
    put('label', (-sx*.12, sy/2+.0008, 0), (lw, .0016, lh), WHITE)
    for i in range(6):
        put(f'bar{i}', (-sx*.12-lw*.35+i*lw*.14, sy/2+.0018, -lh*.22), (lw*.05+(i % 2)*lw*.03, .0012, lh*.35), BLACK)


def parcel_model_xml(name, size, color):
    sdf = ET.Element('sdf', version='1.6')
    model = ET.SubElement(sdf, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    parcel_visuals(ET.SubElement(model, 'link', name='box'), size, rgba(color))
    return ET.tostring(sdf, encoding='unicode')


# ------------------------------------------------------------------ map grid helpers
class Grid:
    """Occupancy mask in a floor's MAP frame (same cell convention as generate_worlds.building)."""

    def __init__(self, mask, config):
        self.mask, self.res = mask, float(config['resolution'])
        self.ox, self.oy, self.angle = (float(v) for v in config['origin'])
        self.c, self.s = math.cos(self.angle), math.sin(self.angle)
        self.h, self.w = mask.shape

    def cell(self, x, y):
        dx, dy = x-self.ox, y-self.oy
        lx, ly = self.c*dx+self.s*dy, -self.s*dx+self.c*dy
        return self.h-1-int(math.floor(ly/self.res)), int(math.floor(lx/self.res))

    def center(self, row, col):
        lx, ly = (col+.5)*self.res, (self.h-row-.5)*self.res
        return self.ox+self.c*lx-self.s*ly, self.oy+self.s*lx+self.c*ly

    def occ(self, x, y):
        row, col = self.cell(x, y)
        return 0 <= row < self.h and 0 <= col < self.w and bool(self.mask[row, col])

    def ray(self, x, y, theta, max_d=2.0, step=.01):
        """Distance to the first occupied cell along theta (None if none within max_d)."""
        for i in range(1, int(max_d/step)+1):
            d = i*step
            if self.occ(x+math.cos(theta)*d, y+math.sin(theta)*d):
                return d
        return None

    def exit(self, x, y, theta, max_d=.8, step=.01):
        """Distance until leaving occupied cells along theta (wall face), None if still inside at max_d."""
        for i in range(0, int(max_d/step)+1):
            d = i*step
            if not self.occ(x+math.cos(theta)*d, y+math.sin(theta)*d):
                return d
        return None

    def boxes(self, region, rectangles):
        """Merged rectangles of a boolean region -> (cx, cy, sx, sy, yaw) in the map frame."""
        out = []
        for x0, x1, row0, row1 in rectangles(region):
            y0, y1 = self.h-row1, self.h-row0
            lx, ly = (x0+x1)*self.res/2, (y0+y1)*self.res/2
            out.append((self.ox+self.c*lx-self.s*ly, self.oy+self.s*lx+self.c*ly,
                        (x1-x0)*self.res, (y1-y0)*self.res, self.angle))
        return out


def cabin_region(grid, inside, door, radius):
    """Free cells reachable from the cabin reference point without crossing the closed door line, bounded by
    the walls hit from the reference pose (back/left/right rays, door line ahead): raw scans have gaps."""
    cx, cy = door['center']
    a, half = door['angle'], door['opening_m']/2+.15
    ix, iy, iyaw = inside
    fx, fy = math.cos(iyaw), math.sin(iyaw)
    back = grid.ray(ix, iy, iyaw+math.pi, 1.6) or radius
    left = grid.ray(ix, iy, iyaw+math.pi/2, 1.6) or radius
    right = grid.ray(ix, iy, iyaw-math.pi/2, 1.6) or radius
    ahead = (cx-ix)*fx+(cy-iy)*fy
    start = grid.cell(*inside[:2])
    if not (0 <= start[0] < grid.h and 0 <= start[1] < grid.w) or grid.mask[start]:
        return None
    region = np.zeros_like(grid.mask, dtype=bool)
    queue = deque([start])
    region[start] = True
    while queue:
        row, col = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            r, c = row+dr, col+dc
            if not (0 <= r < grid.h and 0 <= c < grid.w) or region[r, c] or grid.mask[r, c]:
                continue
            x, y = grid.center(r, c)
            if math.hypot(x-inside[0], y-inside[1]) > radius:
                continue
            along = math.cos(a)*(x-cx)+math.sin(a)*(y-cy)
            across = -math.sin(a)*(x-cx)+math.cos(a)*(y-cy)
            if abs(across) < .08 and abs(along) < half:
                continue
            fl, ll = (x-ix)*fx+(y-iy)*fy, -(x-ix)*fy+(y-iy)*fx
            if not (-back-.03 <= fl <= ahead and -right-.03 <= ll <= left+.03):
                continue
            region[r, c] = True
            queue.append((r, c))
    return region


def dilate(region):
    out = region.copy()
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            out |= np.roll(np.roll(region, dr, 0), dc, 1)
    return out


# 7-segment layout (u right, v up, in digit-height units) and the floors shown on the cabin display.
SEGMENTS = {'a': (0, .5, 1, 0), 'b': (.28, .25, 0, 1), 'c': (.28, -.25, 0, 1), 'd': (0, -.5, 1, 0),
            'e': (-.28, -.25, 0, 1), 'f': (-.28, .25, 0, 1), 'g': (0, 0, 1, 0)}
DIGITS = {'1': 'bc', '2': 'abged', '3': 'abgcd', '4': 'fgbc'}


def lit_model(name, x, y, z, normal, offset):
    """Amber cap over one button (moved in front of the button when lit, parked below the floor otherwise)."""
    model, link = static_model(name, x+offset[0], y+offset[1], offset[2]+HIDDEN_Z, 0.0)
    add_visual(link, 'cap', (0, 0, 0), (math.pi/2, 0, normal+math.pi/2), 'cylinder', (.024, .006), AMBER,
               emissive='1 .55 .05 1')
    return model, {'on': [x+offset[0], y+offset[1], z+offset[2], 0.0], 'normal': normal}


def digit_model(name, digit, x, y, z, normal, offset, height=.12):
    """Red 7-segment digit facing `normal` (viewer looks along -normal)."""
    model, link = static_model(name, x+offset[0], y+offset[1], offset[2]+HIDDEN_Z, normal-math.pi/2)
    for seg in DIGITS[digit]:
        u, v, horizontal, vertical = SEGMENTS[seg]
        size = (height*.5, .004, height*.11) if horizontal else (height*.11, .004, height*.45)
        add_visual(link, f'seg_{seg}', (-u*height, 0, v*height), (0, 0, 0), 'box', size, '1 .15 .1 1',
                   emissive='1 .1 .05 1')
    return model, {'on': [x+offset[0], y+offset[1], z+offset[2], normal-math.pi/2]}


HIDDEN_Z = -5.0
ROOT = Path(__file__).resolve().parents[2]


def wall_face(grid, x, y, theta, max_d=1.4, half=.2, step=.02):
    """Wall surface hit along theta from (x, y), as the least-squares line through neighbouring hits (scanned walls
    are 5 cm cell stairs; the robot LiDAR sees their average) -> ((x, y) on that line on the ray, outward normal
    angle), or None."""
    ux, uy = math.cos(theta), math.sin(theta)
    pts = []
    for k in np.arange(-half, half+1e-9, step):
        px, py = x-uy*k, y+ux*k
        dist = grid.ray(px, py, theta, max_d, step=.002)
        if dist is not None:
            pts.append((px+ux*dist, py+uy*dist))
    if len(pts) < 5:
        return None
    pts = np.array(pts)
    centre = pts.mean(axis=0)
    n = np.linalg.svd(pts-centre)[2][1]
    if n[0]*ux+n[1]*uy > 0:
        n = -n
    t = float((centre-np.array([x, y]))@n)/float(ux*n[0]+uy*n[1])
    return (x+ux*t, y+uy*t), math.atan2(float(n[1]), float(n[0]))


def demo_board(link, prefix, x, y, normal, centre_z, size, caps, demo):
    """Demo button board (user photos 2026-10-13): black board on the wall surface point (x, y) facing `normal`,
    white round caps with a black digit (7 segments) or triangle. caps = [[label, right, up], ...] in the frame of
    a viewer facing the board. Returns {label: cap face centre (x, y, z)}."""
    w, h = size
    dia, ch, t = float(demo['cap_diameter_m']), float(demo['cap_height_m']), float(demo['board_thickness_m'])
    nx, ny = math.cos(normal), math.sin(normal)
    rx, ry = -ny, nx                                    # viewer's right
    # sim only: a mounting block from the averaged wall line covers the scanned wall's 5 cm stair teeth
    block = float(demo['sim_mount_block_m'])
    add_visual(link, prefix+'block', (x+nx*block/2, y+ny*block/2, centre_z), (0, 0, normal+math.pi/2), 'box',
               (w+.02, block, h+.02), STEEL)
    x, y = x+nx*block, y+ny*block
    add_visual(link, prefix+'board', (x+nx*t/2, y+ny*t/2, centre_z), (0, 0, normal+math.pi/2), 'box', (w, t, h), BLACK)
    faces = {}
    gh = .03                                            # glyph height on the cap face
    for label, r, u in caps:
        bx, by, bz = x+rx*r, y+ry*r, centre_z+u
        add_visual(link, f'{prefix}cap_{label}', (bx+nx*(t+ch/2), by+ny*(t+ch/2), bz), (math.pi/2, 0, normal+math.pi/2),
                   'cylinder', (dia/2, ch), WHITE)
        fx, fy = bx+nx*(t+ch+.0006), by+ny*(t+ch+.0006)
        faces[label] = (bx+nx*(t+ch), by+ny*(t+ch), bz)
        if label in DIGITS:
            bars = [((SEGMENTS[g][0]*gh, SEGMENTS[g][1]*gh),
                     (gh*.5, gh*.13) if SEGMENTS[g][2] else (gh*.13, gh*.45)) for g in DIGITS[label]]
        else:                                           # triangle: stacked bars, wide at the base
            sign = 1 if label == 'UP' else -1
            bars = [((0, -sign*.008), (.026, .006)), ((0, 0), (.017, .006)), ((0, sign*.008), (.008, .006))]
        for k, ((du, dv), (bw, bh)) in enumerate(bars):
            add_visual(link, f'{prefix}glyph_{label}_{k}', (fx+rx*du, fy+ry*du, bz+dv), (0, 0, normal+math.pi/2),
                       'box', (bw, .001, bh), BLACK)
    return faces

def elevator_dressing(floor, grid, door, inside, offset, params, wall_h, rectangles, lit_up):
    """Cabin floor + stainless cladding + handrail + car panel, and the landing call panel + door jambs."""
    d = params['dressing']
    model, link = static_model(f'{floor.lower()}_elevator_dressing', *offset)
    info = {'floor': floor}
    cx, cy = door['center']
    a, width = door['angle'], door['opening_m']
    ax, ay = math.cos(a), math.sin(a)
    ix, iy, iyaw = inside
    nx, ny = -ay, ax
    if nx*(ix-cx)+ny*(iy-cy) < 0:
        nx, ny = -nx, -ny                      # n points into the cabin
    region = cabin_region(grid, inside, door, float(d['cabin_region_radius_m']))
    area = float(region.sum())*grid.res**2 if region is not None else 0.0
    info['cabin_region_m2'] = round(area, 3)
    if region is not None and 1.0 <= area <= 6.0:
        for i, (bx, by, sx, sy, yaw) in enumerate(grid.boxes(region, rectangles)):
            add_visual(link, f'cabin_floor{i}', (bx, by, .002), (0, 0, yaw), 'box', (sx, sy, .004), GRANITE,
                       specular='.4 .4 .4 1')
        # The flattened F2 world already has continuous plane walls. Raster
        # cladding would visually reintroduce the old staircase/corrugation.
        flat_f2 = floor == 'F2' and params.get('f2_cleanup', {}).get('enabled', False)
        clad = [] if flat_f2 else grid.boxes(dilate(region) & grid.mask, rectangles)
        if flat_f2:
            info['wall_surface_source'] = 'continuous flat world walls (no raster cladding)'
        for i, (bx, by, sx, sy, yaw) in enumerate(clad):
            add_visual(link, f'cabin_wall{i}', (bx, by, (wall_h+.006)/2), (0, 0, yaw), 'box',
                       (sx+.006, sy+.006, wall_h+.006), STEEL, specular='.9 .9 .9 1')
        info['cabin_wall_boxes'] = len(clad)
    else:
        info['cabin_region_skipped'] = True
    fx, fy = math.cos(iyaw), math.sin(iyaw)
    lx, ly = -fy, fx
    back = grid.ray(ix, iy, iyaw+math.pi, 1.6)
    if back:
        qx, qy = ix-fx*(back-.25), iy-fy*(back-.25)
        left, right = grid.ray(qx, qy, iyaw+math.pi/2, 1.6), grid.ray(qx, qy, iyaw-math.pi/2, 1.6)
        if left and right and left+right > .8:
            length, shift = left+right-.30, (left-right)/2
            rx, ry = ix-fx*(back-.07)+lx*shift, iy-fy*(back-.07)+ly*shift
            z = float(d['handrail_z_m'])
            add_visual(link, 'handrail', (rx, ry, z), (math.pi/2, 0, iyaw+math.pi), 'cylinder', (.018, length), STEEL_DARK,
                       specular='1 1 1 1')
            for k, sgn in enumerate((-1, 1)):
                bx, by = rx+lx*sgn*(length/2-.12)-fx*.035, ry+ly*sgn*(length/2-.12)-fy*.035
                add_visual(link, f'handrail_bracket{k}', (bx, by, z), (0, 0, iyaw), 'box', (.07, .02, .02), STEEL_DARK)
            info['handrail_m'] = round(length, 3)
    # Demo button boards (user photos 2026-10-13), at the height where the Jetson's taught press cycles press
    # (forward kinematics of servo_protocol.py under the assumed arm geometry): world_params dressing.demo_buttons.
    demo, arm = d['demo_buttons'], params['arm']
    sp = ag.servo_protocol(ROOT)
    # Car board: on the wall the robot faces after entering along the door normal and turning right 90 deg (the
    # left wall seen from inside), placed so that the robot pressing the target floor with the taught destination
    # cycle stands level with the cabin stand point (it drives straight at the wall and reverses straight back).
    car = ag.taught_press(arm, sp, int(arm['press_cycle']['destination']))
    # the robot stands at the cabin depth on the door axis (Machine.door_path / go_to_cabin_centre)
    depth = (ix-cx)*nx+(iy-cy)*ny
    ix, iy = cx+nx*depth, cy+ny*depth
    target = '2' if floor == 'F1' else '1'
    h = math.atan2(ny, nx)-math.pi/2                   # heading facing the board wall
    yaw = h-car['yaw']                                 # press stance heading (square press, see press_stance)
    tip = (math.cos(yaw)*car['xy'][0]-math.sin(yaw)*car['xy'][1], math.sin(yaw)*car['xy'][0]+math.cos(yaw)*car['xy'][1])
    push = float(params['perception']['press_push_m'])
    along = -(tip[0]+math.cos(h)*push)*nx-(tip[1]+math.sin(h)*push)*ny   # robot centre - button, along n
    hit = wall_face(grid, ix-nx*along, iy-ny*along, h)
    if hit is not None:
        (bx, by), face_yaw = hit
        caps = {lab: (r, u) for lab, r, u in demo['car_caps']}
        rx, ry = -math.sin(face_yaw), math.cos(face_yaw)    # viewer's right (facing the board)
        r_t, u_t = caps[target]
        faces = demo_board(link, 'car_', bx-rx*r_t, by-ry*r_t, face_yaw, car['z']-u_t, demo['car_board_m'],
                           demo['car_caps'], demo)
        info['car_board'] = {'xy': [round(bx-rx*r_t, 3), round(by-ry*r_t, 3)], 'target': target,
                             'taught_tip': car}
        info['car_buttons'] = {'F1': faces['1'], 'F2': faces['2'], 'normal': face_yaw}
    # Call board on the RIGHT of the door as seen from the landing (facing the elevator = facing +n), user
    # 2026-10-13 (required: generation stops if no wall is found there); lobby (F1) presses UP, the target floor
    # (F2) DOWN, so that cap is at the taught call press height.
    call = ag.taught_press(arm, sp, int(arm['press_cycle']['call']))
    right = 1 if ax*ny-ay*nx > 0 else -1           # door-line direction a·(n rotated -90 deg)
    side = d['hall_panel_side'][floor]
    if side == 'left':                             # explicit per-floor exception, reason in world_params
        right = -right
    label = 'UP' if lit_up else 'DOWN'
    caps = {lab: (r, u) for lab, r, u in demo['hall_caps']}
    edge = width/2+float(d['hall_panel_offset_m'])
    for sgn in (right,):
        hit = None
        for t in (0, .05, .1, .15, .2):             # beside the door edge; landing wall = near the door line
            sx, sy = cx+ax*sgn*(edge+t), cy+ay*sgn*(edge+t)
            hit = wall_face(grid, sx-nx*.3, sy-ny*.3, math.atan2(ny, nx), max_d=.45)   # from the landing side
            if hit is not None and abs((hit[0][0]-cx)*nx+(hit[0][1]-cy)*ny) <= .15:
                break
            hit = None                              # scan noise in the landing, not the wall beside the door
        if hit is None:
            # sparse raw scan (F2): landing-side face of the nearest wall cell, oriented along the door line
            spots = [(edge+t, u) for t in (0, .05, .1, .15, .2) for u in (0, -.05, .05, -.1, .1, -.15, .15, -.2, .2)]
            cell = next(((cx+ax*sgn*t+nx*u, cy+ay*sgn*t+ny*u) for t, u in spots
                         if grid.occ(cx+ax*sgn*t+nx*u, cy+ay*sgn*t+ny*u)), None)
            face = None if cell is None else grid.exit(cell[0], cell[1], math.atan2(-ny, -nx), step=.002)
            if face is None:
                continue
            hit = ((cell[0]-nx*face, cell[1]-ny*face), math.atan2(-ny, -nx))
        (hx, hy), face_yaw = hit
        rx, ry = -math.sin(face_yaw), math.cos(face_yaw)
        r_t, u_t = caps[label]
        faces = demo_board(link, 'hall_', hx-rx*r_t, hy-ry*r_t, face_yaw, call['z']-u_t, demo['hall_board_m'],
                           demo['hall_caps'], demo)
        info['hall_board'] = {'xy': [round(hx-rx*r_t, 3), round(hy-ry*r_t, 3)], 'target': label, 'taught_tip': call,
                              'side': side}
        info['hall_button'] = {'xyz': faces[label], 'normal': face_yaw}
    if 'hall_button' not in info:
        raise SystemExit(f'{floor}: no wall found on the {side} of the elevator door for the call board')
    # Floor display above the entrance (cabin side): header over the door line, black screen facing the cabin.
    disp = d['floor_display']
    dz, dw, dh = float(disp['center_z_m']), float(disp['width_m']), float(disp['height_m'])
    add_visual(link, 'door_header', (cx, cy, wall_h+(dz+dh/2+.04-wall_h)/2), (0, 0, a), 'box',
               (width+.20, .22, dz+dh/2+.04-wall_h), STEEL, specular='.9 .9 .9 1')
    sx, sy = cx+nx*.12, cy+ny*.12
    add_visual(link, 'floor_display', (sx, sy, dz), (0, 0, a), 'box', (dw, .02, dh), BLACK)
    info['display'] = {'xyz': [sx+nx*.012, sy+ny*.012, dz], 'normal': math.atan2(ny, nx)}
    for k, sgn in enumerate((-1, 1)):
        jx, jy = cx+ax*sgn*(width/2+.035), cy+ay*sgn*(width/2+.035)
        add_visual(link, f'door_jamb{k}', (jx, jy, (wall_h+.01)/2), (0, 0, a), 'box', (.07, .22, wall_h+.01), STEEL,
                   specular='.9 .9 .9 1')
    return model, info


def call_view_pose(grid, face_xyz, normal, start_xy, body_r=.42, path_r=.25):
    """The taught spot to look at the call board from: on its normal 1.0 m out, closer (down to 0.5 m) or a bit to
    the side when the scanned landing has walls there; a robot-size disc must be free and the straight path from
    the elevator-entry waypoint clear. -> [x, y, yaw facing the board] (map) or None."""
    nx, ny = math.cos(normal), math.sin(normal)

    def free(x, y, r):
        return not any(grid.occ(x+r*k/3*math.cos(a), y+r*k/3*math.sin(a))
                       for k in range(4) for a in np.linspace(0, 2*math.pi, 16, endpoint=False))
    for d in np.arange(1.0, .49, -.05):
        for lat in (0.0, .1, -.1, .2, -.2):
            x, y = face_xyz[0]+nx*d-ny*lat, face_xyz[1]+ny*d+nx*lat
            n = max(2, int(math.hypot(x-start_xy[0], y-start_xy[1])/.05))
            if free(x, y, body_r) and all(free(start_xy[0]+(x-start_xy[0])*k/n, start_xy[1]+(y-start_xy[1])*k/n,
                                               path_r) for k in range(n+1)):
                return [round(x, 3), round(y, 3), round(math.atan2(-ny, -nx), 4)]
    return None


def indicator_models(floor, info, offset):
    """Dynamic indicator models for one cabin/landing copy (all parked below the floor; sim_elevator moves them):
    display digits for each floor, lit caps for the car floor buttons and the landing call button."""
    models, entries = [], {'digits': {}, 'car_lit': {}, 'hall_lit': None, 'models': {}}
    tag = floor.lower()
    disp = info['display']
    for shown in ('F1', 'F2'):
        name = f'{tag}_display_{shown.lower()}'
        model, entry = digit_model(name, shown[1], *disp['xyz'], disp['normal'], offset)
        models.append(model)
        entries['models'][name], entries['digits'][shown] = entry, name
    for target in ('F1', 'F2'):
        if 'car_buttons' in info:
            name = f'{tag}_car_lit_{target.lower()}'
            model, entry = lit_model(name, *info['car_buttons'][target], info['car_buttons']['normal'], offset)
            models.append(model)
            entries['models'][name], entries['car_lit'][target] = entry, name
    if 'hall_button' in info:
        name = f'{tag}_hall_lit'
        model, entry = lit_model(name, *info['hall_button']['xyz'], info['hall_button']['normal'], offset)
        models.append(model)
        entries['models'][name], entries['hall_lit'] = entry, name
    entries['display_map'] = disp
    return models, entries


def pallet_visuals(link, pallet):
    """2-way wooden pallet, origin at bottom centre, fork entry along -x: two edge stringers, deck boards on top.
    The centre pocket (between the stringers, below the deck) is open for the lift carrier."""
    length, width, height = pallet['size_m']
    sw, deck = float(pallet['stringer_width_m']), float(pallet['deck_thickness_m'])
    pocket, wood = float(pallet['pocket_height_m']), rgba(pallet['color'])
    for k, sgn in enumerate((-1, 1)):
        add_visual(link, f'stringer{k}', (0, sgn*(width/2-sw/2), pocket/2), (0, 0, 0), 'box', (length, sw, pocket), wood)
    n = int(pallet['deck_boards'])
    board = length/(2*n-1)
    for k in range(n):
        x = -length/2+board/2+k*2*board
        add_visual(link, f'deck{k}', (x, 0, height-deck/2), (0, 0, 0), 'box', (board, width, deck), wood)
    # Two cross boards under the deck tie the stringers (top of the pocket stays at pocket_height_m).
    for k, sgn in enumerate((-1, 1)):
        add_visual(link, f'cross{k}', (sgn*(length/2-board/2), 0, pocket+(height-deck-pocket)/2), (0, 0, 0), 'box',
                   (board, width, max(height-deck-pocket, .002)), wood)


def pallet_model_xml(name, pallet):
    sdf = ET.Element('sdf', version='1.6')
    model = ET.SubElement(sdf, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    pallet_visuals(ET.SubElement(model, 'link', name='pallet'), pallet)
    return ET.tostring(sdf, encoding='unicode')


def pickup_marking(stop, offset, params):
    """Yellow floor tape around the pallet pickup spot (right of the f1_locker entry direction; no locker)."""
    pallet = params['parcel']['pallet']
    x, y, yaw = pickup_pallet_pose(stop, pallet)
    margin = float(params['dressing']['pickup_marking_margin_m'])
    length, width = pallet['size_m'][0]+2*margin, pallet['size_m'][1]+2*margin
    model, link = static_model('f1_pickup_marking', x+offset[0], y+offset[1], offset[2], yaw)
    tape, t = '.95 .78 .10 1', .05
    for k, (cx, cy, sx, sy) in enumerate(((-length/2, 0, t, width), (length/2, 0, t, width),
                                          (0, -width/2, length, t), (0, width/2, length, t))):
        add_visual(link, f'tape{k}', (cx, cy, .002), (0, 0, 0), 'box', (sx, sy, .004), tape)
    return model, {'pallet_pose': [x, y, yaw]}


def delivery_mat(pose, offset):
    x, y, yaw = pose
    model, link = static_model('f2_delivery_mat', x+offset[0], y+offset[1], offset[2], yaw)
    add_visual(link, 'mat', (.45, 0, .003), (0, 0, 0), 'box', (.70, .50, .006), '.22 .30 .26 1')
    return model


def recolor(world_models, door_names):
    """Off-white walls and stainless door leaves (visual material only)."""
    for model in world_models:
        name = model.get('name')
        color = WALL if name.endswith('_real_building') else STEEL if name in door_names else None
        if color is None:
            continue
        for vis in model.iter('visual'):
            old = vis.find('material')
            if old is not None:
                vis.remove(old)
            material(vis, color, specular='.9 .9 .9 1' if color == STEEL else None)


def collision_signature(models):
    return sorted((m.get('name'), c.get('name'), c.findtext('pose'), ET.tostring(c.find('geometry'), encoding='unicode'))
                  for m in models for c in m.iter('collision'))
