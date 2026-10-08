#!/usr/bin/env python3
"""Exact raster-to-box Gazebo Classic worlds. Generated assets stay local."""
import hashlib
import importlib.util
import json
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / 'generated'


def noglass():
    source = ROOT / 'docs/field_review_20260915/f1_manual_clean_20260915_v3/build_v3.py'
    copied = OUT / 'build_v3.py'
    shutil.copyfile(source, copied)
    spec = importlib.util.spec_from_file_location('v3', copied)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    glass = [(357.3, 204.5), (383.6, 204.5)]
    assert m.INNER_WALLS_UV.count(glass) == 1
    m.INNER_WALLS_UV.remove(glass)
    outline = [m.to_raw(*p) for p in m.OUTLINE_UV]
    walls = [(outline[i], outline[(i + 1) % len(outline)])
             for i in range(len(outline)) if i not in m.OPEN_EDGES]
    walls += [tuple(m.to_raw(*p) for p in seg) for seg in m.INNER_WALLS_UV]
    ys, xs = np.mgrid[0:m.H, 0:m.W]
    px, py = xs + .5, ys + .5
    grid = np.full((m.H, m.W), 205, np.uint8)
    inside = np.fromiter((m.inside(x, y, outline) for x, y in zip(px.ravel(), py.ravel())),
                         bool, count=m.W*m.H).reshape(grid.shape)
    grid[inside] = 254
    mask = np.zeros_like(inside)
    for a, b in walls:
        mask |= m.seg_dist(px, py, a, b) <= m.WALL_HALF_WIDTH_PX
    grid[mask] = 0
    original = np.asarray(Image.open(ROOT / 'src/slam_pkg/maps/field/f1/f1_manual_clean_v3.pgm'))
    glass_mask = m.seg_dist(px, py, *[m.to_raw(*p) for p in glass]) <= m.WALL_HALF_WIDTH_PX
    rebuilt = grid.copy()
    rebuilt[glass_mask] = 0
    assert np.array_equal(rebuilt, original), 'v3 generator does not reproduce pinned raster'
    changed = grid != original
    assert np.array_equal(changed, glass_mask & ~mask), 'changes outside glass segment'
    assert np.all(original[changed] == 0) and np.all(grid[changed] == 254)
    image = OUT / 'f1_manual_clean_v3_noglass.pgm'
    Image.fromarray(grid).save(image)
    data = yaml.safe_load((ROOT / 'src/slam_pkg/maps/field/f1/f1_manual_clean_v3.yaml').read_text())
    data['image'] = image.name
    (OUT / 'f1_manual_clean_v3_noglass.yaml').write_text(yaml.safe_dump(data))
    return {'changed_cells': int(changed.sum()), 'outside_glass_changed_cells': 0,
            'original_sha256': hashlib.sha256(original.tobytes()).hexdigest(),
            'copied_generator_sha256': hashlib.sha256(copied.read_bytes()).hexdigest()}


def rectangles(mask):
    """Horizontal runs merged vertically when column intervals are identical."""
    active, result = {}, []
    for row in range(mask.shape[0] + 1):
        runs = []
        if row < mask.shape[0]:
            edges = np.flatnonzero(np.diff(np.r_[False, mask[row], False].astype(np.int8)))
            runs = list(zip(edges[::2].tolist(), edges[1::2].tolist()))
        current = set(runs)
        for interval in list(active):
            if interval not in current:
                result.append((*interval, active.pop(interval), row))
        for interval in runs:
            active.setdefault(interval, row)
    return result


def world(floor, yaml_path, suffix="", door_clear=False):
    config = yaml.safe_load(yaml_path.read_text())
    pixels = np.asarray(Image.open(yaml_path.parent / config['image']))
    probability = pixels.astype(float) / 255 if config['negate'] else (255-pixels.astype(float))/255
    mask = probability > config['occupied_thresh']
    removed = 0
    if door_clear:
        # User-authorized open doorway only; A/B worlds retain every raw point.
        ys, xs = np.mgrid[0:mask.shape[0], 0:mask.shape[1]]
        wx = config['origin'][0]+(xs+.5)*config['resolution']
        wy = config['origin'][1]+(mask.shape[0]-ys-.5)*config['resolution']
        da = 2.312744-math.pi/2
        dx,dy = wx+11.925,wy+1.925
        along = math.cos(da)*dx+math.sin(da)*dy
        across = -math.sin(da)*dx+math.cos(da)*dy
        opening = (np.abs(along)<.5)&(np.abs(across)<.4)
        removed = int((mask&opening).sum())
        mask = mask & ~opening
    rects = rectangles(mask)
    rebuilt = np.zeros_like(mask, dtype=np.uint16)
    resolution = config['resolution']
    ox, oy, angle = config['origin']
    c, s = math.cos(angle), math.sin(angle)
    boxes, max_error = [], 0.0
    sdf = ET.Element('sdf', version='1.6')
    w = ET.SubElement(sdf, 'world', name='default')
    state = ET.SubElement(w, 'plugin', name='gazebo_ros_state', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(state, 'ros'), 'namespace').text = '/gazebo'
    ET.SubElement(state, 'update_rate').text = '20'
    physics = ET.SubElement(w, 'physics', name='ode', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    ET.SubElement(w, 'gravity').text = '0 0 -9.8'
    # Local ground geometry avoids Gazebo's external model database.
    ground = ET.SubElement(w, 'model', name='ground')
    ET.SubElement(ground, 'static').text = 'true'
    link = ET.SubElement(ground, 'link', name='ground')
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, name=kind)
        plane = ET.SubElement(ET.SubElement(item, 'geometry'), 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'
        ET.SubElement(plane, 'size').text = '200 200'
    light = ET.SubElement(w, 'light', name='sun', type='directional')
    ET.SubElement(light, 'direction').text = '-.5 .1 -1'
    ET.SubElement(light, 'diffuse').text = '.8 .8 .8 1'
    model = ET.SubElement(w, 'model', name=f'{floor.lower()}_real_building')
    ET.SubElement(model, 'static').text = 'true'
    wall_link = ET.SubElement(model, 'link', name='walls')
    for i, (x0, x1, row0, row1) in enumerate(rects):
        rebuilt[row0:row1, x0:x1] += 1
        y0, y1 = mask.shape[0]-row1, mask.shape[0]-row0
        lx, ly = (x0+x1)*resolution/2, (y0+y1)*resolution/2
        cx, cy = ox+c*lx-s*ly, oy+s*lx+c*ly
        sx, sy = (x1-x0)*resolution, (y1-y0)*resolution
        values = [float(f'{v:.12f}') for v in (cx, cy, sx, sy, angle)]
        vx, vy, vsx, vsy, va = values
        # Check emitted numerical rectangle corners against map cell boundaries.
        for dx, dy in ((-1,-1), (-1,1), (1,-1), (1,1)):
            emitted = (vx+math.cos(va)*dx*vsx/2-math.sin(va)*dy*vsy/2,
                       vy+math.sin(va)*dx*vsx/2+math.cos(va)*dy*vsy/2)
            exact = (cx+c*dx*sx/2-s*dy*sy/2, cy+s*dx*sx/2+c*dy*sy/2)
            max_error = max(max_error, math.dist(emitted, exact))
        boxes.append(values)
        for kind in ('collision', 'visual'):
            item = ET.SubElement(wall_link, kind, name=f'{kind}_{i}')
            ET.SubElement(item, 'pose').text = f'{vx:.12f} {vy:.12f} .5 0 0 {va:.12f}'
            box = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = f'{vsx:.12f} {vsy:.12f} 1.0'
    assert np.array_equal(rebuilt, mask.astype(np.uint16)), 'missing/overlapping occupied cells'
    ET.ElementTree(model).write(OUT / f'{floor.lower()}{suffix}_building.xml', encoding='unicode')
    if max_error > .05:
        raise SystemExit(f'world-map error {max_error} > .05; user confirmation required')
    ET.indent(sdf)
    ET.ElementTree(sdf).write(OUT / f'{floor.lower()}{suffix}.world', encoding='utf-8', xml_declaration=True)
    (OUT / f'{floor.lower()}{suffix}_boxes.json').write_text(json.dumps(boxes))
    return {'door_open_removed_cells': removed, 'occupied_cells': int(mask.sum()), 'merged_rectangles': len(rects),
            'raster_mismatch_cells': 0, 'max_corner_error_m': max_error,
            'resolution_m': resolution, 'origin': config['origin'],
            'pgm_sha256': hashlib.sha256((yaml_path.parent/config['image']).read_bytes()).hexdigest()}


def doors():
    """Two explicit sliding leaves; existing map/frame cells are never removed."""
    a = math.radians(21.75)
    raw_x, raw_y = 245*math.cos(a)-129*math.sin(a), 245*math.sin(a)+129*math.cos(a)
    specs = {'F1': {'center': [-18.5+raw_x*.05, -11.6+(423-raw_y)*.05],
                    'angle': -a, 'opening_m': 1.0},
             'F2': {'center': [-11.925, -1.925], 'angle': 2.312744-math.pi/2,
                    'opening_m': 1.0}}
    for floor, spec in specs.items():
        width, angle = spec['opening_m'], spec['angle']
        spec['leaves'] = {}
        for side, sign in [('left', -1), ('right', 1)]:
            name = f'{floor.lower()}_door_{side}'
            positions = {}
            for mode, offset in [('closed', sign*width/4), ('open', sign*(3*width/4+.06))]:
                positions[mode] = [spec['center'][0]+math.cos(angle)*offset,
                                   spec['center'][1]+math.sin(angle)*offset, angle]
            spec['leaves'][name] = positions
            root = ET.Element('sdf', version='1.6')
            model = ET.SubElement(root, 'model', name=name)
            ET.SubElement(model, 'static').text = 'true'
            x,y,yaw = positions['open']
            ET.SubElement(model, 'pose').text = f'{x} {y} 0 0 0 {yaw}'
            link = ET.SubElement(model, 'link', name='leaf')
            for kind in ('collision', 'visual'):
                item = ET.SubElement(link, kind, name=kind)
                ET.SubElement(item, 'pose').text = '0 0 .5 0 0 0'
                box = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
                ET.SubElement(box, 'size').text = f'{width/2} .025 1.0'
                if kind == 'visual':
                    material = ET.SubElement(item, 'material')
                    ET.SubElement(material, 'ambient').text = '.15 .45 .7 1'
                    ET.SubElement(material, 'diffuse').text = '.15 .45 .7 1'
            ET.ElementTree(root).write(OUT/f'{name}.sdf', encoding='unicode')
    (OUT/'doors.json').write_text(json.dumps(specs, indent=2)+'\n')


def robot():
    import xacro
    doc = xacro.process_file(str(ROOT / 'src/common_pkg/urdf/delivery_robot.urdf.xacro'))
    tree = ET.fromstring(doc.toxml())
    tree.find("link[@name='base_link']/collision/geometry/box").set('size', '.360 .438 .250')
    # Keep the body and wheel horizontal envelope identical to the navigation footprint.
    for name in ('lead_screw_base', 'lead_screw_carrier', 'laser'):
        node = tree.find(f"link[@name='{name}']")
        if node is not None:
            for collision in node.findall('collision'):
                node.remove(collision)
    for plugin in tree.findall('gazebo/plugin'):
        if plugin.get('name') == 'diff_drive':
            plugin.find('wheel_separation').text = '.4323'
            ros = plugin.find('ros')
            ET.SubElement(ros, 'remapping').text = 'cmd_vel:=/sim/cmd_vel_drive'
            plugin.find('publish_wheel_tf').text = 'false'
    sensor = tree.find("gazebo[@reference='laser']/sensor")
    sensor.find('update_rate').text = '7.6'
    sensor.find('ray/scan/horizontal/samples').text = '1450'
    sensor.find('visualize').text = 'false'
    gazebo = ET.SubElement(tree, 'gazebo')
    p3d = ET.SubElement(gazebo, 'plugin', name='ground_truth', filename='libgazebo_ros_p3d.so')
    ros = ET.SubElement(p3d, 'ros')
    ET.SubElement(ros, 'remapping').text = 'odom:=/sim/ground_truth'
    # Gazebo fixed-joint reduction merges base_link into the root base_footprint.
    for key, value in {'body_name':'base_footprint', 'frame_name':'world', 'update_rate':'20.0',
                       'gaussian_noise':'0.0'}.items():
        ET.SubElement(p3d, key).text = value
    ET.indent(tree)
    # Humble spawn_entity passes a Unicode string to lxml; no encoding declaration.
    ET.ElementTree(tree).write(OUT/'robot.urdf', encoding='utf-8', xml_declaration=False)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = {'validation_scope':'simulation geometry only', 'noglass':noglass()}
    report['F1'] = world('F1', OUT/'f1_manual_clean_v3_noglass.yaml')
    report['F2'] = world('F2', ROOT/'src/slam_pkg/maps/field/f2/f2_nav_unknown_v1.yaml')
    report['F2_manual_door_open'] = world('F2', ROOT/'src/slam_pkg/maps/field/f2/f2_nav_unknown_v1.yaml', '_door_open', True)
    robot()
    doors()
    (OUT/'geometry_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
