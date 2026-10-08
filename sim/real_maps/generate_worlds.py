#!/usr/bin/env python3
"""Exact raster-to-box Gazebo Classic worlds. Generated assets stay local.

Every dimension comes from world_params.yaml (with its source). Outputs:
  f1.world / f2.world / f2_door_open.world   single-floor worlds (9/15 layout)
  building.world                              F1 + F2 in one world, F2 shifted by
                                              its world_offset, elevator door leaves
  *_boxes.json                                wall boxes in each floor's MAP frame
  robot.urdf / robot_nonoise.urdf             sim robot, encoder odom (LiDAR noise on / off)
  robot_world.urdf                            WORLD odom (truth as odom) — 9/15 reproduction only
  robot_arm.urdf                              robot.urdf + simplified 4-DOF arm (--arm-sim, [가정값])
  doors.json, geometry_validation.json
"""
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
HERE = Path(__file__).resolve().parent
OUT = HERE / 'generated'
PARAMS = yaml.safe_load((HERE / 'world_params.yaml').read_text())
MAX_ERROR_M = .05


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
    data = yaml.safe_load((ROOT / PARAMS['floors']['F1']['nav_map']).read_text())
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


def building(floor, yaml_path, suffix='', door_clear=False):
    """Map-frame wall boxes from occupied cells, with exact raster/corner checks."""
    config = yaml.safe_load(yaml_path.read_text())
    pixels = np.asarray(Image.open(yaml_path.parent / config['image']))
    probability = pixels.astype(float) / 255 if config['negate'] else (255-pixels.astype(float))/255
    mask = probability > config['occupied_thresh']
    removed = 0
    if door_clear:
        # User-authorized open doorway only; A/B worlds retain every raw point.
        spec = PARAMS['f2_door_open']
        ys, xs = np.mgrid[0:mask.shape[0], 0:mask.shape[1]]
        wx = config['origin'][0]+(xs+.5)*config['resolution']
        wy = config['origin'][1]+(mask.shape[0]-ys-.5)*config['resolution']
        da = spec['axis_rad']
        dx, dy = wx-spec['center'][0], wy-spec['center'][1]
        along = math.cos(da)*dx+math.sin(da)*dy
        across = -math.sin(da)*dx+math.cos(da)*dy
        opening = (np.abs(along) < spec['half_along_m'])&(np.abs(across) < spec['half_across_m'])
        removed = int((mask&opening).sum())
        mask = mask & ~opening
    rects = rectangles(mask)
    rebuilt = np.zeros_like(mask, dtype=np.uint16)
    resolution = config['resolution']
    ox, oy, angle = config['origin']
    c, s = math.cos(angle), math.sin(angle)
    boxes, max_error = [], 0.0
    for x0, x1, row0, row1 in rects:
        rebuilt[row0:row1, x0:x1] += 1
        y0, y1 = mask.shape[0]-row1, mask.shape[0]-row0
        lx, ly = (x0+x1)*resolution/2, (y0+y1)*resolution/2
        cx, cy = ox+c*lx-s*ly, oy+s*lx+c*ly
        sx, sy = (x1-x0)*resolution, (y1-y0)*resolution
        values = [float(f'{v:.12f}') for v in (cx, cy, sx, sy, angle)]
        vx, vy, vsx, vsy, va = values
        # Check emitted numerical rectangle corners against map cell boundaries.
        for dx, dy in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            emitted = (vx+math.cos(va)*dx*vsx/2-math.sin(va)*dy*vsy/2,
                       vy+math.sin(va)*dx*vsx/2+math.cos(va)*dy*vsy/2)
            exact = (cx+c*dx*sx/2-s*dy*sy/2, cy+s*dx*sx/2+c*dy*sy/2)
            max_error = max(max_error, math.dist(emitted, exact))
        boxes.append(values)
    assert np.array_equal(rebuilt, mask.astype(np.uint16)), 'missing/overlapping occupied cells'
    if max_error > MAX_ERROR_M:
        raise SystemExit(f'world-map error {max_error} > {MAX_ERROR_M}; user confirmation required')
    (OUT / f'{floor.lower()}{suffix}_boxes.json').write_text(json.dumps(boxes))
    stats = {'door_open_removed_cells': removed, 'occupied_cells': int(mask.sum()),
             'merged_rectangles': len(rects), 'raster_mismatch_cells': 0, 'max_corner_error_m': max_error,
             'resolution_m': resolution, 'origin': config['origin'],
             'pgm_sha256': hashlib.sha256((yaml_path.parent/config['image']).read_bytes()).hexdigest()}
    return boxes, stats


def wall_model(name, boxes, offset):
    """Static model with one collision/visual per merged box, shifted by the floor offset."""
    height = float(PARAMS['walls']['height_m'])
    model = ET.Element('model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='walls')
    for i, (x, y, sx, sy, yaw) in enumerate(boxes):
        for kind in ('collision', 'visual'):
            item = ET.SubElement(link, kind, name=f'{kind}_{i}')
            ET.SubElement(item, 'pose').text = (f'{x+offset[0]:.12f} {y+offset[1]:.12f} '
                                                f'{offset[2]+height/2:.6f} 0 0 {yaw:.12f}')
            box = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = f'{sx:.12f} {sy:.12f} {height}'
    return model


def offset_error(model, boxes, offset):
    """Largest distance between an emitted (offset) box centre and map box + offset."""
    worst = 0.0
    poses = [c.find('pose').text.split() for c in model.iter('collision')]
    for (x, y, *_), pose in zip(boxes, poses):
        worst = max(worst, math.dist((float(pose[0]), float(pose[1])), (x+offset[0], y+offset[1])))
    return worst


def write_world(path, models, ground_size):
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
        ET.SubElement(plane, 'size').text = f'{ground_size} {ground_size}'
    light = ET.SubElement(w, 'light', name='sun', type='directional')
    ET.SubElement(light, 'direction').text = '-.5 .1 -1'
    ET.SubElement(light, 'diffuse').text = '.8 .8 .8 1'
    for model in models:
        w.append(model)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(path, encoding='utf-8', xml_declaration=True)


def door_specs():
    """Two sliding leaves per floor; map-frame closed/open poses. Map cells are never removed."""
    leaf = PARAMS['elevator']['leaf']
    specs = {}
    for floor in ('F1', 'F2'):
        p = PARAMS['elevator'][floor]
        width, angle = float(p['opening_width_m']), float(p['door_axis_rad'])
        spec = {'center': p['door_center'], 'angle': angle, 'opening_m': width,
                'leaf_size_m': [width/2, leaf['thickness_m'], leaf['height_m']],
                'offset': PARAMS['floors'][floor]['world_offset'], 'leaves': {}}
        for side, sign in [('left', -1), ('right', 1)]:
            positions = {}
            for mode, along in [('closed', sign*width/4), ('open', sign*(3*width/4+leaf['open_overtravel_m']))]:
                positions[mode] = [spec['center'][0]+math.cos(angle)*along,
                                   spec['center'][1]+math.sin(angle)*along, angle]
            spec['leaves'][f'{floor.lower()}_door_{side}'] = positions
        specs[floor] = spec
    return specs


def door_model(name, spec, mode):
    x, y, yaw = spec['leaves'][name][mode]
    off = spec['offset']
    sx, sy, sz = spec['leaf_size_m']
    model = ET.Element('model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x+off[0]} {y+off[1]} {off[2]} 0 0 {yaw}'
    link = ET.SubElement(model, 'link', name='leaf')
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, name=kind)
        ET.SubElement(item, 'pose').text = f'0 0 {sz/2} 0 0 0'
        box = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
        ET.SubElement(box, 'size').text = f'{sx} {sy} {sz}'
        if kind == 'visual':
            material = ET.SubElement(item, 'material')
            ET.SubElement(material, 'ambient').text = '.15 .45 .7 1'
            ET.SubElement(material, 'diffuse').text = '.15 .45 .7 1'
    return model


def doors():
    specs = door_specs()
    for floor, spec in specs.items():
        for name in spec['leaves']:
            # Standalone leaf SDF (single-floor rehearsal, no offset), 9/15 compatible.
            single = dict(spec, offset=[0.0, 0.0, 0.0])
            root = ET.Element('sdf', version='1.6')
            root.append(door_model(name, single, 'open'))
            ET.ElementTree(root).write(OUT/f'{name}.sdf', encoding='unicode')
    (OUT/'doors.json').write_text(json.dumps(specs, indent=2)+'\n')
    return specs


def rolling_rear_caster(tree):
    """Replace the fixed rear ball-caster sphere by a ball that rolls (x then y revolute joints).

    A fixed sphere slides with friction and drags the drive wheels into slip; a real ball caster rolls.
    """
    joint = tree.find("joint[@name='caster_joint']")
    xyz = joint.find('origin').get('xyz')
    tree.remove(joint)
    roll = ET.SubElement(tree, 'link', name='caster_roll_x')
    inertial = ET.SubElement(roll, 'inertial')
    ET.SubElement(inertial, 'mass', value='0.01')
    ET.SubElement(inertial, 'inertia', ixx='1e-6', iyy='1e-6', izz='1e-6', ixy='0', ixz='0', iyz='0')
    for name, parent, child, origin, axis in (('caster_joint_x', 'base_link', 'caster_roll_x', xyz, '1 0 0'),
                                             ('caster_joint_y', 'caster_roll_x', 'caster_wheel', '0 0 0', '0 1 0')):
        j = ET.SubElement(tree, 'joint', name=name, type='continuous')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=origin, rpy='0 0 0')
        ET.SubElement(j, 'axis', xyz=axis)
        ET.SubElement(j, 'dynamics', damping='0.0005', friction='0.0')


def add_arm(tree):
    """Simplified 4-DOF servo arm (sim-only option). All geometry/axes are [가정값] in world_params arm."""
    arm = PARAMS['arm']

    def inertial(link, mass=.1):
        node = ET.SubElement(link, 'inertial')
        ET.SubElement(node, 'mass', value=str(mass))
        ET.SubElement(node, 'inertia', ixx='1e-4', iyy='1e-4', izz='1e-4', ixy='0', ixz='0', iyz='0')

    def fixed(name, parent, child, xyz):
        j = ET.SubElement(tree, 'joint', name=name, type='fixed')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=' '.join(map(str, xyz)), rpy='0 0 0')

    mount = ET.SubElement(tree, 'link', name='arm_mount')
    inertial(mount, .05)
    fixed('arm_mount_joint', 'base_link', 'arm_mount', arm['mount_xyz'])
    parent = 'arm_mount'
    joints = arm['joints']
    for i, spec in enumerate(joints):
        link = ET.SubElement(tree, 'link', name=f'arm_link_{spec["servo"]}')
        inertial(link)
        length = (joints[i+1]['parent_offset'][0] if i+1 < len(joints) else arm['tip_offset'][0]) or .05
        visual = ET.SubElement(link, 'visual')
        ET.SubElement(visual, 'origin', xyz=f'{length/2} 0 0', rpy='0 1.5708 0')
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder', radius='0.02', length=str(length))
        j = ET.SubElement(tree, 'joint', name=spec['name'], type='revolute')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=link.get('name'))
        ET.SubElement(j, 'origin', xyz=' '.join(map(str, spec['parent_offset'])), rpy='0 0 0')
        ET.SubElement(j, 'axis', xyz=' '.join(map(str, spec['axis'])))
        ET.SubElement(j, 'limit', lower='-1.6', upper='1.6', effort='10', velocity='3')
        ET.SubElement(j, 'dynamics', damping='0.1', friction=str(arm['hold_friction_nm']))
        g = ET.SubElement(tree, 'gazebo', reference=link.get('name'))
        ET.SubElement(g, 'gravity').text = 'false'
        parent = link.get('name')
    tip = ET.SubElement(tree, 'link', name='arm_tip')
    inertial(tip, .01)
    fixed('arm_tip_joint', parent, 'arm_tip', arm['tip_offset'])
    gazebo = ET.SubElement(tree, 'gazebo')
    states = ET.SubElement(gazebo, 'plugin', name='arm_joint_state_publisher',
                           filename='libgazebo_ros_joint_state_publisher.so')
    ET.SubElement(ET.SubElement(states, 'ros'), 'namespace').text = '/'
    ET.SubElement(states, 'update_rate').text = '30'
    for spec in joints:
        ET.SubElement(states, 'joint_name').text = spec['name']
    gazebo = ET.SubElement(tree, 'gazebo')
    traj = ET.SubElement(gazebo, 'plugin', name='arm_trajectory', filename='libgazebo_ros_joint_pose_trajectory.so')
    ros = ET.SubElement(traj, 'ros')
    ET.SubElement(ros, 'namespace').text = '/'
    ET.SubElement(ros, 'remapping').text = 'set_joint_trajectory:=set_arm_trajectory'
    ET.SubElement(traj, 'update_rate').text = '20'


def robot(noise, filename, odometry=None, arm=False):
    import xacro
    lidar, body = PARAMS['lidar'], PARAMS['robot']
    doc = xacro.process_file(str(ROOT / 'src/common_pkg/urdf/delivery_robot.urdf.xacro'), mappings={
        'laser_x': str(lidar['pose_xyz'][0]), 'laser_y': str(lidar['pose_xyz'][1]),
        'laser_z': str(lidar['pose_xyz'][2])})
    tree = ET.fromstring(doc.toxml())
    tree.find("link[@name='base_link']/collision/geometry/box").set(
        'size', ' '.join(f'{v:.3f}' for v in body['collision_box_m']))
    # Keep the body and wheel horizontal envelope identical to the navigation footprint.
    for name in ('lead_screw_base', 'lead_screw_carrier', 'laser'):
        node = tree.find(f"link[@name='{name}']")
        if node is not None:
            for collision in node.findall('collision'):
                node.remove(collision)
    for plugin in tree.findall('gazebo/plugin'):
        if plugin.get('name') == 'diff_drive':
            plugin.find('wheel_separation').text = str(body['wheel_separation_m'])
            plugin.find('wheel_diameter').text = str(2*body['wheel_radius_m'])
            ros = plugin.find('ros')
            ET.SubElement(ros, 'remapping').text = 'cmd_vel:=/sim/cmd_vel_drive'
            plugin.find('publish_wheel_tf').text = 'false'
            profile = odometry or body['odometry_source']
            ET.SubElement(plugin, 'odometry_source').text = str(body['odometry_profiles'][profile])
    if (odometry or body['odometry_source']) == 'encoder':
        # Caster contact simplification for encoder odometry (see world_params robot.contact).
        contact = body['contact']
        origin = tree.find("joint[@name='caster_front_joint']/origin")
        xyz = [float(v) for v in origin.get('xyz').split()]
        xyz[2] += float(contact['front_aux_caster_lift_m'])
        origin.set('xyz', ' '.join(f'{v:.4f}' for v in xyz))
        for ref, key in (('caster_wheel', 'rear_caster_mu'), ('caster_wheel_front', 'front_caster_mu')):
            g = tree.find(f"gazebo[@reference='{ref}']")
            g.find('mu1').text = g.find('mu2').text = str(contact[key])
        if contact['rear_caster'] == 'rolling_ball':
            rolling_rear_caster(tree)
    sensor = tree.find("gazebo[@reference='laser']/sensor")
    sensor.find('update_rate').text = str(lidar['update_rate_hz'])
    sensor.find('ray/scan/horizontal/samples').text = str(lidar['samples'])
    sensor.find('ray/range/min').text = str(lidar['range_min_m'])
    sensor.find('ray/range/max').text = str(lidar['range_max_m'])
    sensor.find('visualize').text = 'false'
    noise_node = sensor.find('ray/noise')
    if noise:
        noise_node.find('stddev').text = str(lidar['noise_stddev_m'])
    else:
        sensor.find('ray').remove(noise_node)
    gazebo = ET.SubElement(tree, 'gazebo')
    p3d = ET.SubElement(gazebo, 'plugin', name='ground_truth', filename='libgazebo_ros_p3d.so')
    ros = ET.SubElement(p3d, 'ros')
    ET.SubElement(ros, 'remapping').text = 'odom:=/sim/ground_truth'
    # Gazebo fixed-joint reduction merges base_link into the root base_footprint.
    for key, value in {'body_name': 'base_footprint', 'frame_name': 'world',
                       'update_rate': str(body['ground_truth_rate_hz']), 'gaussian_noise': '0.0'}.items():
        ET.SubElement(p3d, key).text = value
    if arm:
        add_arm(tree)
    ET.indent(tree)
    # Humble spawn_entity passes a Unicode string to lxml; no encoding declaration.
    ET.ElementTree(tree).write(OUT/filename, encoding='utf-8', xml_declaration=False)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    floors = PARAMS['floors']
    report = {'validation_scope': 'simulation geometry only', 'params_sha256':
              hashlib.sha256((HERE/'world_params.yaml').read_bytes()).hexdigest(), 'noglass': noglass()}
    f1_boxes, report['F1'] = building('F1', OUT/'f1_manual_clean_v3_noglass.yaml')
    f2_yaml = ROOT/floors['F2']['nav_map']
    f2_boxes, report['F2'] = building('F2', f2_yaml)
    f2_open_boxes, report['F2_door_open'] = building('F2', f2_yaml, '_door_open', True)
    zero = [0.0, 0.0, 0.0]
    for name, boxes in [('f1', f1_boxes), ('f2', f2_boxes), ('f2_door_open', f2_open_boxes)]:
        floor = name[:2].upper()
        write_world(OUT/f'{name}.world', [wall_model(f'{floor.lower()}_real_building', boxes, zero)],
                    floors['ground_plane_m'])
    specs = doors()
    # Combined two-floor world: F2 uses the door-open walls so only the leaves close the doorway.
    models, offsets = [], {}
    for floor, boxes in [('F1', f1_boxes), ('F2', f2_open_boxes)]:
        offset = floors[floor]['world_offset']
        model = wall_model(f'{floor.lower()}_real_building', boxes, offset)
        offsets[floor] = {'offset': offset, 'max_offset_error_m': offset_error(model, boxes, offset)}
        if offsets[floor]['max_offset_error_m'] > MAX_ERROR_M:
            raise SystemExit(f'{floor} offset error > {MAX_ERROR_M}; user confirmation required')
        models.append(model)
        for name in specs[floor]['leaves']:
            models.append(door_model(name, specs[floor], 'closed'))
    write_world(OUT/'building.world', models, floors['ground_plane_m'])
    report['building'] = offsets
    robot(PARAMS['lidar']['noise_enabled'], 'robot.urdf')
    robot(False, 'robot_nonoise.urdf')
    robot(PARAMS['lidar']['noise_enabled'], 'robot_world.urdf', 'world')
    robot(PARAMS['lidar']['noise_enabled'], 'robot_arm.urdf', arm=True)   # --arm-sim option only
    (OUT/'geometry_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
