"""Bounded, user-authorized simulation map cleanup; field rasters are never written.

Wall planes follow the existing doorway and median raw cabin wall positions.
Straight Gazebo boxes replace the noisy band; Nav2 gets their rasterization.
"""
import math
import numpy as np


def flatten(pixels, config, spec):
    if abs(config['origin'][2]) > 1e-9:
        raise ValueError('F2 cleanup requires the current axis-aligned raster')
    r = float(config['resolution'])
    ys, xs = np.mgrid[:pixels.shape[0], :pixels.shape[1]]
    x = config['origin'][0]+(xs+.5)*r
    y = config['origin'][1]+(pixels.shape[0]-ys-.5)*r
    cx, cy = spec['center']
    a = float(spec['axis_rad'])
    c, s = math.cos(a), math.sin(a)
    t, n = c*(x-cx)+s*(y-cy), -s*(x-cx)+c*(y-cy)
    t0, t1 = spec['along_m']
    n0, n1 = spec['normal_m']
    region = (t >= t0)&(t <= t1)&(n >= n0)&(n <= n1)
    left, right, back = spec['cabin_planes_m']
    half = spec['opening_m']/2
    thick = float(spec['wall_thickness_m'])
    # Local (t,n) boxes: two landing walls, two cabin sides and the back.
    local = [((t0-half)/2, 0, -half-t0, thick, 0),
             ((half+t1)/2, 0, t1-half, thick, 0),
             (left, back/2, thick, back, 0),
             (right, back/2, thick, back, 0),
             ((left+right)/2, back, right-left+thick, thick, 0)]
    wall = np.zeros(pixels.shape, bool)
    boxes = []
    for bt, bn, wt, wn, _ in local:
        wall |= (abs(t-bt) <= wt/2)&(abs(n-bn) <= wn/2)
        boxes.append([cx+c*bt-s*bn, cy+s*bt+c*bn, wt, wn, a])
    cleaned = pixels.copy()
    free = (n <= 0)|((t > left)&(t < right)&(n < back))
    cleaned[region] = 205
    cleaned[region&free] = 254
    cleaned[region&wall] = 0
    occupied = (255-pixels.astype(float))/255 > config['occupied_thresh']
    outside = occupied&~region
    stats = {'changed_cells': int((cleaned != pixels).sum()),
             'removed_occupied_cells': int((occupied&region&~wall).sum()),
             'added_occupied_cells': int((~occupied&region&wall).sum()),
             'outside_region_changed_cells': int(((cleaned != pixels)&~region).sum()),
             'analytic_wall_boxes': len(boxes),
             'max_raster_boundary_error_m': r/math.sqrt(2),
             'region': spec}
    return cleaned, outside, boxes, stats
