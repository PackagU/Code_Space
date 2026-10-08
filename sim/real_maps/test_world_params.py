#!/usr/bin/env python3
"""world_params.yaml contract: every dimension carries a source, floors cannot see each other,
transfer/relocalization criteria exist, and generated geometry stays within 0.05 m (if generated)."""
import json
import math
import unittest
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
PARAMS = yaml.safe_load((HERE/'world_params.yaml').read_text())


class WorldParamsTests(unittest.TestCase):
    def test_every_section_has_source(self):
        def walk(node, path):
            if isinstance(node, dict):
                numeric = [k for k, v in node.items() if isinstance(v, (int, float, list)) and not isinstance(v, bool)]
                if numeric and path not in ('', 'floors', 'robot.odometry_profiles'):
                    self.assertTrue(any('source' in k for k in node), f'{path} has values without a source')
                for k, v in node.items():
                    walk(v, f'{path}.{k}' if path else k)
        walk(PARAMS, '')

    def test_floors_out_of_lidar_range(self):
        f1 = PARAMS['floors']['F1']['world_offset']
        f2 = PARAMS['floors']['F2']['world_offset']
        # F1 x <= 7.1, F2 x >= -44.9 in their own map frames.
        gap = (-44.9+f2[0]-f1[0])-7.1
        self.assertGreater(gap, 2*PARAMS['lidar']['range_max_m'])

    def test_unknown_dimensions_are_marked(self):
        for floor in ('F1', 'F2'):
            self.assertIn('아직 모름', PARAMS['elevator'][floor]['opening_width_source'])
        self.assertIn('아직 모름', PARAMS['elevator']['timing']['source'])

    def test_encoder_default_and_world_profile(self):
        robot = PARAMS['robot']
        self.assertEqual(robot['odometry_source'], 'encoder')
        self.assertEqual(robot['odometry_profiles'], {'encoder': 0, 'world': 1})

    def test_relocalization_gate_criteria(self):
        reloc = PARAMS['relocalization']
        for key in ('min_new_amcl_samples', 'position_error_max_m', 'yaw_error_max_rad', 'covariance_xy_max_m2',
                    'covariance_yaw_max_rad2', 'tf_stable_window_s', 'timeout_s'):
            self.assertGreater(reloc[key], 0)
        self.assertIn('[제안값]', reloc['source'])

    def test_generated_geometry_within_tolerance(self):
        report = HERE/'generated/geometry_validation.json'
        if not report.exists():
            self.skipTest('generate_worlds.py has not run')
        data = json.loads(report.read_text())
        for key in ('F1', 'F2', 'F2_door_open'):
            self.assertEqual(data[key]['raster_mismatch_cells'], 0)
            self.assertLess(data[key]['max_corner_error_m'], .05)
        for floor in ('F1', 'F2'):
            self.assertLess(data['building'][floor]['max_offset_error_m'], .05)
        self.assertEqual(data['noglass']['changed_cells'], 44)
        self.assertEqual(data['F2_door_open']['door_open_removed_cells'], 78)

    def test_cabin_waypoints_exist(self):
        wps = json.loads((HERE.parents[1]/'src/slam_pkg/maps/field/waypoints.json').read_text())['waypoints']
        for floor in ('F1', 'F2'):
            self.assertIn(PARAMS['elevator'][floor]['inside_waypoint'], wps)
        registry = yaml.safe_load((HERE/'floor_maps_real.yaml').read_text())
        for floor, name in (('F1', 'f1_elevator_inside'), ('F2', 'f2_elevator_inside')):
            p = registry['floors'][floor]['points']['elevator_inside']
            self.assertAlmostEqual(p['x'], wps[name]['x'])
            self.assertAlmostEqual(math.radians(p['yaw_deg']), wps[name]['yaw_rad'], places=4)


if __name__ == '__main__':
    unittest.main()
