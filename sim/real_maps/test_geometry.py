#!/usr/bin/env python3
"""Behaviour checks for exact raster coverage and asymmetric footprint distances."""
import math
import json
from pathlib import Path
import unittest
import numpy as np
from generate_worlds import rectangles
from clearance import minimum_clearance


class GeometryTests(unittest.TestCase):
    def test_runs_cover_without_overlap(self):
        rng=np.random.default_rng(715)
        for shape in ((1,1),(1,9),(9,1),(13,17)):
            for _ in range(10):
                mask=rng.random(shape)>.65
                rebuilt=np.zeros(shape,dtype=int)
                for x0,x1,y0,y1 in rectangles(mask):
                    rebuilt[y0:y1,x0:x1]+=1
                np.testing.assert_array_equal(rebuilt,mask.astype(int))

    def test_vertical_merging(self):
        self.assertEqual(rectangles(np.ones((80,30),bool)),[(0,30,0,80)])

    def test_asymmetric_body_front_back(self):
        front=[[1,0,.1,2,0]]
        back=[[-1,0,.1,2,0]]
        self.assertAlmostEqual(minimum_clearance(front,0,0,0),.917)
        self.assertAlmostEqual(minimum_clearance(back,0,0,0),.623)
        self.assertAlmostEqual(minimum_clearance(front,0,0,math.pi),.623)

    def test_overlap_and_corner(self):
        self.assertEqual(minimum_clearance([[0,0,.1,.1,0]],0,0,0),0)
        self.assertAlmostEqual(minimum_clearance([[1,1,.1,.1,0]],0,0,0),math.hypot(.917,.731))

    def test_rotated_wall(self):
        self.assertAlmostEqual(minimum_clearance([[0,1,2,.1,math.pi/2]],0,0,math.pi/2),0)


class DoorTests(unittest.TestCase):
    def setUp(self):
        self.generated = Path(__file__).parent/'generated'
        if not (self.generated/'doors.json').exists():
            self.skipTest('generate_worlds.py has not run')

    def test_both_leaves_leave_existing_opening(self):
        specs = json.loads((self.generated/'doors.json').read_text())
        for floor, spec in specs.items():
            axis = np.array([math.cos(spec['angle']), math.sin(spec['angle'])])
            for positions in spec['leaves'].values():
                open_offset = abs(np.dot(np.array(positions['open'][:2])-spec['center'], axis))
                closed_offset = abs(np.dot(np.array(positions['closed'][:2])-spec['center'], axis))
                self.assertGreater(open_offset-spec['opening_m']/4, spec['opening_m']/2)
                self.assertLess(closed_offset, spec['opening_m']/2)

    def test_manual_F2_opening_clears_full_body(self):
        boxes = np.array(json.loads((self.generated/'f2_door_open_boxes.json').read_text()))
        a,b = np.array([-11.375,-2.525]), np.array([-12.475,-1.325])
        for fraction in np.linspace(0,1,41):
            x,y = a*(1-fraction)+b*fraction
            self.assertGreater(minimum_clearance(boxes,x,y,2.312744), .15)


if __name__=='__main__':
    unittest.main()
