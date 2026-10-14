#!/usr/bin/env python3
"""Offline checks for the bounded simulation-only F2 cleanup."""
import unittest
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from f2_cleanup import flatten

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

class CleanupTests(unittest.TestCase):
    def test_outside_region_and_source_unchanged(self):
        spec = yaml.safe_load((HERE/'world_params.yaml').read_text())['f2_cleanup']
        source = ROOT/'src/slam_pkg/maps/field/f2/f2_nav_unknown_v1.yaml'
        cfg = yaml.safe_load(source.read_text())
        image = source.parent/cfg['image']
        original = image.read_bytes()
        pixels = np.asarray(Image.open(image))
        before = pixels.copy()
        cleaned, outside, boxes, stats = flatten(pixels,cfg,spec)
        self.assertTrue(np.array_equal(pixels,before))
        self.assertEqual(image.read_bytes(),original)
        self.assertEqual(stats['outside_region_changed_cells'],0)
        self.assertGreater(stats['removed_occupied_cells'],0)
        self.assertEqual(len(boxes),5)
        self.assertLess(stats['max_raster_boundary_error_m'],.05)
        self.assertEqual(cleaned.shape,pixels.shape)
        self.assertFalse(np.any(outside & (cleaned != pixels)))

if __name__ == '__main__':
    unittest.main()
