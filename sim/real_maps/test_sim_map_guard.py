#!/usr/bin/env python3
"""Generated-map guard must reject changed files, wrong live maps and duplicate servers."""
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from sim_map_guard import check_map, pin_map

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('field_guard_test_helpers', ROOT/'scripts/field_map_guard.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)


class SimMapGuardTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)
        self.image = self.path/'map.pgm'
        self.image.write_bytes(b'P5\n2 1\n255\n\x00\xff')
        self.yaml = self.path/'map.yaml'
        self.yaml.write_text('image: map.pgm\nresolution: 0.05\n')
        self.pin = pin_map(self.yaml)
        self.source = SimpleNamespace(live_map_servers=lambda _: [123], map_server_loaded=helpers.map_server_loaded)
        self.log(self.yaml, self.image)

    def log(self, yaml, image):
        (self.path/'nav2_0.log').write_text(f'Loading yaml file: {yaml}\nRead map {image}: 2 X 1 map\n')

    def check(self, stage='goal'):
        return check_map(self.pin, stage, self.path, self.source)['result']

    def test_exact_map_and_live_server_pass(self):
        self.assertEqual(self.check('pre-nav'), 'PASS')
        self.assertEqual(self.check(), 'PASS')

    def test_changed_image_fails_even_if_live_path_matches(self):
        self.image.write_bytes(b'P5\n2 1\n255\n\xff\xff')
        self.assertEqual(self.check(), 'FAIL')

    def test_changed_yaml_fails(self):
        self.yaml.write_text('image: map.pgm\nresolution: 0.10\n')
        self.assertEqual(self.check('pre-nav'), 'FAIL')

    def test_wrong_loaded_map_fails(self):
        self.log(self.path/'original.yaml', self.path/'original.pgm')
        self.assertEqual(self.check(), 'FAIL')

    def test_duplicate_or_missing_server_fails(self):
        for pids in ([], [123, 456]):
            self.source.live_map_servers = lambda _, pids=pids: pids
            self.assertEqual(self.check(), 'FAIL')

    def test_missing_log_fails(self):
        (self.path/'nav2_0.log').unlink()
        self.assertEqual(self.check(), 'FAIL')


if __name__ == '__main__':
    unittest.main()
