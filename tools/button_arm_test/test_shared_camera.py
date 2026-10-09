"""One website/capture for button calibration, floor recognition and arm views."""
import json
import tempfile
import threading
import time
import unittest
import types
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen
from unittest.mock import patch

import cv2
import app
from test_button_arm import args


class Tests(unittest.TestCase):
    def test_ros_home_survives_saving_button_calibration(self):
        with tempfile.TemporaryDirectory() as folder:
            options = args(folder); options.ros_arm = True
            state = app.App(options)
            state.commanded = dict(app.ros_protocol.POSES_1['press'])
            source = dict(state.source, home={'000':1500,'001':1141,'002':2470,'003':1570})
            state.session = types.SimpleNamespace(source=state.source)
            with patch.object(app.arm, 'save_menu_press', return_value=source):
                state.save_menu('6')
            self.assertEqual(state.source['home'], app.ros_protocol.HOME)
            self.assertEqual(state.session.source['home'], app.ros_protocol.HOME)

    def test_same_frame_reaches_both_readers_and_only_one_capture_is_opened(self):
        with tempfile.TemporaryDirectory() as folder:
            state = app.App(args(folder))
            state.args.source = '/dev/test-camera'
            class Camera:
                count = 0
                def isOpened(self): return True
                def read(self):
                    self.count += 1
                    time.sleep(.11)
                    return (True, app.floor_app.pattern('4')) if self.count <= 2 else (False, None)
                def release(self): self.released = True
            camera = Camera()
            with patch.object(app, 'open_camera', return_value=camera) as opened, \
                    patch.object(state.floor, 'run', side_effect=AssertionError('second capture')):
                state.capture()
            opened.assert_called_once_with('/dev/test-camera')
            self.assertEqual(state.frames, 2)
            self.assertEqual(state.floor.frame_seq, 2)
            self.assertTrue(camera.released)
            self.assertEqual(state.floor.status['floor'], 'UNKNOWN')
            self.assertFalse(state.state()['floor']['exit_condition_met'])
            self.assertTrue(state.error)

    def test_shared_http_floor_roi_enrollment_and_view_switch(self):
        with tempfile.TemporaryDirectory() as folder:
            state = app.App(args(folder))
            frame = app.floor_app.pattern('4')
            state.process(frame)
            with ThreadingHTTPServer(('127.0.0.1', 0), app.handler(state)) as server:
                thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
                base = 'http://127.0.0.1:%s' % server.server_port
                def post(path, value):
                    request = Request(base+path, json.dumps(value).encode(), {'Content-Type':'application/json'})
                    with urlopen(request, timeout=3) as response: return json.load(response)
                try:
                    roi = [[0,0],[1,0],[1,1],[0,1]]
                    self.assertTrue(post('/floor/roi', dict(points=roi))['ok'])
                    state.process(frame)
                    self.assertTrue(post('/floor/enroll', dict(label='4'))['ok'])
                    self.assertTrue(post('/floor/target', dict(label='4'))['ok'])
                    self.assertTrue(post('/api/target', dict(label='4', floor_target='F4'))['ok'])
                    self.assertTrue(post('/api/view', dict(view='floor_view'))['ok'])
                    with urlopen(base+'/api/state') as response: result = json.load(response)
                    self.assertEqual(result['view_mode'], 'floor')
                    self.assertEqual(result['commanded'], state.camera_views['floor_view'])
                    with urlopen(base+'/floor/api/state') as response: floor = json.load(response)
                    self.assertEqual(floor['roi'], roi)
                    self.assertEqual(floor['frame_seq'], 2)
                    self.assertEqual(floor['target'], '4')
                    with urlopen(base+'/floor/test') as response:
                        self.assertIn(b'/floor/pattern.png', response.read())
                    with urlopen(base+'/floor/pattern.png?floor=4') as response:
                        self.assertTrue(response.read().startswith(b'\x89PNG'))
                    self.assertTrue(post('/api/display', dict(mode='buttons'))['ok'])
                    self.assertEqual(state.view_mode, 'buttons')
                    self.assertIsNone(state.session)  # Preview must never open serial.
                finally:
                    server.shutdown(); thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
