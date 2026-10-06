import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
import app
from app import Gate, Reader, pattern, normalize, classify

class Tests(unittest.TestCase):
    def test_live_frame_identity_and_camera_loss(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(app, 'DATA', Path(folder)):
            reader, restarted = Reader(0), Reader(0)
            self.assertNotEqual(reader.stream_id, restarted.stream_id)
            reader.roi = [[0,0],[1,0],[1,1],[0,1]]
            samples = []
            class GateRecorder:
                def update(self, *args):
                    samples.append(dict(reader.status))
                    return False
                def reset(self):
                    pass
            class Capture:
                def __init__(self):
                    self.count = 0
                    self.released = False
                def isOpened(self):
                    return True
                def read(self):
                    self.count += 1
                    return (self.count <= 2, np.zeros((100,160,3), np.uint8))
                def release(self):
                    self.released = True
            reader.gate = GateRecorder()
            capture = Capture()
            with patch.object(app.cv2, 'VideoCapture', return_value=capture), \
                    patch.object(app.time, 'monotonic', side_effect=[1.,1.2]), \
                    patch.object(app.time, 'time', side_effect=[1001.,1001.2]):
                reader.run()
            self.assertEqual([s['frame_seq'] for s in samples], [1,2])
            self.assertTrue(all(s['stream_id'] == reader.stream_id for s in samples))
            self.assertTrue(capture.released)
            self.assertTrue(reader.status['error'])
            self.assertEqual(reader.status['floor'], 'UNKNOWN')

    def test_perspective_crop_camera_frame(self):
        reader = Reader.__new__(Reader)
        reader.roi = [[.1,.2],[.8,.15],[.85,.85],[.12,.9]]
        frame = np.zeros((720,1280,3),np.uint8)
        frame[200:500,300:600] = 255
        crop = reader.crop(frame)
        self.assertEqual(crop.ndim,3)
        self.assertGreater(crop.shape[0],16)
        self.assertIsNotNone(normalize(crop))

    def test_digits_and_basement(self):
        refs = {s:[normalize(pattern(s))] for s in ['B1','B2','1','2','3','4','7','8','10','11','20']}
        for s in refs:
            self.assertEqual(classify(normalize(pattern(s)),refs)[0],s)
        self.assertEqual(classify(normalize(np.zeros((100,100,3),np.uint8)),refs)[0],'UNKNOWN')

    def test_no_early_duplicate_or_stale_event(self):
        g = Gate()
        for i in range(9):
            self.assertFalse(g.update('4','4',i*.1))
        self.assertTrue(g.update('4','4',.9))
        self.assertFalse(g.update('4','4',1.0))
        g.reset()
        for i in range(9):
            g.update('4','4',i*.1)
        self.assertFalse(g.update('4','4',10))
        g.reset()
        for i in range(9):
            g.update('4','4',i*.1)
        self.assertFalse(g.update('UNKNOWN','4',.9))

if __name__ == '__main__':
    unittest.main()
