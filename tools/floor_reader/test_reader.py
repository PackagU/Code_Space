import unittest
import numpy as np
from app import Gate, Reader, pattern, normalize, classify

class Tests(unittest.TestCase):
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
