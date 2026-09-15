#!/usr/bin/env python3
"""One-command offline tests + optional camera-only probe. Never opens serial."""
import argparse
import json
import platform
import resource
import subprocess
import sys
import time
import tempfile
from pathlib import Path

import cv2
import numpy as np
import app
import synth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', help='명시하면 실제 카메라 프레임만 시험')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    result = subprocess.run([sys.executable, '-B', '-m', 'unittest', '-v', 'test_button_arm'], cwd=str(root))
    if result.returncode:
        return result.returncode
    output = {'platform': platform.platform(), 'python': platform.python_version(),
              'opencv': cv2.__version__, 'numpy': np.__version__, 'serial_opened': False,
              'offline_tests': 'PASS', 'physical_contact': 'NOT RUN'}
    with tempfile.TemporaryDirectory() as data:
        options = argparse.Namespace(data=data, source='synthetic', execute=False,
                                     servo_source=str(root/'servo_test.py'), port='/dev/arm_servo')
        state = app.App(options)
        for seed in (1000, 1001, 1002):
            state.process(synth.scene(app.DIGITS, seed)[0])
            state.enroll('1234')
        frame = synth.scene(app.DIGITS, 7)[0]
        cpu_start = time.process_time()
        durations = []
        for _ in range(100):
            start = time.monotonic()
            state.process(frame)
            durations.append((time.monotonic()-start)*1000)
        output['synthetic_benchmark'] = {'frames': 100, 'buttons_per_frame': 4,
                                         'mean_ms': round(float(np.mean(durations)), 3),
                                         'p95_ms': round(float(np.percentile(durations, 95)), 3),
                                         'cpu_seconds': round(time.process_time()-cpu_start, 3),
                                         'peak_rss_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss}
    if args.camera:
        cap = app.open_camera(args.camera)
        try:
            if not cap.isOpened():
                raise RuntimeError('camera open failed')
            begin = time.monotonic()
            frames = 0
            while frames < 30:
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('camera read failed at frame '+str(frames))
                frames += 1
            gray = cv2.cvtColor(cv2.resize(frame, (640, 480)), cv2.COLOR_BGR2GRAY)
            output['camera_probe'] = {'source': args.camera, 'frames': frames,
                                      'shape': list(frame.shape),
                                      'elapsed_seconds': round(time.monotonic()-begin, 3),
                                      'buttons_in_last_frame': len(app.vision.find_buttons(gray))}
        finally:
            cap.release()
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
