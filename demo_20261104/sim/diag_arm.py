#!/usr/bin/env python3
"""Diagnostic: run the top-row and low-row press cycles in place (open floor) and log commanded vs measured joints.
  python3 demo_20261104/sim/diag_arm.py      (host: re-runs itself in the isolated container like the runner)
"""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.argv += ['--no-record']
os.environ.setdefault('PACKAGU_SIM_CONTAINER', 'packagu_sim_classroom')
import importlib.util                                         # noqa: E402
spec = importlib.util.spec_from_file_location('runner', HERE/'run_classroom_demo.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)                               # trampoline + prepare + imports (does not run main)

import copy, json, time                                       # noqa: E401,E402
import rclpy                                                  # noqa: E402

rclpy.init()
stack = runner.ClassroomStack(f"diag_arm_{time.strftime('%H%M%S')}", world='classroom', floor='F1', params='P0',
                              spawn=tuple(runner.CFG['poses']['start'])+('F1',), arm=True, speed=3.0, parcels=2,
                              load='pallet', record=False)
out = []
LOADED = '--loaded' in sys.argv
stack.start_loaded = LOADED                                   # scenario setup: parcel already on the fork
try:
    stack.start()
    P = copy.deepcopy(runner.WORLD)
    a = runner.ArmLift(stack, P)
    if LOADED:
        print(json.dumps({'carry_up': a.carry_up()}, default=str), flush=True)
    low = runner.add_low_cycles(a.sp, P['arm'], runner.CFG['boards']['low_row_dz_m'], P['arm']['press_cycle'])
    for number in (2, low['destination'], 1, low['call']):
        for pose, ms, send in a.sp.PRESS_CYCLES[number]:
            pwm = a.sp.POSE_TABLES[number][pose]
            t0, w0 = stack.node.now(), time.monotonic()
            r = a.move(pwm, ms) if send else {'reached': None}
            tgt = a.target(pwm)
            err = {k: (None if v is None else round(v, 3)) for k, v in a.errors(tgt).items()}
            row = {'cycle': number, 'pose': pose, 'reached': r.get('reached'), 'err': err,
                   'joints': {k: round(stack.node.joints.get(k, float('nan')), 3) for k in tgt},
                   'target': {k: round(v, 3) for k, v in tgt.items()}, 'tip_map': a.tip_map(),
                   'sim_s': round(stack.node.now()-t0, 2), 'wall_s': round(time.monotonic()-w0, 2)}
            out.append(row)
            print(json.dumps(row), flush=True)
finally:
    (stack.dir/'diag_arm.json').write_text(json.dumps(out, indent=1))
    stack.finish()
    rclpy.try_shutdown()
