#!/usr/bin/env python3
"""Wait locally for the observed mission, then validate a fresh complete chain.

The observed robot and its target are untouched. The new chain runs in a separate
container/domain. Refresh evidence when finished results arrive, without polling
the coding model. No publishing, field deployment or parameter selection occurs.
"""
import argparse
import json
import os
import re
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT/'logs/real_map_sim'


def export():
    for script in ('analyze_localization.py', 'summarize.py', 'write_report.py'):
        command = ['python3', str(ROOT/'sim/real_maps'/script)]
        if script == 'analyze_localization.py':
            command = ['bash', '-c', 'source /opt/ros/humble/setup.bash && exec python3 "$1"',
                       'sim_localization', command[1]]
        subprocess.run(command, cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--after', required=True, help='observed M* result to wait for')
    parser.add_argument('--name', default='M1_full_validation_r1')
    parser.add_argument('--domain', type=int, default=217)
    args = parser.parse_args()
    if not all(re.fullmatch(r'M[A-Za-z0-9_-]+', name) for name in (args.after, args.name)):
        parser.error('mission names required')
    if not 1 <= args.domain <= 232:
        parser.error('ROS domain outside supported range')
    awaited = LOG/args.after/'result.json'
    print('WAITING_FOR_TERMINAL_RESULT', awaited, flush=True)
    while not awaited.exists():
        time.sleep(5)
    print('OBSERVED_MISSION_TERMINAL', json.loads(awaited.read_text()).get('completed'), flush=True)
    env = dict(os.environ, SIM_SKIP_PREPARE='1', SIM_MODE='manual', SIM_GUI='0',
               PACKAGU_SIM_CONTAINER='packagu_real_map_full_validation',
               PACKAGU_SIM_DOMAIN=str(args.domain))
    with open(LOG/'full_validation.log', 'w') as output:
        result = subprocess.run(['bash', str(ROOT/'sim/real_maps/run_suite.sh'), '--name', args.name],
                                cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT)
    print('FRESH_CHAIN_EXIT', result.returncode, flush=True)
    export()
    previous = None
    while True:
        completed = sorted(path for path in LOG.glob('S*/result.json')
                           if 'truth_samples' in json.loads(path.read_text()))
        signature = tuple((str(path), path.stat().st_mtime_ns) for path in completed)
        if signature != previous:
            export()
            previous = signature
            print('FINISHED_REGULAR_CONDITIONS', len(completed), '/33', flush=True)
        if len(completed) == 33:
            print('ALL_RESULTS_READY_FOR_REVIEW', flush=True)
            return
        time.sleep(15)


if __name__ == '__main__':
    main()
