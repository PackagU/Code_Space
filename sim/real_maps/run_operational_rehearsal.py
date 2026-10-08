#!/usr/bin/env python3
"""Host-side S6: unchanged fieldctl start/check/stop, then verify inner recorder.

Only dedicated packagu_real_map_* simulation containers are accepted. No drive
or goal commands are issued. Original scripts and default configs stay intact.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--container', default='packagu_real_map_sim')
    parser.add_argument('--name', default='sim_s6_rehearsal')
    parser.add_argument('--duration', type=float, default=15)
    args = parser.parse_args()
    import re
    if not re.fullmatch(r'packagu_real_map[A-Za-z0-9_.-]*', args.container):
        raise SystemExit('dedicated simulation container required')
    if not re.fullmatch(r'sim_[A-Za-z0-9_-]+', args.name):
        raise SystemExit('sim_ bag name required')
    directory = ROOT/'logs/real_map_sim'/args.name
    directory.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PACKAGU_CONTAINER_NAME=args.container)
    result = {'scope': 'simulation', 'steps': []}

    def run(label, command, timeout=60):
        proc = subprocess.run(command, env=env, capture_output=True, text=True, timeout=timeout)
        (directory/(label+'.log')).write_text(proc.stdout+proc.stderr)
        result['steps'].append({'step': label, 'exit': proc.returncode})
        return proc

    ctl = ['bash', str(ROOT/'scripts/fieldctl')]
    if run('record_start', [*ctl, 'record', 'start', args.name]).returncode:
        raise SystemExit('record start failed')
    try:
        if run('record_check', [*ctl, 'record', 'check']).returncode:
            raise RuntimeError('record check failed')
        time.sleep(max(1, min(args.duration, 60)))
        original = run('record_stop_original', [*ctl, 'record', 'stop'])
        bag = ROOT/'logs/field_bags'/args.name
        result['original_stop_metadata_present'] = (bag/'metadata.yaml').exists()
        if not result['original_stop_metadata_present']:
            # The outer docker exec can exit while its recorder remains inside.
            # Match exact recorder executable and output path, never pgrep broadly.
            code = '''import os,sys,signal,json
from pathlib import Path
expected = '/ros2_ws/logs/field_bags/'+sys.argv[1]
pids=[]
for path in Path('/proc').glob('[0-9]*/cmdline'):
 try: argv=path.read_bytes().split(b'\\0'); words=[a.decode() for a in argv if a]
 except (OSError,UnicodeError): continue
 if len(words)>3 and words[0].endswith(('python3','ros2')) and 'bag' in words and 'record' in words and expected in words:
  os.kill(int(path.parent.name), signal.SIGINT); pids.append(int(path.parent.name))
print(json.dumps({'signaled_exact_inner_recorder_pids':pids}))
'''
            fix = run('stop_inner_sim_recorder', ['docker', 'exec', args.container, 'python3', '-c', code, args.name])
            if fix.returncode:
                raise RuntimeError('inner recorder stop failed')
        deadline = time.monotonic()+30
        while not (bag/'metadata.yaml').exists() and time.monotonic()<deadline:
            time.sleep(.2)
        result['bag_finalized'] = (bag/'metadata.yaml').exists()
        if not result['bag_finalized']:
            raise RuntimeError('metadata still missing; stop is not a PASS')
        def inside(label, script, extra):
            return run(label, ['docker', 'exec', '-w', '/ros2_ws', args.container, 'bash', '-c',
                'source /opt/ros/humble/setup.bash; source install/setup.bash; exec python3 "$@"',
                'sim_s6', script, '/ros2_ws/logs/field_bags/'+args.name, *extra], 120)
        contract = run('bag_contract',
            ['docker','exec','-w','/ros2_ws',args.container,'python3','scripts/bag_contract.py','inspect',
             '/ros2_ws/logs/field_bags/'+args.name,'--require','/scan','--require','/odom',
             '--require','/tf','--require','/tf_static'])
        analysis = inside('analyze_original', 'scripts/analyze_nav_bag.py', [])
        result['contract_exit'],result['analysis_exit'] = contract.returncode,analysis.returncode
        result['completed'] = contract.returncode == 0 and analysis.returncode == 0
    except Exception as exc:
        result['error'] = str(exc)
        result['completed'] = False
    (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result.get('completed') else 1)


if __name__ == '__main__':
    main()
