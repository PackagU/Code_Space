#!/usr/bin/env python3
"""G4: Saturday command-order rehearsal with the UNCHANGED field CLI (scripts/fieldctl) on the sim stack.

record start -> record check -> pose capture -> goal f1_locker -> (if not finished by --stop-after s:
stop) -> cancel --all -> final action status -> resume -> record stop -> bag metadata / hidden action
status topic / analyze_nav_bag. Also re-checks the 9/15 field_pose_capture lethal false negative.
fieldctl targets the isolated sim container via PACKAGU_CONTAINER_NAME (never ros2_humble).
Host-side; simulation only.
  python3 sim/real_maps/run_g4_rehearsal.py --tag a --stop-after 300
  python3 sim/real_maps/run_g4_rehearsal.py --tag b --stop-after 40     # forces the stop path
"""
import argparse
import json
import os
import re
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SC = HERE/'sim_container.sh'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tag', required=True)
    parser.add_argument('--stop-after', type=float, default=300.0, help='[제안값] operator wall seconds before stop')
    parser.add_argument('--params', default='P0')
    args = parser.parse_args()
    container = os.environ.get('PACKAGU_SIM_CONTAINER', 'packagu_sim_e2e_g4')
    if not re.fullmatch(r'packagu_sim[A-Za-z0-9_.-]*', container):
        raise SystemExit('dedicated packagu_sim* container required')
    stamp = time.strftime('%Y%m%d_%H%M%S')
    run = f'g4_{args.tag}_{stamp}'
    bag_name = f'sim_g4_{args.tag}_{stamp}'
    out = ROOT/'logs/real_map_sim'/run
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PACKAGU_SIM_CONTAINER=container, PACKAGU_CONTAINER_NAME=container)
    result = {'scope': 'simulation', 'run': run, 'container': container, 'stop_after_s': args.stop_after,
              'steps': []}

    def step(label, command, timeout=180):
        t0 = time.monotonic()
        proc = subprocess.run(command, env=env, capture_output=True, text=True, timeout=timeout)
        (out/f'{len(result["steps"]):02d}_{label}.log').write_text(proc.stdout+proc.stderr)
        result['steps'].append({'step': label, 'exit': proc.returncode, 'wall_s': round(time.monotonic()-t0, 1),
                                'tail': (proc.stdout+proc.stderr).strip().splitlines()[-1:]})
        print(f'G4 {label}: exit={proc.returncode}', flush=True)
        return proc

    ctl = ['bash', str(ROOT/'scripts/fieldctl')]
    start = step('start_sim', ['bash', str(HERE/'start_sim.sh'), '--floor', 'F1', '--params', args.params,
                               '--spawn', 'f1_initial_test', '--name', run+'_stack', '--no-record'], 700)
    try:
        if start.returncode:
            raise RuntimeError('sim stack failed to start')
        step('record_start', [*ctl, 'record', 'start', bag_name])
        check = step('record_check', [*ctl, 'record', 'check'])
        result['record_check_pass'] = check.returncode == 0
        cap = step('pose_capture', [*ctl, 'pose', 'capture', f'{bag_name}_pre', 'F1', '--duration', '10'])
        result['pose_capture_exit'] = cap.returncode
        step('resume_before_goal', [*ctl, 'resume'])
        goal_log = out/'goal.log'
        t0 = time.monotonic()
        with open(goal_log, 'w') as handle:
            goal = subprocess.Popen([*ctl, 'goal', 'f1_locker', 'F1'], env=env, stdout=handle,
                                    stderr=subprocess.STDOUT)
            while goal.poll() is None and time.monotonic()-t0 < args.stop_after:
                time.sleep(1)
            result['goal_finished_before_stop'] = goal.poll() is not None
            result['goal_wall_s'] = round(time.monotonic()-t0, 1)
            if goal.poll() is None:
                step('stop', [*ctl, 'stop'])
                try:
                    goal.wait(60)
                except subprocess.TimeoutExpired:
                    goal.kill()
        text = goal_log.read_text()
        m = re.search(r'goal result: id=\w+ status=(\w+)', text)
        result['goal_status'] = m.group(1) if m else None
        result['goal_exit'] = goal.returncode
        step('cancel_all', [*ctl, 'cancel', '--all'])
        status = step('final_action_status', ['bash', str(SC), 'exec', 'python3', 'sim/real_maps/action_status_check.py'])
        try:
            result['final_action_status'] = json.loads(status.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            result['final_action_status'] = None
        step('resume', [*ctl, 'resume'])
        step('record_stop', [*ctl, 'record', 'stop'])
        bag = ROOT/'logs/field_bags'/bag_name
        deadline = time.monotonic()+20
        while not (bag/'metadata.yaml').exists() and time.monotonic() < deadline:
            time.sleep(.5)
        result['record_stop_metadata_present'] = (bag/'metadata.yaml').exists()
        if result['record_stop_metadata_present']:
            meta = (bag/'metadata.yaml').read_text()
            result['bag_has_action_status'] = '/navigate_to_pose/_action/status' in meta
            analysis = step('analyze_nav_bag', ['bash', str(SC), 'exec', 'python3', 'scripts/analyze_nav_bag.py',
                                                f'/ros2_ws/logs/field_bags/{bag_name}'])
            result['analysis_exit'] = analysis.returncode
        lethal = step('pose_capture_lethal_regression', ['bash', str(SC), 'exec', 'python3',
                                                          'sim/real_maps/test_pose_capture_sim.py'])
        result['pose_capture_lethal_false_negative_still_present'] = (
            lethal.returncode == 0 and 'misses lethal reproduced' in (lethal.stdout+lethal.stderr))
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        step('stop_sim', ['bash', str(HERE/'start_sim.sh'), '--stop'], 180)
    (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print('G4_RESULT '+json.dumps({k: v for k, v in result.items() if k != 'steps'}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
