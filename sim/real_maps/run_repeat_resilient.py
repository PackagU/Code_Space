#!/usr/bin/env python3
"""Host runner: retry only invalid startup attempts, preserve every bag.

A navigation abort/timeout is a measured result and is never retried as startup.
"""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT/'logs/real_map_sim'
STARTUP = ('AMCL lifecycle', 'AMCL/TF initialization', 'Nav2 action server unavailable',
           'Nav2 localization lifecycle', 'Nav2 navigation lifecycle',
           'map guard goal FAIL', 'recorder not alive/growing')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repeat',type=int,required=True,choices=(1,2,3))
    parser.add_argument('--domain',type=int,required=True)
    parser.add_argument('--wait-existing',action='store_true')
    args=parser.parse_args()
    name=f'packagu_real_map_r{args.repeat}'
    env=dict(os.environ,PACKAGU_SIM_CONTAINER=name,PACKAGU_SIM_DOMAIN=str(args.domain),SIM_SKIP_PREPARE='1')
    if args.wait_existing:
        while subprocess.run(['docker','inspect','--format','{{.State.Running}}',name],
                             capture_output=True,text=True).stdout.strip()=='true':
            time.sleep(3)
    retries={}
    for attempt in range(12):
        invalid=[]
        for file in LOG.glob(f'S*_r{args.repeat}/result.json'):
            data=json.loads(file.read_text())
            error=data.get('error') or ''
            if error and not data.get('action_results') and any(s in error for s in STARTUP):
                invalid.append(file.parent)
        for directory in invalid:
            retries[directory.name]=retries.get(directory.name,0)+1
            if retries[directory.name]>3:
                raise SystemExit('startup failed three retries: '+directory.name)
            # Docker-created directories may be owned by its mapped UID.
            code='from pathlib import Path; import shutil,sys; p=Path(sys.argv[1]); out=Path(sys.argv[2]); out.mkdir(parents=True,exist_ok=True); shutil.move(str(p),str(out/p.name))'
            relative=directory.relative_to(ROOT)
            destination=Path('logs/real_map_sim/preflight')/f'bootstrap_retry_{args.repeat}_{time.time_ns()}'
            subprocess.run(['docker','run','--rm','-v',str(ROOT)+':/ros2_ws','-w','/ros2_ws',
                '--entrypoint','python3',env.get('PACKAGU_SIM_IMAGE','ghcr.io/packagu/ros2-humble-slam:humble'),
                '-c',code,str(relative),str(destination)],check=True)
        with open(LOG/f'resilient_r{args.repeat}.log','a') as output:
            process=subprocess.run(['bash',str(ROOT/'sim/real_maps/run_suite.sh'),'--repeats','3',
                '--filter',f'_r{args.repeat}','--resume','--prioritize','S3'],env=env,stdout=output,stderr=subprocess.STDOUT)
        if process.returncode==0:
            subprocess.run(['bash','-c','source /opt/ros/humble/setup.bash && exec python3 "$1"',
                            'sim_localization',str(ROOT/'sim/real_maps/analyze_localization.py')],check=True)
            subprocess.run(['python3',str(ROOT/'sim/real_maps/summarize.py')],check=True)
            subprocess.run(['python3',str(ROOT/'sim/real_maps/write_report.py')],check=True)
            return
        if not any((json.loads(p.read_text()).get('error') or '') for p in LOG.glob(f'S*_r{args.repeat}/result.json')):
            raise SystemExit('non-scenario launcher failure; inspect resilient log')
    raise SystemExit('startup retries exhausted')


if __name__=='__main__':
    main()
