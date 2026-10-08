#!/usr/bin/env python3
"""S6 proposed recorder fix: reuse original script in an ignored temporary copy.

Expose hidden action topics and stop the exact inner recorder first. Dedicated
simulation containers only. Original fieldctl/record scripts are unchanged.
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]


def stop_inner(container, name, directory):
    # Match only the dedicated recorder and its exact output path.
    code='''import os,signal,sys,json
from pathlib import Path
expected='/ros2_ws/logs/field_bags/'+sys.argv[1];pids=[]
for path in Path('/proc').glob('[0-9]*/cmdline'):
 try: words=[w.decode() for w in path.read_bytes().split(b'\\0') if w]
 except (OSError,UnicodeError): continue
 if words and words[0].endswith(('python3','ros2')) and 'bag' in words and 'record' in words and expected in words:
  pid=int(path.parent.name);os.kill(pid,signal.SIGINT);pids.append(pid)
print(json.dumps(pids))
'''
    stop=subprocess.run(['docker','exec',container,'python3','-c',code,name],
                        capture_output=True,text=True,timeout=15)
    (directory/'stop.log').write_text(stop.stdout+stop.stderr)
    return stop.returncode


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--container',default='packagu_real_map_sim')
    parser.add_argument('--name',default='sim_s6_fixed')
    parser.add_argument('--duration',type=float,default=15)
    args=parser.parse_args()
    if not re.fullmatch(r'packagu_real_map[A-Za-z0-9_.-]*',args.container) or not re.fullmatch(r'sim_[A-Za-z0-9_-]+',args.name):
        raise SystemExit('dedicated simulation container and sim_ name required')
    directory=ROOT/'logs/real_map_sim'/args.name
    directory.mkdir(parents=True,exist_ok=True)
    source=ROOT/'scripts/record_field_bag.sh'
    text=source.read_text()
    assert text.count("in_container 'ros2 topic list'")==1
    text=text.replace("in_container 'ros2 topic list'", "in_container 'ros2 topic list --include-hidden-topics'")
    assert text.count('record_opts="--qos-profile-overrides-path') == 1
    text=text.replace('record_opts="--qos-profile-overrides-path',
                      'record_opts="--include-hidden-topics --qos-profile-overrides-path')
    marker='output_dir="/ros2_ws/logs/field_bags/${BAG_NAME}"'
    assert text.count(marker)==1
    text=text.replace(marker,'topics+=(/clock /sim/ground_truth /global_costmap/costmap_raw /local_costmap/costmap_raw /sim/manual_stage /sim/manual_key /sim/payload_state)\n'+marker)
    temporary=directory/'record_field_bag_sim.sh'
    temporary.write_text(text)
    result={'scope':'simulation','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'steps':[]}
    env=dict(os.environ,PACKAGU_CONTAINER_NAME=args.container,INCLUDE_CONTROL_DIAGNOSTICS='1',INCLUDE_NAV_DIAGNOSTICS='1')
    bag=ROOT/'logs/field_bags'/args.name
    log=open(directory/'record.log','w')
    proc=subprocess.Popen(['bash',str(temporary),args.name],env=env,stdout=log,stderr=subprocess.STDOUT)
    def size(): return sum(p.stat().st_size for p in bag.glob('*.db3*'))
    try:
        deadline=time.monotonic()+40
        while not size() and time.monotonic()<deadline and proc.poll() is None: time.sleep(.2)
        first=size();time.sleep(2);last=size()
        result['steps'].append({'step':'record_start_check','alive':proc.poll() is None,'bytes_before':first,'bytes_after':last})
        if not first or last<=first or proc.poll() is not None:
            raise RuntimeError('recorder failed to start/grow; inspect dedicated log')
        time.sleep(max(1,min(args.duration,60)))
    except BaseException as exc:
        result['error']=str(exc) or type(exc).__name__
    finally:
        try:
            result['steps'].append({'step':'record_stop_inner',
                                   'exit':stop_inner(args.container,args.name,directory)})
            proc.wait(timeout=90)
        except (subprocess.TimeoutExpired, OSError) as exc:
            result['error']='recorder cleanup failed: '+str(exc)
            proc.terminate()
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill();proc.wait()
        log.close()
    result['bag_finalized']=(bag/'metadata.yaml').exists()
    result['original_wrapper_exit']=proc.returncode
    def inside(label,script,extra):
        command=['docker','exec','-w','/ros2_ws',args.container,'bash','-c',
                 'source /opt/ros/humble/setup.bash; source install/setup.bash; exec python3 "$@"',
                 'sim_record_check',script,*extra]
        output=subprocess.run(command,capture_output=True,text=True,timeout=120)
        (directory/(label+'.log')).write_text(output.stdout+output.stderr)
        return output.returncode
    path='/ros2_ws/logs/field_bags/'+args.name
    result['contract_exit']=inside('bag_contract','scripts/bag_contract.py',['inspect',path,'--require','/scan','--require','/odom',
        '--require','/tf','--require','/tf_static','--require','/amcl_pose','--require','/plan',
        '--require','/sim/ground_truth','--require','/navigate_to_pose/_action/status'])
    result['analysis_exit']=inside('analyze_original','scripts/analyze_nav_bag.py',[path])
    result['completed']=not result.get('error') and result['bag_finalized'] and proc.returncode==0 and result['contract_exit']==0 and result['analysis_exit']==0
    (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['completed'] else 1)


if __name__=='__main__': main()
