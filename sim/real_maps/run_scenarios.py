#!/usr/bin/env python3
"""Isolated Gazebo/Nav2 A/B runner. All results are simulation evidence only."""
import argparse
import csv
import hashlib
import json
import math
import os
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import Log
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from clearance import minimum_clearance

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
LOG = ROOT/'logs/real_map_sim'
MAPS = ROOT/'maps/field'
CONFIG = ROOT/'src/slam_pkg/config'
IDLE = (-4.518, 2.179, -1.701)
OLD_IDLE = (-3.725, 2.075, -1.701)
PARAMS = {'P0':'nav2_params_pre_wallpush_20260915.yaml', 'P1':'nav2_params.yaml',
          'v011':'nav2_params_speed_v011.yaml', 'v012':'nav2_params_speed_v012.yaml'}
TOPICS = ['/clock','/scan','/odom','/tf','/tf_static','/sim/ground_truth','/amcl_pose',
          '/plan','/global_costmap/costmap','/global_costmap/costmap_updates',
          '/global_costmap/costmap_raw','/local_costmap/costmap','/local_costmap/costmap_updates',
          '/local_costmap/costmap_raw','/cmd_vel','/cmd_vel_safe','/sim/cmd_vel_drive',
          '/sim/shim_stats','/rosout','/drive/ready','/nav_safety/status',
          '/nav_safety/stopped','/navigate_to_pose/_action/status','/initialpose']


def pose_values(p):
    return p.position.x, p.position.y, math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),
                                                1-2*(p.orientation.y**2+p.orientation.z**2))


class Observer(Node):
    def __init__(self):
        super().__init__('real_map_scenario_observer', automatically_declare_parameters_from_overrides=True)
        self.truth = self.amcl = self.scan = None
        self.measure = False
        self.samples = []
        self.events = {'collision_ahead':0, 'lethal_start':0, 'controller_missed':0}
        self.stats = {'rpm_blocks':0,'ready_drops':0}
        self.boxes = []
        self.last_progress = 0
        self.progress_path = None
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m:setattr(self,'amcl',m), latched)
        self.create_subscription(LaserScan, '/scan', lambda m:setattr(self,'scan',m), qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(String, '/sim/shim_stats', lambda m:setattr(self,'stats',json.loads(m.data)),10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',latched)
        self.stop = self.create_publisher(Bool,'/nav_safety/stop',10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def on_truth(self, msg):
        self.truth = msg
        if self.measure:
            x,y,yaw = pose_values(msg.pose.pose)
            clearance = minimum_clearance(self.boxes,x,y,yaw)
            estimate = pose_values(self.amcl.pose.pose) if self.amcl else (None,None,None)
            self.samples.append([msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,x,y,yaw,clearance,*estimate])
            stamp=self.samples[-1][0]
            if self.progress_path and stamp-self.last_progress>=30:
                self.last_progress=stamp
                progress=dict(sim_t=stamp,true_pose=[x,y,yaw],wall_m=clearance,
                              min_wall_m=min(s[4] for s in self.samples),**self.events,**self.stats)
                self.progress_path.write_text(json.dumps(progress))
                print('PROGRESS '+self.progress_path.parent.name+' '+json.dumps(progress),flush=True)

    def on_log(self,msg):
        if self.measure:
            text = msg.msg.lower()
            self.events['collision_ahead'] += int('collision ahead' in text)
            self.events['lethal_start'] += int('starting point in lethal space' in text)
            self.events['controller_missed'] += int('control loop missed' in text)

    def wait(self, predicate, timeout):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():
                return True
            rclpy.spin_once(self,timeout_sec=.05)
        return bool(predicate())

    def pause(self,seconds):
        self.wait(lambda:False,seconds)


class Processes:
    def __init__(self, directory):
        self.items=[]
        self.directory=directory

    def start(self,name,args):
        handle=open(self.directory/f'{name}.log','w')
        proc=subprocess.Popen(args,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        self.items.append((proc,handle))
        return proc

    def stop_one(self,proc):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGKILL)
                    proc.wait()

    def cleanup(self):
        for proc,handle in reversed(self.items):
            self.stop_one(proc)
            handle.close()


def guard(floor,stage,directory):
    extra = []
    if stage == 'goal':
        # Humble's per-node file logger can buffer map_io messages until exit.
        # Give the unchanged guard the actual launch stdout, with its live PID.
        import importlib.util
        spec = importlib.util.spec_from_file_location('sim_guard_source', ROOT/'scripts/field_map_guard.py')
        source = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(source)
        pids = source.live_map_servers('/proc')
        outputs = list(directory.glob('nav2*.log'))
        if len(pids) == 1 and outputs:
            logdir = directory/'guard_stdout'
            logdir.mkdir(exist_ok=True)
            output = max(outputs, key=lambda p:p.stat().st_mtime_ns)
            (logdir/f'map_server_{pids[0]}_stdout.log').write_text(output.read_text())
            extra = ['--log-dir', str(logdir)]
    result=subprocess.run([sys.executable,str(ROOT/'scripts/field_map_guard.py'),'check',
                           '--floor',floor,'--stage',stage,'--record-dir',str(directory/'guard'),*extra],
                          capture_output=True,text=True,timeout=15)
    (directory/f'guard_{stage}_{time.time_ns()}.log').write_text(result.stdout+result.stderr)
    if result.returncode:
        raise RuntimeError(f'map guard {stage} FAIL: {result.stdout[-1000:]}')


def record_check(proc,bag,node,directory):
    def size():
        return sum(p.stat().st_size for p in bag.glob('*.db3*'))
    node.wait(lambda:bool(list(bag.glob('*.db3'))),15)
    first=size()
    ok=node.wait(lambda:proc.poll() is None and size()>first,15)
    (directory/'record_check.json').write_text(json.dumps({'result':'PASS' if ok else 'FAIL',
                                                          'bytes_before':first,'bytes_after':size()}))
    if not ok:
        raise RuntimeError('recorder not alive/growing')


def lifecycle_start(node, group):
    from nav2_msgs.srv import ManageLifecycleNodes
    client = node.create_client(ManageLifecycleNodes, f'/lifecycle_manager_{group}/manage_nodes')
    if not node.wait(client.service_is_ready, 20):
        raise RuntimeError(f'Nav2 {group} lifecycle service unavailable')
    # Allow DDS response endpoints to match before the manager configures nodes.
    node.pause(3)
    request = ManageLifecycleNodes.Request()
    request.command = request.STARTUP
    future = client.call_async(request)
    if not node.wait(future.done, 45) or not future.result().success:
        raise RuntimeError(f'Nav2 {group} lifecycle activation failed')
    node.destroy_client(client)


def scenario_list(repeats):
    wp=json.loads((MAPS/'waypoints.json').read_text())['waypoints']
    def point(name):
        p=wp[name]
        return (p['x'],p['y'],p['yaw_rad'])
    cases=[]
    for rep in range(1,repeats+1):
        for p in ('P0','P1'):
            for target in ('f2_elevator_entry','f2_elevator_staging_v1'):
                cases.append(dict(name=f'S1_{target}_{p}_r{rep}',scenario='S1',floor='F2',params=p,
                                  spawn=point('f2_delivery_destination'),initial=point('f2_delivery_destination'),
                                  goals=[point(target)]))
            cases.append(dict(name=f'S2_{p}_r{rep}',scenario='S2',floor='F1',params=p,spawn=IDLE,initial=IDLE,
                              goals=[point('f1_locker'),point('f1_elevator_entry'),IDLE]))
            cases.append(dict(name=f'S3_{p}_r{rep}',scenario='S3',floor='F1',params=p,spawn=IDLE,initial=OLD_IDLE,
                              goals=[point('f1_locker')]))
        for p in ('P1','v011','v012'):
            cases.append(dict(name=f'S5_{p}_r{rep}',scenario='S5',floor='F1',params=p,spawn=IDLE,initial=IDLE,
                              goals=[point('f1_locker'),point('f1_elevator_entry'),IDLE]))
    return cases


def run_case(case, timeout, robot_path=None):
    robot_path=Path(robot_path) if robot_path is not None else HERE/'generated/robot.urdf'
    directory=LOG/case['name']
    directory.mkdir(parents=True,exist_ok=True)
    (directory/'case.json').write_text(json.dumps(case,indent=2))
    processes=Processes(directory)
    node=Observer()
    node.boxes=np.array(json.loads((HERE/f"generated/{case['floor'].lower()}_boxes.json").read_text()))
    node.progress_path=directory/'progress.json'
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    result=dict(name=case['name'],scenario=case['scenario'],floor=case['floor'],params=case['params'],
                validation_scope='simulation',action_results=[],error=None)
    result['robot_urdf_sha256']=hashlib.sha256(robot_path.read_bytes()).hexdigest()
    result['params_sha256']=hashlib.sha256((CONFIG/PARAMS[case['params']]).read_bytes()).hexdigest()
    bag=directory/'bag'
    try:
        guard(case['floor'],'pre-nav',directory)
        processes.start('gazebo',['gzserver','--verbose',str(HERE/f"generated/{case['floor'].lower()}.world"),
                                  '-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'])
        rsp=directory/'rsp.yaml'
        rsp.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{
            'use_sim_time':True,'robot_description':robot_path.read_text()}}}))
        processes.start('rsp',['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(rsp)])
        x,y,yaw=case['spawn']
        spawn=processes.start('spawn',['ros2','run','gazebo_ros','spawn_entity.py','-entity','packagu_sim',
            '-file',str(robot_path),'-x',str(x),'-y',str(y),'-z','.005','-Y',str(yaw),'-timeout','60'])
        node.wait(lambda:(node.truth is not None and node.scan is not None) or
                  (spawn.poll() is not None and spawn.poll()!=0),90)
        if spawn.poll() is not None and spawn.poll()!=0:
            raise RuntimeError('robot spawn failed; see spawn.log')
        if node.truth is None or node.scan is None:
            raise RuntimeError('GROUND_TRUTH_OR_SCAN_UNAVAILABLE: user confirmation required')
        node.pause(2)
        result['scan_samples']=len(node.scan.ranges)
        result['truth_spawn_pose']=pose_values(node.truth.pose.pose)
        processes.start('bridge',[sys.executable,str(HERE/'opencr_shim.py'),'--ros-args',
            '--params-file',str(ROOT/'src/drive_pkg/config/drive_calib.yaml'),
            '-p','cmd_vel_topic:=/cmd_vel_safe','-p','publish_tf:=false','-p','use_sim_time:=true'])
        gate=[sys.executable,str(ROOT/'src/drive_pkg/drive_pkg/nav_safety_gate.py'),'--ros-args',
              '--params-file',str(ROOT/'src/drive_pkg/config/nav_safety.yaml'),'-p','use_sim_time:=true']
        if case['params']=='v012':
            gate += ['-p','max_linear_speed:=0.13']
        processes.start('gate',gate)
        map_yaml=MAPS/('f1/f1_manual_clean_v3.yaml' if case['floor']=='F1' else 'f2/f2_nav_unknown_v1.yaml')
        processes.start('nav2',['ros2','launch','nav2_bringup','bringup_launch.py',f'map:={map_yaml}',
            f'params_file:={CONFIG/PARAMS[case["params"]]}','use_sim_time:=true','autostart:=false',
            'use_composition:=False','use_respawn:=False'])
        recorder=processes.start('record',['ros2','bag','record','--include-hidden-topics','--max-cache-size','262144',
            '--qos-profile-overrides-path',str(ROOT/'scripts/rosbag_qos_overrides.yaml'),'-o',str(bag),*TOPICS])
        record_check(recorder,bag,node,directory)
        lifecycle_start(node, 'localization')
        from lifecycle_msgs.srv import GetState
        amcl_state=node.create_client(GetState,'/amcl/get_state')
        active=False
        for _ in range(30):
            if amcl_state.service_is_ready():
                state_future=amcl_state.call_async(GetState.Request())
                if node.wait(state_future.done,2) and state_future.result().current_state.id==3:
                    active=True
                    break
            node.pause(.5)
        if not active:
            raise RuntimeError('AMCL lifecycle activation failed')
        initial=PoseWithCovarianceStamped()
        initial.header.frame_id='map'
        initial.pose.pose.position.x,initial.pose.pose.position.y=case['initial'][:2]
        initial.pose.pose.orientation.z=math.sin(case['initial'][2]/2)
        initial.pose.pose.orientation.w=math.cos(case['initial'][2]/2)
        # Match the actual field_nav_cli pose-set operator uncertainty.
        initial.pose.covariance[0]=initial.pose.covariance[7]=.25
        initial.pose.covariance[35]=math.radians(15)**2
        for _ in range(3):
            initial.header.stamp=node.get_clock().now().to_msg()
            node.initial.publish(initial)
            node.pause(.5)
        if not node.wait(lambda:node.amcl is not None and node.buffer.can_transform('map','base_footprint',Time()),20):
            raise RuntimeError('AMCL/TF initialization failed')
        lifecycle_start(node, 'navigation')
        if not node.wait(lambda:node.client.server_is_ready(),75):
            raise RuntimeError('Nav2 action server unavailable')
        node.pause(3)
        guard(case['floor'],'goal',directory)
        capture=processes.start('pose_capture',[sys.executable,str(ROOT/'scripts/field_pose_capture.py'),
            f"sim_{case['name']}",case['floor'],'--duration','10','--record-dir',str(directory/'capture')])
        node.wait(lambda:capture.poll() is not None,20)
        result['pose_capture_exit']=capture.poll()
        fixed_capture=processes.start('pose_capture_raw',[sys.executable,str(HERE/'field_pose_capture_sim.py'),
            f"sim_raw_{case['name']}",case['floor'],'--duration','10','--record-dir',str(directory/'capture_raw')])
        node.wait(lambda:fixed_capture.poll() is not None,20)
        result['pose_capture_raw_exit']=fixed_capture.poll()
        stats_before=node.stats.copy()
        node.measure=True
        start_sim=node.get_clock().now().nanoseconds/1e9
        start_wall=time.monotonic()
        for goal_index, point in enumerate(case['goals']):
            guard(case['floor'],'goal',directory)
            goal=NavigateToPose.Goal()
            goal.pose.header.frame_id='map'
            goal.pose.header.stamp=node.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y=point[:2]
            goal.pose.pose.orientation.z=math.sin(point[2]/2)
            goal.pose.pose.orientation.w=math.cos(point[2]/2)
            future=node.client.send_goal_async(goal)
            if not node.wait(future.done,15):
                raise RuntimeError('goal acceptance timed out')
            handle=future.result()
            if not handle.accepted:
                result['action_results'].append('REJECTED')
                break
            done=handle.get_result_async()
            deadline=node.get_clock().now().nanoseconds/1e9+timeout
            finished=node.wait(lambda:done.done() or node.get_clock().now().nanoseconds/1e9>deadline,timeout*2)
            if not finished or not done.done():
                cancel=handle.cancel_goal_async()
                node.wait(cancel.done,10)
                result['action_results'].append('TIMEOUT')
                break
            status={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(done.result().status,str(done.result().status))
            result['action_results'].append(status)
            (directory/f'goal_{goal_index}.json').write_text(json.dumps({'goal':point,'status':status,
                'true_pose':pose_values(node.truth.pose.pose),'amcl_pose':pose_values(node.amcl.pose.pose)}))
            if status!='SUCCEEDED':
                break
            node.pause(1)
        result['elapsed_sim_s']=node.get_clock().now().nanoseconds/1e9-start_sim
        result['elapsed_wall_s']=time.monotonic()-start_wall
        node.stop.publish(Bool(data=True))
        node.pause(2)
        node.measure=False
        result.update(node.events)
        result['rpm_blocks']=node.stats['rpm_blocks']-stats_before['rpm_blocks']
        result['ready_drops']=node.stats['ready_drops']-stats_before['ready_drops']
        result['truth_samples']=len(node.samples)
        result['min_wall_m']=min((s[4] for s in node.samples),default=None)
        corner=[s[4] for s in node.samples if math.hypot(s[1]+12.95,s[2]+5.25)<=3]
        result['corner_min_wall_m']=min(corner,default=None)
        processes.stop_one(recorder)
        with open(directory/'clearance.csv','w',newline='') as handle:
            writer=csv.writer(handle)
            writer.writerow(['sim_t','true_x','true_y','true_yaw','footprint_wall_m','amcl_x','amcl_y','amcl_yaw'])
            writer.writerows(node.samples)
        analysis=subprocess.run([sys.executable,str(ROOT/'scripts/analyze_nav_bag.py'),str(bag)],
                                capture_output=True,text=True,timeout=90)
        (directory/'analysis.log').write_text(analysis.stdout+analysis.stderr)
        result['analysis_exit']=analysis.returncode
        contract=subprocess.run([sys.executable,str(ROOT/'scripts/bag_contract.py'),'inspect',str(bag),
            '--require','/scan','--require','/odom','--require','/tf','--require','/tf_static',
            '--require','/sim/ground_truth','--require','/amcl_pose','--require','/plan'],
            capture_output=True,text=True,timeout=90)
        (directory/'bag_contract.json').write_text(contract.stdout)
        result['bag_contract_exit']=contract.returncode
    except Exception as exc:
        result['error']=str(exc)
        print(f"ERROR {case['name']}: {exc}",flush=True)
    finally:
        node.measure=False
        processes.cleanup()
        node.client.destroy()
        node.listener.unregister()
        node.destroy_node()
    (directory/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--filter',default='',help='case-name substring; empty means complete suite')
    parser.add_argument('--goal-timeout',type=float,default=900)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--prioritize',default='',help='run case names containing this substring first')
    args=parser.parse_args()
    LOG.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    try:
        cases=scenario_list(args.repeats)
        if args.prioritize:
            cases.sort(key=lambda case:args.prioritize not in case['name'])
        for case in cases:
            if args.filter not in case['name']:
                continue
            previous=LOG/case['name']/'result.json'
            if args.resume and previous.exists():
                old=json.loads(previous.read_text())
                if 'truth_samples' in old and not old.get('error'):
                    continue
            if (LOG/case['name']/'bag').exists():
                raise RuntimeError(f"existing bag for {case['name']}; use --resume or archive it")
            print(f"START {case['name']}",flush=True)
            result=run_case(case,args.goal_timeout)
            if result.get('error'):
                # Environment/ground-truth failures invalidate dependent tests.
                raise RuntimeError(result['error'])
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
