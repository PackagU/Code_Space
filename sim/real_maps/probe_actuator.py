#!/usr/bin/env python3
"""Isolated low-speed actuator diagnostic; never sends commands to mission domains.

Compare Gazebo's original second acceleration limiter to direct application of
an already limited command. This is a model diagnostic, not a Nav2 scenario.
"""
import csv
import hashlib
import json
import math
import os
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from run_scenarios import ROOT, HERE, LOG, Observer, Processes, pose_values


def main():
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) != 218:
        raise SystemExit('actuator diagnostic requires isolated ROS domain 218')
    rclpy.init()
    results=[]
    for acceleration in (8, 0):
        directory=LOG/f'actuator_accel_{acceleration}'
        directory.mkdir(parents=True, exist_ok=True)
        if (directory/'result.json').exists():
            raise SystemExit('existing actuator diagnostic; preserve evidence before a new run')
        tree=ET.parse(HERE/'generated/robot.urdf')
        tree.find("gazebo/plugin[@name='diff_drive']/max_wheel_acceleration").text=str(acceleration)
        robot=directory/'robot.urdf';tree.write(robot,encoding='unicode')
        world=directory/'flat.world'
        world.write_text('''<sdf version="1.6"><world name="default"><gravity>0 0 -9.8</gravity>
<physics type="ode"><max_step_size>.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
<model name="ground"><static>true</static><link name="ground"><collision name="ground"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>
</world></sdf>''')
        processes=Processes(directory);node=Observer()
        node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        command=node.create_publisher(Twist,'/sim/cmd_vel_drive',10)
        velocities={};samples=[];stage='startup'
        node.create_subscription(JointState,'/joint_states',lambda m:velocities.update(dict(zip(m.name,m.velocity))),10)
        try:
            processes.start('gazebo',['gzserver',str(world),'-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'])
            processes.start('spawn',['ros2','run','gazebo_ros','spawn_entity.py','-entity','actuator_probe',
                                    '-file',str(robot),'-z','.005','-timeout','60'])
            if not node.wait(lambda:node.truth is not None,90):raise RuntimeError('actuator ground truth unavailable')
            for stage, v, w, duration in [('forward',.1,0,15),('stop_before_spin',0,0,5),
                                         ('slow_left',0,.03,20),('stop_between',0,0,10),
                                         ('slow_right',0,-.03,20),('final_stop',0,0,15)]:
                beginning=node.get_clock().now().nanoseconds/1e9
                start_pose=pose_values(node.truth.pose.pose)
                while node.get_clock().now().nanoseconds/1e9-beginning<duration:
                    msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);command.publish(msg)
                    node.pause(.02)
                    x,y,yaw=pose_values(node.truth.pose.pose)
                    samples.append([stage,node.get_clock().now().nanoseconds/1e9,x,y,yaw,
                                    node.truth.twist.twist.angular.z,
                                    velocities.get('left_wheel_joint'),velocities.get('right_wheel_joint'),v,w])
                end_pose=pose_values(node.truth.pose.pose)
                results.append(dict(acceleration=acceleration,stage=stage,command_v=v,command_w=w,
                                    displacement_m=math.hypot(end_pose[0]-start_pose[0],end_pose[1]-start_pose[1]),
                                    yaw_change_rad=math.atan2(math.sin(end_pose[2]-start_pose[2]),math.cos(end_pose[2]-start_pose[2]))))
                print(json.dumps(results[-1]),flush=True)
            with open(directory/'trace.csv','w',newline='') as handle:
                writer=csv.writer(handle,lineterminator='\n');writer.writerow(['stage','sim_t','x','y','yaw','true_w','left_joint_w','right_joint_w','cmd_v','cmd_w']);writer.writerows(samples)
            (directory/'result.json').write_text(json.dumps({'scope':'isolated actuator model diagnostic',
                'robot_sha256':hashlib.sha256(robot.read_bytes()).hexdigest(),
                'stages':[r for r in results if r['acceleration']==acceleration]},indent=2)+'\n')
        finally:
            command.publish(Twist());node.pause(.2);processes.cleanup()
            node.client.destroy();node.listener.unregister();node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':main()
