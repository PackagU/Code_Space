#!/usr/bin/env python3
"""Compare original world odometry and encoder odometry in a bounded Nav2 turn.

Original field P1, safety gate and bridge logic remain identical. World variants
have identical collision/friction. Diagnostic H* runs are separate from S1–S5.
"""
import json
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from run_scenarios import IDLE, HERE, LOG, run_case


def main():
    if int(os.environ.get('ROS_DOMAIN_ID','0')) != 218:
        raise SystemExit('heading diagnostic requires isolated ROS domain 218')
    rclpy.init()
    try:
        for source in ('world','encoder'):
            name='H1_heading_'+source
            directory=LOG/name
            directory.mkdir(parents=True,exist_ok=True)
            if (directory/'bag').exists():raise RuntimeError('existing diagnostic bag: '+name)
            tree=ET.parse(HERE/'generated/robot.urdf')
            plugin=tree.find("gazebo/plugin[@name='diff_drive']")
            ET.SubElement(plugin,'odometry_source').text='0' if source=='encoder' else '1'
            robot=directory/'robot.urdf';tree.write(robot,encoding='unicode')
            case=dict(name=name,scenario='heading model diagnostic',floor='F1',params='P1',
                      spawn=IDLE,initial=IDLE,goals=[(*IDLE[:2],IDLE[2]+math.pi/2)],
                      odometry_source=source)
            result=run_case(case,120,robot_path=robot)
            print('HEADING_DIAGNOSTIC',source,json.dumps(result),flush=True)
    finally:
        rclpy.shutdown()


if __name__=='__main__':main()
