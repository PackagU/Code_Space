#!/usr/bin/env python3
"""Persistent Gazebo manual observation at the user's last stopped pose."""
import argparse
import json
import rclpy
from run_manual_mission import Mission


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--x',type=float,required=True)
    parser.add_argument('--y',type=float,required=True)
    parser.add_argument('--yaw',type=float,required=True)
    parser.add_argument('--name',default='user_hold')
    args=parser.parse_args()
    rclpy.init()
    mission=Mission(args.name,900,spawn_pose=(args.x,args.y,args.yaw))
    try:
        mission.bootstrap(start_nav=False)
        mission.stop()
        mission.marker('user_hold_current_position','manual observation; Nav2 stopped')
        print('HOLD_READY '+json.dumps([args.x,args.y,args.yaw]),flush=True)
        while rclpy.ok():
            rclpy.spin_once(mission.node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            mission.finish()
            rclpy.shutdown()


if __name__=='__main__':
    main()
