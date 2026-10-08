#!/usr/bin/env python3
"""Reviewable simulation-only correction: raw costs and optional AMCL refresh.

Uses the original field capture evaluation/CLI without modifying field code.
Never saves field waypoints: --save and --replace are explicitly prohibited.
"""
import math
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import field_pose_capture as field
REFRESH_AMCL = False


def raw_grid(msg):
    q=msg.metadata.origin.orientation
    return dict(frame=msg.header.frame_id,width=msg.metadata.size_x,height=msg.metadata.size_y,
                resolution=msg.metadata.resolution,origin_x=msg.metadata.origin.position.x,
                origin_y=msg.metadata.origin.position.y,
                origin_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),data=list(msg.data))


def capture_raw(duration):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.msg import Costmap
    from std_srvs.srv import Empty
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node=Node('sim_pose_capture_raw')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    amcl,transforms,grids,covariance=[],[],{},[None]
    def on_amcl(msg):
        p=msg.pose.pose
        amcl.append((p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)))
        covariance[0]=list(msg.pose.covariance)
    node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',on_amcl,qos)
    for topic in ('global_costmap','local_costmap'):
        node.create_subscription(Costmap,f'/{topic}/costmap_raw',lambda msg,k=topic:grids.__setitem__(k,msg),qos)
    buffer=Buffer()
    listener=TransformListener(buffer,node)
    refresh=node.create_client(Empty,'/request_nomotion_update')
    end=time.monotonic()+duration
    next_tf=next_refresh=time.monotonic()
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if REFRESH_AMCL and time.monotonic()>=next_refresh and refresh.service_is_ready():
                next_refresh=time.monotonic()+.5
                refresh.call_async(Empty.Request())
            if time.monotonic()>=next_tf:
                next_tf=time.monotonic()+.2
                if buffer.can_transform('map','base_footprint',Time()):
                    t=buffer.lookup_transform('map','base_footprint',Time()).transform
                    transforms.append((t.translation.x,t.translation.y,2*math.atan2(t.rotation.z,t.rotation.w)))
        local_tf=None
        if 'local_costmap' in grids and buffer.can_transform(grids['local_costmap'].header.frame_id,'base_footprint',Time()):
            t=buffer.lookup_transform(grids['local_costmap'].header.frame_id,'base_footprint',Time()).transform
            local_tf=(t.translation.x,t.translation.y,2*math.atan2(t.rotation.z,t.rotation.w))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return amcl,transforms,{k:raw_grid(v) for k,v in grids.items()},covariance[0],local_tf


def main():
    global REFRESH_AMCL
    if '--refresh-amcl' in sys.argv:
        sys.argv.remove('--refresh-amcl')
        REFRESH_AMCL = True
    if '--save' in sys.argv or '--replace' in sys.argv:
        raise SystemExit('simulation capture never saves/replaces field waypoints')
    field.capture=capture_raw
    return field.main()


if __name__=='__main__':
    sys.exit(main())
