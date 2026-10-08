#!/usr/bin/env python3
"""ROS message encoding regression; ROS ImportError follows offline SKIP rule."""
try:
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid
    from field_pose_capture_sim import raw_grid, field
except ImportError:
    print('SKIP: ROS Humble messages unavailable')
    raise SystemExit(0)


def main():
    raw=Costmap()
    raw.header.frame_id='map'
    raw.metadata.resolution=.05
    raw.metadata.size_x=raw.metadata.size_y=20
    raw.metadata.origin.position.x=raw.metadata.origin.position.y=-.5
    raw.metadata.origin.orientation.w=1.
    raw.data=[253]*400
    result=field.footprint_cost(raw_grid(raw),0,0,0)
    assert result['lethal_cells']>0
    occupancy=OccupancyGrid()
    occupancy.header.frame_id='map'
    occupancy.info.resolution=.05
    occupancy.info.width=occupancy.info.height=20
    occupancy.info.origin=raw.metadata.origin
    occupancy.data=[99]*400
    old=field.footprint_cost(field.grid_from_msg(occupancy),0,0,0)
    assert old['lethal_cells']==0, 'original bug changed; reassess this simulation workaround'
    print('raw-cost lethal detection PASS; original OccupancyGrid 99 misses lethal reproduced')


if __name__=='__main__':
    main()
