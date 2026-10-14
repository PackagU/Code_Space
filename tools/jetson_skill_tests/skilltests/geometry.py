"""Pure 2D geometry shared by the simulation and the Jetson tests (no ROS)."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c*v[0]-s*v[1], s*v[0]+c*v[1])


def unit(v):
    n = math.hypot(v[0], v[1])
    if n < 1e-9:
        raise ValueError('zero-length vector')
    return (v[0]/n, v[1]/n)


def to_frame(pose, point):
    """World point -> coordinates in the frame of pose (x, y, yaw): (forward, left)."""
    dx, dy = point[0]-pose[0], point[1]-pose[1]
    return rotate((dx, dy), -pose[2])


def from_frame(pose, local):
    v = rotate(local, pose[2])
    return (pose[0]+v[0], pose[1]+v[1])


def face_wall_stance(button_xy, wall_normal, mount_xy, centre_dist):
    """Robot pose facing a wall button: centre `centre_dist` from the wall plane along the outward normal, laterally
    shifted so that the arm mount (mount_xy in the robot frame) is in line with the button.
    Returns (x, y, yaw) and the horizontal reach from the mount to the button face."""
    n = unit(wall_normal)
    yaw = math.atan2(-n[1], -n[0])
    left = rotate((1.0, 0.0), yaw+math.pi/2)
    x = button_xy[0]+n[0]*centre_dist-left[0]*mount_xy[1]
    y = button_xy[1]+n[1]*centre_dist-left[1]*mount_xy[1]
    return (x, y, yaw), centre_dist-mount_xy[0]


def press_stance(button_xy, wall_normal, tip_xy, tip_yaw, push):
    """Robot pose from which a taught (fixed-angle) press lands on a wall button: the tip of the press pose (robot
    frame `tip_xy`, pointing `tip_yaw`) ends `push` beyond the button face and points straight into the wall.
    The robot is therefore turned by -tip_yaw from facing the wall (a cycle that presses to the left stands turned
    right)."""
    n = unit(wall_normal)
    yaw = wrap(math.atan2(-n[1], -n[0])-tip_yaw)
    tx, ty = rotate(tip_xy, yaw)
    return (button_xy[0]-n[0]*push-tx, button_xy[1]-n[1]*push-ty, yaw)


def press_error(pose, button_xy, wall_normal, tip_xy, tip_yaw):
    """Where the taught press tip lands from `pose`, relative to the button: depth past the face along the press
    (positive = into the wall), lateral offset on the wall, and heading error against the planned stance."""
    n = unit(wall_normal)
    tip = from_frame(pose, tip_xy)
    d = (tip[0]-button_xy[0], tip[1]-button_xy[1])
    planned = math.atan2(-n[1], -n[0])-tip_yaw
    return {'depth_m': -(d[0]*n[0]+d[1]*n[1]), 'lateral_m': d[0]*-n[1]+d[1]*n[0], 'yaw_err_rad': wrap(pose[2]-planned)}


def wall_gap(pose, corners, button_xy, wall_normal):
    """Smallest distance in front of the button face plane over the robot outline corners (robot frame) at `pose`;
    negative = a corner is past the face."""
    n = unit(wall_normal)
    gaps = []
    for corner in corners:
        p = from_frame(pose, corner)
        gaps.append((p[0]-button_xy[0])*n[0]+(p[1]-button_xy[1])*n[1])
    return min(gaps)


def pallet_dock(face_mid, face_normal, depth, centre_ahead, standoff=0.0):
    """Pallet centre and the robot pose whose lift carrier sits in the pocket (or `standoff` before it).
    face_normal points from the robot side into the pallet (fork direction)."""
    n = unit(face_normal)
    centre = (face_mid[0]+n[0]*depth/2, face_mid[1]+n[1]*depth/2)
    back = centre_ahead+standoff
    return centre, (centre[0]-n[0]*back, centre[1]-n[1]*back, math.atan2(n[1], n[0]))


def door_axis(door_xy, cabin_xy, outside_m, normal=None, through_m=.55):
    """Pass a door square to it through its centre. Returns the unit door normal into the cabin (`normal`, oriented
    toward the cabin point; door centre -> cabin point if not given), the point `through_m` inside the door on that
    normal (the robot has fully crossed the door line there) and the alignment point `outside_m` outside the door.
    The cabin centre itself is the taught cabin point (reached from the through point inside the cabin).
    (Simulation: driving straight from a taught pose, or obliquely through the door, clipped the door frame.)"""
    to_cabin = (cabin_xy[0]-door_xy[0], cabin_xy[1]-door_xy[1])
    n = unit(normal if normal is not None else to_cabin)
    if n[0]*to_cabin[0]+n[1]*to_cabin[1] < 0:
        n = (-n[0], -n[1])
    through = (door_xy[0]+n[0]*through_m, door_xy[1]+n[1]*through_m)
    outside = (door_xy[0]-n[0]*outside_m, door_xy[1]-n[1]*outside_m)
    return n, through, outside


def rightmost(points, pose):
    """The point most to the right of the robot (smallest 'left' coordinate): nearest the entrance when the robot
    has turned right at the parcel area."""
    return min(points, key=lambda p: to_frame(pose, p)[1])
