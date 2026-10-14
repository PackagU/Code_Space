"""Robot-side perception for --arm-sim (simulation only). Uses ONLY what the robot has: the arm-tip camera image
(OpenCV colour segmentation), its intrinsics, joint states, encoder odometry (/odom) and the LiDAR scan.
Ground truth is never read here; run_e2e.py compares the estimates with ground truth afterwards (evaluation).

Colour classes are sim conventions ([가정값] world_params perception.hsv): cardboard parcel, green plastic pallet,
blue LED-ring elevator buttons, red 7-segment floor digits.
"""
import math

import sys
from pathlib import Path

import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import arm_geometry as ag

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/jetson_skill_tests'))
from skilltests import vision_core as vc   # noqa: E402


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Perception:
    def __init__(self, node, params):
        self.node, self.p = node, params['perception']
        self.arm = params['arm']
        self.lidar_xy = params['lidar']['pose_xyz'][:2]
        cam = self.arm['camera']
        self.w, self.h = int(cam['width']), int(cam['height'])
        self.fx = vc.fx_from_hfov(self.w, float(cam['hfov_rad']))
        self.bridge = CvBridge()
        self.image, self.image_t = None, None
        node.create_subscription(Image, '/arm_camera/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        self.image, self.image_t = msg, msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    # ------------------------------------------------------------ robot-available poses
    def odom_pose(self):
        """Robot base (x, y, z, yaw) in the odom frame from encoder odometry."""
        p = self.node.odom.pose.pose
        return (p.position.x, p.position.y, 0.0, yaw_of(p.orientation))

    def joints(self):
        return {n: self.node.joints.get(n) for n in ag.names(self.arm)}

    def camera_pose(self, base=None):
        _, _, cp, cR = ag.fk(self.arm, self.joints(), base or self.odom_pose())
        return cp, cR

    # ------------------------------------------------------------ image
    def frame(self, settle=0.6, timeout=15.0):
        """A camera frame rendered after the arm settled (sim time), with the camera pose (odom) at that time."""
        self.node.sim_pause(settle)
        after = self.node.now()
        self.node.wait(lambda: self.image_t is not None and self.image_t >= after, timeout)
        if self.image is None or self.image_t is None or self.image_t < after:
            return None, None
        bgr = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
        return bgr, self.camera_pose()

    # Image processing and projection are the shared skilltests.vision_core functions (same code as on the robot).
    def size(self):
        return (self.w, self.h)

    def mask(self, bgr, cls):
        return vc.mask(bgr, self.p['hsv'][cls])

    def blobs(self, bgr, cls, min_area=None):
        return vc.blobs(bgr, self.p['hsv'][cls], int(self.p['min_blob_px']) if min_area is None else min_area)

    def ray(self, u, v, pose):
        return vc.ray(u, v, self.size(), self.fx, pose)

    def on_plane_z(self, u, v, pose, z):
        return vc.on_plane_z(u, v, self.size(), self.fx, pose, z)

    def on_wall(self, u, v, pose, wall):
        return vc.on_wall(u, v, self.size(), self.fx, pose, wall)

    # ------------------------------------------------------------ LiDAR wall
    def wall_line(self, bearing, half_width=math.radians(20), max_range=2.5):
        """Fit a line to LiDAR points around `bearing` (robot frame) -> (point, unit normal toward the robot) in odom."""
        scan = self.node.scan
        if scan is None:
            return None
        base = self.odom_pose()
        lx = self.lidar_xy
        pts = []
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min+i*scan.angle_increment
            if not (scan.range_min < r < min(scan.range_max, max_range)):
                continue
            if abs(math.atan2(math.sin(a-bearing), math.cos(a-bearing))) > half_width:
                continue
            x, y = lx[0]+r*math.cos(a), lx[1]+r*math.sin(a)
            c, s = math.cos(base[3]), math.sin(base[3])
            pts.append((base[0]+c*x-s*y, base[1]+s*x+c*y))
        if len(pts) < 8:
            return None
        pts = np.array(pts)
        centre = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts-centre)
        n = vt[1]/np.linalg.norm(vt[1])
        if n@(np.array(base[:2])-centre) < 0:
            n = -n
        resid = float(np.abs((pts-centre)@n).max())
        return centre, n, {'points': len(pts), 'max_residual_m': round(resid, 4)}

    # ------------------------------------------------------------ detectors
    def parcels(self, bgr, pose):
        return vc.parcels(bgr, self.p['hsv']['parcel'], int(self.p['min_blob_px']), self.fx, pose,
                          float(self.p['parcel_mid_z_m']))

    def pallet_face(self, bgr, pose, near_uv=None):
        return vc.pallet_face(bgr, self.p['hsv']['pallet'], int(self.p['min_blob_px']), self.fx, pose, near_uv)

    def buttons(self, bgr):
        return vc.button_caps(bgr, self.p['hsv'], int(self.p['board_min_px']), int(self.p['button_min_px']))

    def digit(self, bgr):
        return vc.seven_segment(bgr, self.p['hsv']['digit'], int(self.p['digit_min_px']),
                                float(self.p['digit_one_max_aspect']))
