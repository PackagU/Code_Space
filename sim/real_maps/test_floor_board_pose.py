#!/usr/bin/env python3
"""Camera board geometry must overcome a biased stair-wall estimate without selecting another cap."""
import sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/jetson_skill_tests'))
from skilltests import vision_core as vc

layout=[['1',-.049,.04],['2',.049,.04],['3',-.049,-.04],['4',.049,-.04]]
fx=vc.fx_from_hfov(320,1.047)
pose=(np.zeros(3), np.eye(3))
# Camera tilted to the board normal; the jagged wall normal is deliberately biased.
rotation,_=cv2.Rodrigues(np.array([.03,.08,0.]))
translation=np.array([0.,0.,.7])
objects=np.array([[r,-up,0.] for _,r,up in layout])
optical=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
points=(objects@rotation.T+translation)@optical.T
normal=-optical@rotation[:,2]
n=normal[:2]/np.linalg.norm(normal[:2])
caps=[]
for p in points:
 u,v=vc.project(p,(320,240),fx,pose)
 caps.append({'u':u,'v':v,'area':100,'bbox':[int(u)-5,int(v)-5,10,10],'board_bbox':[100,70,110,100]})
biased_n=vc.rot_z(.04)[:2,:2]@n
prior=points[1]+np.array([.015,.03,0.])
solved=vc.floor_board_pose(caps,layout,'2',(320,240),fx,pose,prior,biased_n)
assert solved is not None
assert np.linalg.norm(np.asarray(solved['xyz'])-points[1])<1e-5
assert np.linalg.norm(np.asarray(solved['normal'])-n)<1e-5
assert vc.floor_board_pose(caps[:3],layout,'2',(320,240),fx,pose,prior,biased_n) is None
assert vc.floor_board_pose(caps,layout,'2',(320,240),fx,pose,prior+np.array([1.,0.,0.]),biased_n) is None
print('camera board pose / incomplete board / LiDAR disagreement PASS')
