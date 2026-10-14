"""Exact 2D distance between the true rectangular footprint and raster wall boxes."""
import math
import numpy as np

FOOTPRINT = np.array([[.033,.219],[.033,-.219],[-.327,-.219],[-.327,.219]])


def polygon(x, y, yaw, footprint=None):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(FOOTPRINT if footprint is None else footprint) @ np.array([[c,s],[-s,c]]) + [x,y]


def segment_distances(points, starts, ends):
    d = ends-starts
    fraction = np.clip(np.sum((points-starts)*d, axis=-1)/np.sum(d*d, axis=-1), 0, 1)
    return np.linalg.norm(points-starts-fraction[...,None]*d, axis=-1)


def minimum_clearance(boxes, x, y, yaw, footprint=None):
    boxes = np.asarray(boxes)
    # A remote box cannot improve a local minimum < 2 m.
    delta = np.array([x,y])-boxes[:,:2]
    bc, bs = np.cos(boxes[:,4]), np.sin(boxes[:,4])
    local_delta = np.column_stack((bc*delta[:,0]+bs*delta[:,1],-bs*delta[:,0]+bc*delta[:,1]))
    center_distance = np.linalg.norm(np.maximum(np.abs(local_delta)-boxes[:,2:4]/2,0),axis=1)
    # The farthest footprint vertex is < .4 m from the origin. Therefore any
    # wall > .8 m farther from the centre cannot beat the nearest centre wall.
    radius = float(np.linalg.norm(FOOTPRINT if footprint is None else footprint, axis=1).max())
    nearby = boxes[center_distance <= center_distance.min()+2*radius]
    signs = np.array([[-1,-1],[-1,1],[1,1],[1,-1]])
    local = signs[None,:,:]*nearby[:,None,2:4]/2
    c, s = np.cos(nearby[:,4]), np.sin(nearby[:,4])
    walls = np.empty_like(local)
    walls[:,:,0] = nearby[:,None,0]+c[:,None]*local[:,:,0]-s[:,None]*local[:,:,1]
    walls[:,:,1] = nearby[:,None,1]+s[:,None]*local[:,:,0]+c[:,None]*local[:,:,1]
    body = polygon(x,y,yaw,footprint)
    angles = np.unique(nearby[:,4])
    axes = np.array([[math.cos(a),math.sin(a)] for a in [yaw,*angles]] +
                    [[-math.sin(a),math.cos(a)] for a in [yaw,*angles]])
    bp = body @ axes.T
    wp = walls @ axes.T
    overlap = np.all((wp.max(axis=1) >= bp.min(axis=0)) & (wp.min(axis=1) <= bp.max(axis=0)), axis=1)
    if overlap.any():
        return 0.0
    d1 = segment_distances(body[None,:,None,:], walls[:,None,:,:], np.roll(walls,-1,axis=1)[:,None,:,:])
    d2 = segment_distances(walls[:,:,None,:], body[None,None,:,:], np.roll(body,-1,axis=0)[None,None,:,:])
    return float(min(d1.min(),d2.min()))
