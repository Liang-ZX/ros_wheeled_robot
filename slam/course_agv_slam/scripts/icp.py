#!/usr/bin/env python2

import rospy
import numpy as np
import math

# structure of the nearest neighbor 
class NeighBor:
    def __init__(self):
        self.distances =None
        self.src_indices = None
        self.tar_indices = None

class ICP:
    def __init__(self):
        # max iterations
        self.max_iter = rospy.get_param('/icp/max_iter',10)
        # distance threshold for filter the matching points
        self.dis_th = rospy.get_param('/icp/dis_th',3)
        # tolerance to stop icp
        self.tolerance = rospy.get_param('/icp/tolerance',0)
        # min match
        self.min_match = rospy.get_param('/icp/min_match',2)
    
    # ICP process function
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def process(self,tar_pc,src_pc):
        # tar_pc: last_time [3,n], src_pc: this_time [3,n]
        Transform_acc = np.eye(3)
        prev_error = 0.0
        iter_cnt = 0
        for _ in range(self.max_iter):
            neigh = self.findNearest(src_pc.T, tar_pc.T)
            # filter
            src_pc_xy = ((src_pc.T)[neigh.src_indices]).T
            tar_chorder = ((tar_pc.T)[neigh.tar_indices]).T
            Transform = self.getTransform(src_pc_xy.T, tar_chorder.T)
            Transform_acc = Transform.dot(Transform_acc)
            src_pc = Transform.dot(src_pc)
            mean_error = np.mean(neigh.distances)
            if abs(prev_error - mean_error) < self.tolerance:
                break
            prev_error = mean_error
            iter_cnt = iter_cnt + 1
        print("total_iter: {:d}".format(iter_cnt))
        return Transform_acc[:2,:]

    # find the nearest points & filter
    # return: neighbors of src and tar
    def findNearest(self,src,tar):
        neigh = NeighBor()
        src = np.expand_dims(src, axis=1)
        tar = np.expand_dims(tar, axis=0)
        diff = src[...,:2] - tar[...,:2]
        tmp_dist = np.hypot(diff[...,0], diff[...,1])
        min_dist = np.min(tmp_dist, axis=1)
        mask = min_dist < self.dis_th
        min_index = np.argmin(tmp_dist[mask,:], axis=1)
        src_index = np.arange(src.shape[0])[mask]
        neigh.src_indices = src_index
        neigh.tar_indices = min_index
        neigh.distances = min_dist[mask]
        return neigh     

    # Waiting for Implementation 
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def getTransform(self,src,tar):
        # return 3*3 (last line [0,0,1])
        centroid_src = np.mean(src, axis=0)
        centroid_tar = np.mean(tar, axis=0)
        src_ = (src - centroid_src)[...,:2]
        tar_ = (tar - centroid_tar)[...,:2]

        W = np.dot(src_.T, tar_)  # H
        U, S, Vt = np.linalg.svd(W)    # Vt = V.T
        V = Vt.T
        R = V.dot(U.T)
        t = centroid_tar[:2,np.newaxis] - R.dot(centroid_src[:2,np.newaxis])
        transform = np.hstack([R, t])
        transform = np.vstack((transform, [0,0,1]))
        return transform

    # def calcDist(self,a,b):
    #     dx = a[0] - b[0]
    #     dy = a[1] - b[1]
    #     return math.hypot(dx,dy)
