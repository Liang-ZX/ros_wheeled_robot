#!/usr/bin/env python2

import rospy
import numpy as np
import math

# structure of the nearest neighbor 
class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

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
    # Waiting for Implementation 
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def process(self,tar_pc,src_pc):
        # ...
        # ... 
        pass

    # find the nearest points & filter
    # return: neighbors of src and tar
    def findNearest(self,src,tar):
        # ...
        # ... 
        pass

    # Waiting for Implementation 
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def getTransform(self,src,tar):
        # ...
        # ... 
        pass

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)
