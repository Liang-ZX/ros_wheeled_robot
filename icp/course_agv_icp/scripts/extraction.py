#!/usr/bin/env python2
from sensor_msgs.msg import LaserScan
import numpy as np
import rospy
class LandMarkSet():
    def __init__(self):
        self.position_x = []
        self.position_y = []
        self.id = []

class Extraction():
    def __init__(self):
        self.range_threshold = rospy.get_param('/extraction/range_threshold',0.7)
        self.radius_max_th = rospy.get_param('/extraction/radius_max_th',0.3)
        self.landMark_min_pt = rospy.get_param('/extraction/landMark_min_pt',1)

    # todo ...
    def process(self,msg):
    
        # ...
        # ...
        pass

    # todo ...
    def extractLandMark(self,msg):
        # ...
        # ...
        pass
