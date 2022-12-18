#!/usr/bin/env python2
import rospy
import tf
import sys
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
# from course_agv_nav.srv import Plan, PlanResponse
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import matplotlib.pyplot as plt
from my_planner import Planner

import time


class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
        self.plan_grid_size = 0.5  # 0.3
        self.plan_robot_radius = 0.6
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []

        # count to update map
        self.map_count = 0
        self.initFlag = False
        self.tf = tf.TransformListener()
        self.goal_sub = rospy.Subscriber('/course_agv/goal',PoseStamped,self.goalCallback)
        # self.plan_srv = rospy.Service('/course_agv/global_plan',Plan,self.replan)
        self.path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
        self.map_sub = rospy.Subscriber('/slam_map',OccupancyGrid,self.mapCallback)
        self.updateMap()
        self.initPlanner()
        # self.updateGlobalPose()
        print("global planner init well")
        pass

    def goalCallback(self,msg):
        self.initPlanner()
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        # print("get new goal!!! ",self.plan_goal)
        self.replan()
        pass

    def collisionCallback(self,msg):
        self.replan()

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]

    def replan(self,req=None):
        print('get request for replan!!!!!!!!')
        self.updateGlobalPose()

        # self.planner.update_planner(self.map)
        # get result
        start_t = time.clock()
        plan_rx, plan_ry = self.planner.planning(self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        end_t = time.clock()
        print "It takes", end_t-start_t, "seconds to find the path."
        if plan_rx is not None:
            self.plan_rx = (np.array(plan_rx) * self.map.info.resolution + self.map.info.origin.position.x).tolist()
            self.plan_ry = (np.array(plan_ry) * self.map.info.resolution + self.map.info.origin.position.y).tolist()
            
            self.publishPath()
            # res = True
            # return PlanResponse(res)

    def initPlanner(self):
        if self.initFlag:
            return
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        # ox,oy = np.nonzero(map_data > 50)
        # self.plan_ox = (ox*self.map.info.resolution+self.map.info.origin.position.x).tolist()
        # self.plan_oy = (oy*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        self.planner = Planner(map_data, self.map.info, self.plan_grid_size, self.plan_robot_radius)
        self.initFlag = True

    def mapCallback(self,msg):
        self.map = msg
        pass
    
    def updateMap(self):
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            msg = getMap().map
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        # Update for planning algorithm
        self.mapCallback(msg)

    def publishPath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[len(self.plan_rx)-1-i]
            pose.pose.position.y = self.plan_ry[len(self.plan_ry)-1-i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    print("PLease wait for global planner init...")
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
