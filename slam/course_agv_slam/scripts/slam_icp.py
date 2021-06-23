#!/usr/bin/env python2
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray,Marker
from nav_msgs.msg import OccupancyGrid
import numpy as np
from icp import ICP
from mapping import Mapping
import sys

MAX_LASER_RANGE = 30
STATE_SIZE = 3

class SLAM_ICP():
    def __init__(self):
        # ros param
        self.robot_x = rospy.get_param('/slam/robot_x',0)
        self.robot_y = rospy.get_param('/slam/robot_y',0)
        self.robot_theta = rospy.get_param('/slam/robot_theta',0)
        ## ros param of mapping
        self.map_x_width = rospy.get_param('/slam/map_width')
        self.map_y_width = rospy.get_param('/slam/map_height')
        self.map_reso = rospy.get_param('/slam/map_resolution')
        self.map_cellx_width = int(round(self.map_x_width/self.map_reso))
        self.map_celly_width = int(round(self.map_y_width/self.map_reso))

        # odom robot init states
#        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []

        self.icp = ICP()
        self.mapping = Mapping(self.map_cellx_width,self.map_celly_width,self.map_reso)

        # State [cos(yaw) -sin(yaw) x]
        #       [sin(yaw)  cos(yaw) y]
        #       [0          0       1]
        self.xEst = np.eye(STATE_SIZE)
        self.xOdom = np.zeros((STATE_SIZE, 1))
        
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10

        # ros topic
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laserCallback)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.location_pub = rospy.Publisher('icp_location',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.map_pub = rospy.Publisher('/slam_map',OccupancyGrid,queue_size=1)

    def laserCallback(self,msg):
        np_msg = self.laserToNumpy(msg)
        #icp implement in self.calc_odometry. Return the robot pose in world frame.
        self.xEst = self.calc_odometry(np_msg)
        #transform the laser points from the laser frame into the world frame
        obs = self.xEst.dot(np_msg)
        pmap = self.mapping.update(obs[0], obs[1], self.xEst[0,2], self.xEst[1,2])

        self.publishMap(pmap)
        #self.publishLandMark(lm)
        self.publishResult()
        pass


    def calc_odometry(self,np_msg):
        if self.isFirstScan:
            self.tar_pc = np_msg
            self.isFirstScan = False
            return np.eye(STATE_SIZE)
        self.src_pc = np_msg
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        return transform_acc

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        range_l[range_l == np.inf] = MAX_LASER_RANGE
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc


    def publishResult(self):
        # tf
        s = math.atan2(self.xEst[1,0],self.xEst[0,0])
        q = tf.transformations.quaternion_from_euler(0,0,s)
        self.odom_broadcaster.sendTransform((self.xEst[0,2],self.xEst[1,2],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"icp_location","world_base")
        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = self.xEst[0,2]
        odom.pose.pose.position.y = self.xEst[1,2]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.location_pub.publish(odom)

        s = self.xOdom
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)
        
        pass


    def publishMap(self,pmap):
        map_msg = OccupancyGrid()
        map_msg.header.seq = 1
        map_msg.header.stamp = rospy.Time().now()
        map_msg.header.frame_id = "world_base"

        map_msg.info.map_load_time = rospy.Time().now()
        map_msg.info.resolution = self.map_reso
        map_msg.info.width = self.map_cellx_width
        map_msg.info.height = self.map_celly_width
        map_msg.info.origin.position.x = -self.map_cellx_width*self.map_reso/2.0
        map_msg.info.origin.position.y = -self.map_celly_width*self.map_reso/2.0
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = list(pmap.T.reshape(-1)*100)
        
        self.map_pub.publish(map_msg)

def main():
    rospy.init_node('slam_node')
    s = SLAM_ICP()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
