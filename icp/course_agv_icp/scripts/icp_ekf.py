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
import cv2
from icp import ICP
from ekf import EKF,STATE_SIZE
# from extraction import Extraction

MAX_LASER_RANGE = 30

class SLAM_EKF():
    def __init__(self):
        # ros param
        self.robot_x = rospy.get_param('/slam/robot_x',0)
        self.robot_y = rospy.get_param('/slam/robot_y',0)
        self.robot_theta = rospy.get_param('/slam/robot_theta',0)

        self.icp = ICP()
        self.ekf = EKF()
        # self.extraction = Extraction()
        self.tf = tf.TransformListener()

        # odom robot init states
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []
        self.map = []

        # State Vector [x y yaw]
        self.xOdom = np.zeros((3, 1))
        self.xEst = np.zeros((3, 1))
        self.PEst = np.zeros((3, 3))
        
        # ros topic
        self.laser_sub = rospy.Subscriber('/scan',LaserScan,self.laserCallback)
        self.map_sub = rospy.Subscriber('/map',OccupancyGrid,self.mapCallback)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.landMark_pub = rospy.Publisher('/landMarks',MarkerArray,queue_size=1)

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/laser", rospy.Time(), rospy.Duration(4.0))
            (trans, rot1) = self.tf.lookupTransform('/map', '/laser', rospy.Time(0))
            # self.tf.waitForTransform("/world_base", "/map", rospy.Time(), rospy.Duration(4.0))
            # (trans2, rot2) = self.tf.lookupTransform('/world_base', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler1 = tf.transformations.euler_from_quaternion(rot1)
        _, _, yaw = euler1[0], euler1[1], euler1[2]
        # matrix1 = np.array(tf.transformations.quaternion_matrix(rot1))
        # matrix2 = np.array(tf.transformations.quaternion_matrix(rot2))
        # matrix_ = matrix2.dot(matrix1)
        # trans = matrix2[:3,:3].dot(np.array(trans1).reshape(3,1)) + np.array(trans2).reshape(3,1)
        # euler = tf.transformations.euler_from_matrix(matrix_, axes='sxyz')
        # _, _, yaw = euler[0], euler[1], euler[2]

        self.x = trans[0]  # trans[0,0]
        self.y = trans[1]  # trans[1,0]
        self.yaw = yaw
        return np.array([[self.x, self.y, self.yaw]]).T

    def laserCallback(self,msg):
        np_msg = self.laserToNumpy(msg)
        # lm = self.extraction.process(np_msg)  # [3, n]
        u = self.calc_odometry(np_msg)
        # z = self.match(lm)
        z = self.updateGlobalPose()
        self.xEst,self.PEst = self.ekf.fusion(self.xEst,self.PEst,z,u)

        self.publishResult()
        pass

    def mapCallback(self,msg):
        self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width).astype(np.uint8)

        # lines = cv2.Canny(self.map,50,150,apertureSize = 3)
        pass

    def match(self, lm):
        pass

    def observation(self,lm):
        landmarks = lm
        z = np.zeros((0, 3))
        for i in range(len(landmarks.id)):
            dx = landmarks.position_x[i]
            dy = landmarks.position_y[i]
            d = math.hypot(dx, dy)
            angle = self.ekf.pi_2_pi(math.atan2(dy, dx))
            zi = np.array([d, angle, i])
            z = np.vstack((z, zi))
        return z

    def calc_odometry(self,np_msg):
        if self.isFirstScan:
            self.tar_pc = np_msg
            self.isFirstScan = False
            return np.array([[0.0,0.0,0.0]]).T
        self.src_pc = np_msg
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = np_msg
        return self.T2u(transform_acc)

    def laserToNumpy(self,msg):
        range_l = np.array(msg.ranges)
        range_l = range_l[~ np.isnan(range_l)]
        # mask = np.sort(np.random.choice(range_l.shape[0], min(300, range_l.shape[0]), replace=False))
        # range_l = range_l[mask]
        total_num = range_l.shape[0]
        pc = np.ones([3,total_num])
        range_l[range_l == np.inf] = MAX_LASER_RANGE
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = np.array([[t[0,2],t[1,2],dw]])
        return u.T
    
    def u2T(self,u):
        w = u[2]
        dx = u[0]
        dy = u[1]

        return np.array([
            [ math.cos(w),-math.sin(w), dx],
            [ math.sin(w), math.cos(w), dy],
            [0,0,1]
        ])

    def lm2pc(self,lm):
        total_num = len(lm.id)
        dy = lm.position_y
        dx = lm.position_x
        range_l = np.hypot(dy,dx)
        angle_l = np.arctan2(dy,dx)
        pc = np.ones((3,total_num))
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def publishResult(self):
        # tf
        s = self.xEst.reshape(-1)
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"ekf_location","map") #TODO
        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map" #TODO

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
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
        odom.header.frame_id = "map" #TODO

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)
        
        pass

def main():
    rospy.init_node('slam_node')
    s = SLAM_EKF()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
