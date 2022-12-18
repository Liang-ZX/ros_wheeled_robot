#!/usr/bin/env python2

import rospy
import geometry_msgs.msg
import std_msgs.msg

class velocity_Publisher:
    def __init__(self, side="left"):
        topic_str = "/course_agv/" + side + "_wheel_velocity_controller/command"
        self.vel_pub = rospy.Publisher(
            topic_str, std_msgs.msg.Float64, queue_size=1)
    def pub(self, velocity):
        if rospy.is_shutdown():
            exit()
        # print("publish command : vx - %.2f vw - %.2f"%(command['vx'],command['vw']))
        self.vel_pub.publish(velocity)

def process(data, args):
    vx = data.linear.x
    vw = data.angular.z
    l = 0.1133 # (0.2+0.08/3) / 2
    vel_left = vx - vw * l / 2.0
    vel_right = vx + vw * l / 2.0
    print("subscribe command: left_vel - %.2f right_vel - %.2f"%(vel_left, vel_right))
    publish_l = args[0]
    publish_r = args[1]
    publish_l.pub(vel_left)
    publish_r.pub(vel_right)

def listener():
    pub_left = velocity_Publisher(side="left")
    pub_right = velocity_Publisher(side="right")
    rospy.Subscriber('/course_agv/velocity', geometry_msgs.msg.Twist, 
        process, (pub_left, pub_right))

def main():
    node_name = "course_agv_kinematics"
    print("node : ",node_name)
    rospy.init_node(node_name,anonymous=False)
    listener()
    rospy.spin()


if __name__ == '__main__':
    main()

