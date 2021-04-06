#!/usr/bin/env python
import rospy
print("Hello ROS")
print(rospy.get_param('test_param',"default_value"))