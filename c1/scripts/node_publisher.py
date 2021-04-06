#!/usr/bin/env python
import rospy
# from std_msgs.msg import String

# def main():
#     pub = rospy.Publisher('test_msg',String,queue_size = 10)
#     rospy.init_node('msg_publisher',anonymous=True)
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

from c1.msg import NumWithKey

def main():
    pub = rospy.Publisher('test_msg',NumWithKey,queue_size = 10)
    rospy.init_node('msg_publisher',anonymous=True)
    rate = rospy.Rate(10)
    count = 0
    param_string = rospy.get_param("/test_param_server/key_of_str")
    while not rospy.is_shutdown():
        num_with_key = NumWithKey()
        num_with_key.name = param_string
        num_with_key.num = count
        rospy.loginfo(num_with_key)
        pub.publish(num_with_key)
        rate.sleep()
        count = count + 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass