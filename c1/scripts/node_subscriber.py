#!/usr/bin/env python
import rospy
# from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

# def main():
#     rospy.init_node('msg_subscriber',anonymous=True)
#     rospy.Subscriber("test_msg",String,callback)
#     rospy.spin()

from c1.msg import NumWithKey
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "receive : %s %d" % (data.name , data.num))

def main():
    rospy.init_node('msg_subscriber',anonymous=True)
    rospy.Subscriber("test_msg",NumWithKey,callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass