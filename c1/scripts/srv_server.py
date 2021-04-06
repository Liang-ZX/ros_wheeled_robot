#!/usr/bin/env python
import rospy
from c1.srv import AddTwoInts,AddTwoIntsResponse

def handle_request(req):
    res = req.a+req.b
    print("server: %s + %s = %s"%(req.a,req.b,res))
    return AddTwoIntsResponse(res)

def main():
    rospy.init_node('server_add_two_ints')
    s = rospy.Service("add_two_ints",AddTwoInts, handle_request)
    rospy.spin()
    pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass