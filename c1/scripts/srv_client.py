#!/usr/bin/env python
import sys
import rospy
from c1.srv import *
import time
def usage():
    print("get args num : %d"%len(sys.argv))
    return "%s [x,y]"%sys.argv[0]

def add_two_ints_request(x,y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints',AddTwoInts)
        response = add_two_ints(x,y)
        return response.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
    print("")
    print("%s + %s = %s"%(x, y, add_two_ints_request(x,y)))

if __name__ == '__main__':
    try:
        while True:
            main()
            time.sleep(1)
    except rospy.ROSInterruptException:
        pass