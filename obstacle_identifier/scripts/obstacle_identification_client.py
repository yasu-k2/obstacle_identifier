#!/usr/bin/env python

import sys
import rospy
from obstacle_identifier.srv import IdentifyObstacle

def obstacle_identification_client(mode):
    rospy.wait_for_service('obstacle_identification')
    try:
        obstacle_identification = rospy.ServiceProxy('obstacle_identification', IdentifyObstacle)
        resp = obstacle_identification(mode)
        return resp.case
    except rospy.ServiceException, e:
        print "fail: %s"%e

def usage():
    return "%s [mode]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        mode = 1
    else:
        print usage()
        sys.exit(1)
    print "case: %s"%(obstacle_identification_client(mode))
