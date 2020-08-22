#!/usr/bin/env python

import sys
import rospy, os, json 
from load_waypoints.srv import *
from sensor_msgs.msg import NavSatFix


def load_waypoint_client(x):
    rospy.wait_for_service('load_waypoint')
    try:
        # Create handle for calling the service 
        load_waypoint = rospy.ServiceProxy('load_waypoint', WaypointRequest)

        # This generates the LoadWaypointRequest object, and returns a LoadWaypointResponse object 
        resp = load_waypoint(x)
        return resp
 
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%(e))

def usage():
    return "%s [x]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else: # Incorrect user input 
        rospy.loginfo(usage())
        sys.exit(1)

    # rospy.loginfo("Requesting waypoint #%s"%(x))
    # rospy.loginfo("Waypoint #%s is %s"%(x, load_waypoint_client(x)))

    print("Requesting waypoint #{}".format(x))
    print("Waypoint #{} is {}".format(x, load_waypoint_client(x)))


# Notes:
# Regarding the acceptance criteria for the issue, this issue is not able to do that. 
# Must integrate with the rest of the gps issues. 
# Double check that the other waypoints are given @ competition time. 

# caffeine/gps/