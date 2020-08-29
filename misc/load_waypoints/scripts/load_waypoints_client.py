#!/usr/bin/env python

########## TO DO ############
import sys
import rospy, os, json 
from load_waypoints.srv import *

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

    waypoint_info = load_waypoint_client(x)

    print("Requesting waypoint #%s"%(x))
    print("Waypoint #%s is %s"%(x, waypoint_info))