#!/usr/bin/env python

import rospy, os, json 
from load_waypoints.srv import *

all_waypoints = list()

def populate_waypoint_table():
    base_dir = rospy.get_param('~arg_name') # ~ added to arg_name because private param 
    
    with open(base_dir + '/scripts/waypoints.json') as f:
        try:
            waypoint_data = json.load(f)
        except:
            rospy.loginfo("Invalid JSON")
            sys.exit(1)

    # Parse through dictionary and create list of lists holding all waypoints
    for waypoint in waypoint_data["waypoints"]:
        all_waypoints.append([waypoint['coordinate 1'], waypoint['coordinate 2']])

    # Add the initial waypoint as the final waypoint 
    all_waypoints.append([waypoint_data["waypoints"][0]['coordinate 1'], waypoint_data["waypoints"][0]['coordinate 2']])

    rospy.loginfo("Loaded waypoints: %s", all_waypoints)
    return 

def handle_waypoint_request(waypoint_request):
    rospy.loginfo("Returning request for waypoint #%s "%(waypoint_request.waypoint_number))
    if (waypoint_request.waypoint_number > (len(all_waypoints) - 1)) or (waypoint_request.waypoint_number < 0):
        waypoint = None 
        valid_request_flag = False
        description = "Request error: Invalid waypoint number"
        rospy.loginfo(description)  # Print in server terminal 
    else:
        waypoint = all_waypoints[waypoint_request.waypoint_number]
        valid_request_flag = True
        description = ""
    return WaypointRequestResponse(waypoint, valid_request_flag, description)

def load_waypoint_server():
    # Must init node before reading any files
    rospy.init_node('load_waypoint_server')
    populate_waypoint_table()
    rospy.Service('load_waypoint', WaypointRequest, handle_waypoint_request)
    rospy.loginfo("Ready to load waypoints.")
    rospy.spin() # Keeps code from exiting until the service is shutdown

if __name__ == "__main__":
    load_waypoint_server()
    