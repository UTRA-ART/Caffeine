#!/usr/bin/env python

import rospy, os, json, sys 
from load_waypoints.srv import *
import os, rospkg

all_waypoints = list()

def populate_waypoint_table():
    # latitude, longitude = get_gps_msgs()
    # base_dir = rospy.get_param('~arg_name') # ~ added to arg_name because private param 
    # base_dir = rospy.get_param('waypoint_dir') # ~ added to arg_name because private param 
    rospack = rospkg.RosPack()
    base_dir = rospack.get_path('load_waypoints')
    with open(base_dir + '/scripts/waypoints.json') as f:
        try:
            waypoint_data = json.load(f)
        except:
            rospy.loginfo("Invalid JSON")
            sys.exit(1)

    # Get first waypoints 
    with open(base_dir + '/scripts/sample_waypoints.json') as f:
        try:
            first_coords = json.load(f)
        except:
            rospy.loginfo("Invalid JSON")
            sys.exit(1)

    # Parse through dictionary and create list of lists holding all waypoints
    for first_coord in first_coords["waypoints"]:
        all_waypoints.append([first_coord['coordinate 1'], first_coord['coordinate 2']])

    for waypoint in waypoint_data["waypoints"]:
        all_waypoints.append([waypoint['coordinate 1'], waypoint['coordinate 2']])

    # Add the initial waypoint as the final waypoint 
    # all_waypoints.append([waypoint_data["waypoints"][0]['coordinate 1'], waypoint_data["waypoints"][0]['coordinate 2']])
    for first_coord in first_coords["waypoints"]:
        all_waypoints.append([first_coord['coordinate 1'], first_coord['coordinate 2']])

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
    