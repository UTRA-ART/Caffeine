#!/usr/bin/env python

import rospy, os, json, sys 
from load_waypoints.srv import *
import rospkg
import std_msgs.msg, tf
from sensor_msgs.msg import NavSatFix

all_waypoints = list()

def populate_waypoint_table():
    start_time = rospy.get_time()

    # Initialize transform listener 
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    rospy.sleep(1) # This prevents the waitfortransform to error out 
    
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            # Wait for transform from /caffeine/map to /utm
            listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(50.0))
            print(rospy.get_time() - start_time)
            break

        except (tf.LookupException, tf.ConnectivityException):
            continue

        rate.sleep()

    # After waiting UTM transform, we can wait for a message from the gps/fix topic
    gps_info = rospy.wait_for_message('/caffeine/gps/fix', NavSatFix)

    # Add the first coords to the waypoints list 
    all_waypoints.append([gps_info.longitude, gps_info.latitude])

    base_dir = rospkg.RosPack().get_path('load_waypoints')
    
    # Load in static (IGVC-provided) waypoints
    with open(base_dir + '/scripts/waypoints.json') as f:
        try:
            waypoint_data = json.load(f)
        except:
            rospy.loginfo("Invalid JSON")
            sys.exit(1)

    # Parse through json data and create list of lists holding all waypoints
    for waypoint in waypoint_data["waypoints"]:
        all_waypoints.append([waypoint['longitude'], waypoint['latitude']])

    # Append the initial gps coordinate as the final waypoint 
    all_waypoints.append([gps_info.longitude, gps_info.latitude])

    # Show waypoints 
    rospy.loginfo("Loaded waypoints: %s", all_waypoints)
    return 

def handle_waypoint_request(waypoint_request):
    rospy.loginfo("Returning request for waypoint #%s "%(waypoint_request.waypoint_number))
    
    # Handle invalid requests 
    if (waypoint_request.waypoint_number > (len(all_waypoints) - 1)) or (waypoint_request.waypoint_number < 0):
        waypoint = None 
        valid_request_flag = False
        description = "Request error: Invalid waypoint number"
        rospy.loginfo(description)  
    else:
        waypoint = all_waypoints[waypoint_request.waypoint_number]
        valid_request_flag = True
        description = "Request Successful"
    return WaypointRequestResponse(waypoint, valid_request_flag, description)

def load_waypoint_server():
    rospy.init_node('load_waypoint_server')
    populate_waypoint_table()
    rospy.Service('load_waypoint', WaypointRequest, handle_waypoint_request)
    rospy.loginfo("Ready to load waypoints.")
    rospy.spin() # Keeps code from exiting until the service is shutdown

if __name__ == "__main__":
    load_waypoint_server()
    
# Current issues :
# The returning request waypoint is printing/getting called twice ? 
# The time doesn't look right 
# First waypoint missing id and description
# The waypoints list isn't descriptive 