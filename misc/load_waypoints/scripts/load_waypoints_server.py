#!/usr/bin/env python

import rospy, os, tf, json, sys 
from load_waypoints.srv import *
import rospkg
import std_msgs.msg
from sensor_msgs.msg import NavSatFix

all_waypoints = list()

def populate_waypoint_table():

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

    wait_for_utm_transform()

    # After waiting UTM transform, we can wait for a message from the gps/fix topic
    gps_info = rospy.wait_for_message('/caffeine/gps/fix', NavSatFix)
    
    # Append the initial gps coordinate as the final waypoint 
    all_waypoints.append([gps_info.longitude, gps_info.latitude])

    # Show waypoints 
    rospy.loginfo("Loaded waypoints: %s", all_waypoints)
    return 

def wait_for_utm_transform():
    # Initialize transform listener 
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    rospy.sleep(1) # This prevents the waitfortransform to error out 
    
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            # Wait for transform from /caffeine/map to /utm
            listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(4.0))
            rospy.loginfo(rospy.get_time() - start_time)
            # end_time = rospy.Time.now()
            break

        # except (tf.LookupException, tf.ConnectivityException):
        except Exception as e:
            pass

        rate.sleep()

def handle_waypoint_request(waypoint_request):
    rospy.loginfo("Returning request for waypoint #%s "%(waypoint_request.waypoint_number))
    rospy.sleep(1)
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
# The returning request waypoint is printing/getting called twice ? --> sending request before getting response (not blocking)
# The time doesn't look right --> do more checks

# load json @ beginning instead of end --> keep a list of dicts or just dicts 
# add id (len of list) + description if u want 
# make a class --> make waypoints an attribute --> put the handle as a method 

# First waypoint missing id and description
# The waypoints list isn't descriptive 