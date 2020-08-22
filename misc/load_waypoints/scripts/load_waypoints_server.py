#!/usr/bin/env python

import rospy, os, json, sys 
from load_waypoints.srv import *
import rospkg
import std_msgs.msg, tf
from sensor_msgs.msg import NavSatFix
# import sensor_msgs.msg

all_waypoints = list()

def populate_waypoint_table():
    # gps_info = rospy.wait_for_message('gps_ready', std_msgs.msg.Bool)
    # print(gps_info.data)
    # print(gps_info)
    # while gps_info.data == False:
    #     continue
    # gps_info = rospy.wait_for_message('gps_ready', std_msgs.msg.Bool)
    # while True:
    #     # rospy.loginfo(gps_info, 'service')
    #     if gps_info.data == False:
    #         continue
    #     elif gps_info.data == True:
    #         break

    #     gps_info = rospy.wait_for_message('gps_ready', std_msgs.msg.Bool)

    # rospy.sleep(5)

    start_time = rospy.get_time()

    # Initialize transform listener 
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    rospy.sleep(1) # This prevents the waitfortransform to error out 
    
    # Wait for transform from /caffeine/map to /utm
    listener.waitForTransform("/caffeine/map", "/utm", rospy.Time(), rospy.Duration(50.0))

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(50.0))
            print(rospy.get_time() - start_time)
            break

        except (tf.LookupException, tf.ConnectivityException):
            continue

        rate.sleep()

    # After waiting UTM transform, we can wait for a message from the gps/fix topic
    gps_info = rospy.wait_for_message('/caffeine/gps/fix', NavSatFix)
    print(gps_info.latitude, gps_info.longitude)

    # Add the first coords to the waypoints list 
    all_waypoints.append([gps_info.longitude, gps_info.latitude])

    # rospy.sleep(5)

    base_dir = rospkg.RosPack().get_path('load_waypoints')
    with open(base_dir + '/scripts/waypoints.json') as f:
        try:
            waypoint_data = json.load(f)
        except:
            rospy.loginfo("Invalid JSON")
            sys.exit(1)

    # Get first waypoint 
    ## Need to allow enough time for get_gps_msgs to finish 
    # with open(base_dir + '/scripts/first_gps_coords.json') as f:
    #     try:
    #         first_coords = json.load(f)
    #     except:
    #         rospy.loginfo("Invalid JSON")
    #         sys.exit(1)

    # Parse through dictionary and create list of lists holding all waypoints
    for waypoint in waypoint_data["waypoints"]:
        all_waypoints.append([waypoint['longitude'], waypoint['latitude']])

    # Add the initial waypoint as the final waypoint 
    # for first_coord in first_coords["waypoints"]:
        # all_waypoints.append([first_coord['longitude'], first_coord['latitude']])
    all_waypoints.append([gps_info.longitude, gps_info.latitude])

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
    
# Current issues 
# The returning request waypoint is printing/getting called twice ? 
# The time doesn't look right 