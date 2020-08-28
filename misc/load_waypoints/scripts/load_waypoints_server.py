#!/usr/bin/env python

import rospy, os, json, sys 
from load_waypoints.srv import *
import rospkg
import std_msgs.msg
import tf
from sensor_msgs.msg import NavSatFix

class load_waypoints:
    def __init__(self):
        self.all_waypoints = dict()

    def populate_waypoint_dict(self):
        '''
        Description: 
            Used to populate the waypoint dictionary with i) the static waypoints obtained during competition time and 
            ii) the first gps coordinate that acts as the final waypoint. 
        '''
        base_dir = rospkg.RosPack().get_path('load_waypoints')
        
        # Load in static waypoints (provided at competition time) 
        with open(base_dir + '/scripts/static_waypoints.json') as f:
            try:
                waypoint_data = json.load(f)
            except:
                rospy.loginfo("Invalid JSON")
                sys.exit(1)

        # Parse through json data and create list of lists holding all waypoints
        for waypoint in waypoint_data["waypoints"]:
            self.all_waypoints[waypoint['id']] = waypoint

        # Call method to wait for transform 
        self.wait_for_utm_transform()

        # After waiting UTM transform, capture a message from the /gps/fix topic
        gps_info = rospy.wait_for_message('/gps/fix', NavSatFix)

        # Append the starting gps coordinate to the waypoints dict as the final waypoint
        last_coord_idx = len(self.all_waypoints) 
        self.all_waypoints[last_coord_idx] = {'id': last_coord_idx,'longitude':gps_info.longitude, 'latitude':gps_info.latitude, 'description': 'Initial start location'}

        # Show waypoints 
        rospy.loginfo("Loaded waypoints dictionary:\n %s", self.all_waypoints)
        return 

    def wait_for_utm_transform(self):
        '''
        Description: 
            Used to wait for a transform from the /caffeine/map frame to /utm frame (which indicates that the GPS is ready). This accounts/simulates for gps start-up time. 
            Once the transform is detected, this function will exit. 
        '''        
        # Initialize transform listener 
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)
        
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                # Wait for transform from /caffeine/map to /utm
                listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(4.0))
                rospy.loginfo("Time waited for transform: %s s"%(rospy.get_time() - start_time))
                break

            except:
                pass

            rate.sleep()
        return 

    def load_waypoint_server(self):
        '''
        Description: 
            Used to declare a service called 'load_waypoint'. 
        '''
        rospy.Service('load_waypoint', WaypointRequest, self.handle_waypoint_request)
        rospy.loginfo("Ready to load waypoints.")
        rospy.spin() # Keeps code from exiting until the service is shutdown

    def handle_waypoint_request(self,waypoint_request):
        '''
        Description: 
            Used to return waypoint coordinate in the form of ['longitude', 'latitude'] and other information about the waypoint and request.
            This is called up a request to the 'load_waypoint_server' node. 
        '''

        rospy.loginfo("Returning request for waypoint #%s "%(waypoint_request.waypoint_number))

        # Handle invalid requests 
        if (waypoint_request.waypoint_number > (len(self.all_waypoints) - 1)) or (waypoint_request.waypoint_number < 0):
            waypoint_coord = None 
            valid_request_flag = False
            request_description = "Request error: Invalid waypoint number"
            waypoint_description = ""

        # Handle valid requests
        else:
            waypoint_coord = [float(self.all_waypoints[waypoint_request.waypoint_number]['longitude']), float(self.all_waypoints[waypoint_request.waypoint_number]['latitude'])]
            valid_request_flag = True
            request_description = "Request Successful"
            waypoint_description = str(self.all_waypoints[waypoint_request.waypoint_number]['description'])

        # Determine the next valid waypoint number 
        if (waypoint_request.waypoint_number >= len(self.all_waypoints) - 1) or (waypoint_request.waypoint_number < 0):
            next_waypoint_number = -1 
        else:
            next_waypoint_number = waypoint_request.waypoint_number + 1
            
        return WaypointRequestResponse(next_waypoint_number, waypoint_coord, valid_request_flag, request_description, waypoint_description)

if __name__ == "__main__":
    LoadWaypoints = load_waypoints()

    rospy.init_node('load_waypoint_server')
    LoadWaypoints.populate_waypoint_dict()

    LoadWaypoints.load_waypoint_server()
