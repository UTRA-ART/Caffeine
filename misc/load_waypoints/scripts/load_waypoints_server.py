#!/usr/bin/env python

import rospy, os, json, sys 
from load_waypoints.srv import *
import rospkg
import std_msgs.msg
import tf
from sensor_msgs.msg import NavSatFix

class load_waypoints:
    def __init__(self, static_waypoint_file, max_time_to_wait):
        self.all_waypoints = dict()
        self.static_waypoint_file = static_waypoint_file
        self.max_time_to_wait = max_time_to_wait 
        self.waited_for_transform = False # Initialize the boolean for whether or waiting has timed out 

        rospy.init_node('load_waypoint_server')
        self.populate_waypoint_dict()

    def populate_waypoint_dict(self):
        '''
        Description: 
            Used to populate the waypoint dictionary with i) the static waypoints obtained during competition time and 
            ii) the first gps coordinate that acts as the final waypoint. 
        '''
        base_dir = rospkg.RosPack().get_path('load_waypoints')

        # Load in static waypoints (provided at competition time) 
        with open(base_dir + '/scripts/'+ self.static_waypoint_file) as f:
            try:
                waypoint_data = json.load(f)
            except:
                rospy.loginfo("Invalid JSON")
                sys.exit(1)

        # Parse through json data and create list of lists holding all waypoints
        for waypoint in waypoint_data["waypoints"]:
            self.all_waypoints[waypoint['id']] = waypoint

        # Call method to wait for transform 
        self.waited_for_transform = self.wait_for_utm_transform()

        # Check if successfully waited for the transform within the time limit. If successful, continue populating the waypoint dict. 
        if self.waited_for_transform:
            # After waiting UTM transform, capture a message from the /gps/fix topic
            gps_info = rospy.wait_for_message('/gps/fix', NavSatFix)

            # Append the starting gps coordinate to the waypoints dict as the final waypoint
            last_coord_idx = len(self.all_waypoints) 
            self.all_waypoints[last_coord_idx] = {'id': last_coord_idx,'longitude':gps_info.longitude, 'latitude':gps_info.latitude, 'description': 'Initial start location'}

            # Show waypoints 
            rospy.loginfo("Loaded waypoints dictionary:\n %s", self.all_waypoints)
        else:
            rospy.loginfo("Waiting for transform from /map to /utm timed out!")
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
            time_waited = rospy.get_time() - start_time
            if (time_waited) >= self.max_time_to_wait:
                rospy.loginfo("Waiting for transform timed out. Time waited for transform: %s s"%(time_waited))
                waited_for_transform = False
                break
            else:
                try:
                    now = rospy.Time.now()
                    # Wait for transform from /caffeine/map to /utm
                    listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(5.0))
                    rospy.loginfo("Transform found. Time waited for transform: %s s"%(rospy.get_time() - start_time))
                    waited_for_transform = True
                    break

                except:
                    pass

            rate.sleep()
        return waited_for_transform

    def load_waypoint_server(self):
        '''
        Description: 
            Used to declare a service called 'load_waypoint'. 
        '''
        rospy.Service('load_waypoint', WaypointRequest, self.handle_waypoint_request)

        # Check if we successfully waited for the transform. If not, the node shuts down. 
        if (not self.waited_for_transform):
            rospy.loginfo("Populating waypoint failed.")
        else:
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
        if (waypoint_request.waypoint_number not in self.all_waypoints.keys()):
            waypoint_coord = None 
            valid_request_flag = False
            request_description = "Request error: Invalid waypoint number"
            waypoint_description = ""

        # Handle valid requests
        else:
            waypoint_coord = [float(self.all_waypoints[waypoint_request.waypoint_number]['longitude']), float(self.all_waypoints[waypoint_request.waypoint_number]['latitude'])]
            valid_request_flag = True
            request_description = "Request Successful"
            waypoint_description = self.all_waypoints[waypoint_request.waypoint_number]['description']

        # Determine the next valid waypoint number 
        if (waypoint_request.waypoint_number not in self.all_waypoints.keys()) or (waypoint_request.waypoint_number == (len(self.all_waypoints) - 1)):
            next_waypoint_number = -1 
        else:
            next_waypoint_number = waypoint_request.waypoint_number + 1
            
        return WaypointRequestResponse(waypoint_coord, valid_request_flag, request_description, waypoint_description, next_waypoint_number)

if __name__ == "__main__":
    static_waypoint_file = 'static_waypoints.json' # File name for static waypoints (provided at competition-time) 
    max_time_to_wait = 60.0 # Maximum time to wait for the transform. The node times out and shut down if this limit is exceeded.

    LoadWaypoints = load_waypoints(static_waypoint_file, max_time_to_wait)
    LoadWaypoints.load_waypoint_server() # Start service node 
