#!/usr/bin/env python

import json
import os
import sys

import actionlib
import rospkg
import rospy
import std_msgs.msg
import tf
import tf2_ros
import utm
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix
from tf import TransformListener
from tf.transformations import quaternion_from_euler


class NavigateWaypoints:
    def __init__(self, static_waypoint_file, max_time_for_transform):
        self.waypoints = dict() 
        self.static_waypoint_file = static_waypoint_file
        self.max_time_for_transform = max_time_for_transform
        self.waited_for_transform = False # Initialize the boolean for whether or waiting has timed out 

        self.populate_waypoint_dict() 
        self.curr_waypoint_idx = 0 
        self.tf = TransformListener()


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
            self.waypoints[waypoint['id']] = waypoint
    
        # Call method to wait for transform 
        self.waited_for_transform = self.wait_for_utm_transform()

        # Check if successfully waited for the transform within the time limit. If successful, continue populating the waypoint dict. 
        if self.waited_for_transform:
            # After waiting UTM transform, capture a message from the gps/fix topic
            gps_info = rospy.wait_for_message('gps/fix', NavSatFix)
            # Append the starting gps coordinate to the waypoints dict as the final waypoint
            last_coord_idx = len(self.waypoints) 
 
            # If waypoints are reported in longitudes and latitudes, append a final waypoint to return to the start
            if self.waypoints[0].get("longitude"):
                self.waypoints[last_coord_idx] = {
                    'id': last_coord_idx, 'longitude': gps_info.longitude, 'latitude': gps_info.latitude, 'description': 'Initial start location', 'frame_id': 'odom'}

            # Show waypoints 
            rospy.loginfo("Successfully loaded waypoints dict")
        else:
            rospy.loginfo("Waiting for transform from /map to /utm timed out!")
        return 

    def wait_for_utm_transform(self):
        '''
        Description: 
            Used to wait for a transform from the /map frame to /utm frame (which indicates that the GPS is ready). This accounts/simulates for gps start-up time. 
            Once the transform is detected, this function will exit. 
        '''        
        # Initialize transform listener 
        listener = tf.TransformListener()

        rate = rospy.Rate(10.0)

        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            time_waited = rospy.get_time() - start_time
            if (time_waited) >= self.max_time_for_transform:
                rospy.loginfo("Waiting for transform timed out. Time waited for transform: %s s"%(time_waited))
                waited_for_transform = False
                break
            else:
                try:
                    now = rospy.Time.now()
                    # Wait for transform from /map to /utm
                    listener.waitForTransform("map", "/utm", now, rospy.Duration(5.0))
                    rospy.loginfo("Transform found. Time waited for transform: %s s"%(rospy.get_time() - start_time))
                    waited_for_transform = True
                    break
                except:
                    pass
            rate.sleep()
        
        return waited_for_transform
    
    def get_next_waypoint(self):
        waypoint = self.waypoints[self.curr_waypoint_idx]
        print self.curr_waypoint_idx
        self.curr_waypoint_idx += 1
        return waypoint
            
            #converts gps coordinated to frame (odom,map,etc)
    def get_pose_from_gps(self, longitude, latitude, frame, pose_test_var = None):
        utm_coords = utm.from_latlon(latitude, longitude)
        
        # create PoseStamped message to set up for do_transform_pose 
        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.pose.position.x = utm_coords[0]
        utm_pose.pose.position.y = utm_coords[1]
        utm_pose.pose.orientation.w = 1.0 #make sure its right side up

        #t = self.tf.getLatestCommonTime("/"+frame, "utm")
        p_in_frame = self.tf.transformPose("/"+frame, utm_pose)
        #print p_in_frame

        return p_in_frame

    def get_pose_from_xy(self, x, y):

        # Convert (x,y) coordinate to pose
        # (x,y) coordinate follows the coordinate system used in spawn.launch

        xy_pose = PoseStamped()
        xy_pose.header.frame_id = 'odom' # not sure what the proper frame is
        xy_pose.pose.position.x = y - 0 # subtract starting y-coordinate
        xy_pose.pose.position.y = x + 19.5 # subtract starting x-coordinate
        xy_pose.pose.orientation.w = 1.0 #make sure its right side up

        return xy_pose
        
    def send_and_wait_goal_to_move_base(self, curr_waypoint):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        action_client.wait_for_server()

        if curr_waypoint.get("longitude"):
            pose = self.get_pose_from_gps(curr_waypoint["longitude"], curr_waypoint["latitude"], curr_waypoint["frame_id"])
        else:
            pose = self.get_pose_from_xy(curr_waypoint["x"], curr_waypoint["y"])

        # ==> TESTING CODE
        #pose = self.get_pose_from_gps(None, None, None, pose_test_var = pose)

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = curr_waypoint["frame_id"]
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set goal position and orientation
        goal.target_pose.pose = pose.pose

        # Sends goal and waits until the action is completed (or aborted if it is impossible)
        action_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        finished_within_time = action_client.wait_for_result(rospy.Duration(60))

        if not finished_within_time:  
            action_client.cancel_goal()  
            rospy.loginfo("Timed out achieving goal")  
        else:  
            rospy.loginfo("Reached nav goal")

    def navigate_waypoints(self):
        while True:
            curr_waypoint = self.get_next_waypoint()

            self.send_and_wait_goal_to_move_base(curr_waypoint)
   
            if (self.curr_waypoint_idx >= len(self.waypoints)):
                break



if __name__ == "__main__":
    static_waypoint_file = 'static_waypoints.json' # File name for static waypoints (provided at competition-time) 
    #static_waypoint_file = 'static_waypoints_xy.json' # File name for static waypoints as x-y coordinates
    max_time_for_transform = 60.0 # Maximum time to wait for the transform. The node times out and shut down if this limit is exceeded.
    
    rospy.init_node('navigate_waypoints')
    waypoints = NavigateWaypoints(static_waypoint_file, max_time_for_transform)
    waypoints.navigate_waypoints() 
