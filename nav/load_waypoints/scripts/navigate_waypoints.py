#!/usr/bin/env python3

import json
import os
import sys

import actionlib
import rospkg
import rospy
import std_msgs.msg
from std_msgs.msg import Bool
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
        self.max_time_for_transform = max_time_for_transform # Maximum time to wait for the transform. Node shuts down if time limit hit
        self.waited_for_transform = False # Initialize the boolean for whether waiting has timed out 

        self.launch_state = rospy.get_param('/load_waypoints_server/launch_state')
        self.ignore_lidar = False
        self.start_direction = 1 # North: 1, South = -1
        self.laps = 0
        self.populate_waypoint_dict() 
        self.current_lap = 0
        self.curr_waypoint_idx = 0 if self.start_direction == 1 else len(self.waypoints) - 2
        rospy.loginfo("First goal: %s" % (self.curr_waypoint_idx))
        self.tf = TransformListener()
        self.publisher = rospy.Publisher('/waypoint_int', Bool, queue_size=10)



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

        self.start_direction = 1 if waypoint_data["start_direction"] == "north" else -1
        self.laps = waypoint_data["laps"]

        rospy.loginfo("start_direction: %s" % (self.start_direction))
    
        # Call method to wait for transform 
        self.waited_for_transform = self.wait_for_utm_transform()

        # Check if successfully waited for the transform within the time limit. If successful, continue populating the waypoint dict. 
        if self.waited_for_transform:
            # After waiting UTM transform, capture a message from the gps/fix topic
            gps_info = rospy.wait_for_message('gps/fix', NavSatFix)
        else:
            rospy.loginfo("Waiting for transform from /map to /utm timed out!")

        # Add additional waypoints to the corners of the course to avoid incorrect shortcuts
        if waypoint_data["add_corners"]:
            self.add_corners(waypoint_data, gps_info)
        else:
            # Parse through json data and create list of lists holding all waypoints
            for waypoint in waypoint_data["waypoints"]:
                self.waypoints[waypoint['id']] = waypoint

        # Append the starting gps coordinate to the waypoints dict as the final waypoint
        last_coord_idx = len(self.waypoints) 

        # Append a final waypoint to return to the start (i.e. waypoint to return to start)
        self.waypoints[last_coord_idx] = {
            'id': last_coord_idx, 
            'longitude': gps_info.longitude, 
            'latitude': gps_info.latitude, 
            'description': 'Initial start location', 
            'frame_id': waypoint_data["waypoints"][0]["frame_id"] # For now is 'odom'
        }

        # Show waypoints 
        rospy.loginfo("Successfully loaded waypoints dict")

        return 
    
    def add_corners(self, waypoint_data, gps_info):
        is_sim = self.launch_state == "sim"
        frame = waypoint_data["waypoints"][0]["frame_id"]
        j = 0

        # Account for whether the state is sim because map is rotated to face East instead of North
        for i in range(len(waypoint_data["waypoints"]) + 3):
            if i == 0:
                self.waypoints[i] = {
                    'id': i, 
                    'longitude': -79.3905355 if is_sim else gps_info.longitude, 
                    'latitude': gps_info.latitude + 0.00001 if is_sim else waypoint_data["waypoints"][0]["latitude"], 
                    'description': "First Corner", 
                    'frame_id': frame
                }
            elif i == 5:
                self.waypoints[i] = {
                    'id': i, 
                    'longitude': -79.38998072 if is_sim else waypoint_data["waypoints"][3]["longitude"], 
                    'latitude': 43.65714925 if is_sim else waypoint_data["waypoints"][3]["latitude"] - 0.000036, 
                    'description': "Third Corner", 
                    'frame_id': frame
                }
            elif i == 6:
                self.waypoints[i] = {
                    'id': i, 
                    'longitude': waypoint_data["waypoints"][3]["longitude"] if is_sim else gps_info.longitude, 
                    'latitude':  gps_info.latitude - 0.00001 if is_sim else waypoint_data["waypoints"][3]["latitude"], 
                    'description': "Fourth Corner", 
                    'frame_id': frame
                }
            else:
                self.waypoints[i] = waypoint_data["waypoints"][j]
                self.waypoints[i]["id"] = i
                j += 1

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
                    listener.waitForTransform("/map", "/utm", now, rospy.Duration(5.0))
                    rospy.loginfo("Transform found. Time waited for transform: %s s"%(rospy.get_time() - start_time))
                    waited_for_transform = True
                    break
                except:
                    pass
            rate.sleep()
        
        return waited_for_transform
    
    def get_next_waypoint(self):
        waypoint = self.waypoints[self.curr_waypoint_idx]
        rospy.loginfo("Next Goal: %s"%(waypoint["description"]))
        if self.curr_waypoint_idx == 3 and self.start_direction == 1: # curr_waypoint_idx = 2 means heading towards id 2
            self.ignore_lidar = True 
        elif self.curr_waypoint_idx == 2 and self.start_direction == -1:
            self.ignore_lidar = True 
        else:
            self.ignore_lidar = False

        for i in range(10):
            self.publisher.publish(self.ignore_lidar)

        self.curr_waypoint_idx += self.start_direction #try self.curr_waypoint_idx = (self.curr_waypoint_idx + self.start_direction) % len(self.waypoints)
        if self.curr_waypoint_idx < 0 and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = len(self.waypoints) - 1
        elif self.curr_waypoint_idx >= len(self.waypoints) and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = 0

        return waypoint
    
    def get_pose_from_gps(self, longitude, latitude, frame, pose_test_var = None):
        '''converts gps coordinates to frame (odom,map,etc)'''
        
        # create PoseStamped message to set up for do_transform_pose
        utm_coords = utm.from_latlon(latitude, longitude)#latitude and longitude transformed into UTM
        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.pose.position.x = utm_coords[0]
        utm_pose.pose.position.y = utm_coords[1]
        utm_pose.pose.orientation.w = 1.0 # to make sure its right side up

        p_in_frame = self.tf.transformPose("/"+frame, utm_pose)

        return p_in_frame
        
    def send_and_wait_goal_to_move_base(self, curr_waypoint):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        action_client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = curr_waypoint["frame_id"]
        goal.target_pose.header.stamp = rospy.Time.now()

        #while not reached Goal, resend the goal. 
        #if finished goal, send the next goal and start again. 
        finished_within_time = 0

        times =0
        
        while 1:
            # Set goal position and orientation
            pose = self.get_pose_from_gps(curr_waypoint["longitude"], curr_waypoint["latitude"], curr_waypoint["frame_id"])
            goal.target_pose.pose = pose.pose
            #rospy.loginfo("read from json again")
            
            # Sends goal and waits until the action is completed (or aborted if it is impossible)
            action_client.send_goal(goal)
            #rospy.loginfo("sends goal again")

            # Give certain time for rover to set goal repetitively
            finished_within_time = action_client.wait_for_result(rospy.Duration(5))
            #rospy.loginfo("wait 5 secs again")
            if finished_within_time:
                #rospy.loginfo("Reached nav goal")
                break
            else:
                times += 1
                #rospy.loginfo("Resending the goal: %d", times)  


    def navigate_waypoints(self):
        while True:
            curr_waypoint = self.get_next_waypoint()
            self.send_and_wait_goal_to_move_base(curr_waypoint)

            if (self.current_lap >= self.laps):
                break


if __name__ == "__main__":
    # CHANGE THIS TO GET MAP SPECIFIC GPS WAYPOINTS
    launch_state = rospy.get_param('/load_waypoints_server/launch_state')
    if launch_state == "sim":
        static_waypoint_file = 'static_waypoints_pavement.json'
        # static_waypoint_file = 'static_waypoints_grass.json'
    else:
        static_waypoint_file = 'IGVC_practice.json'

    rospy.init_node('navigate_waypoints')
    waypoints = NavigateWaypoints(static_waypoint_file, max_time_for_transform=60.0)
    waypoints.navigate_waypoints()
    rospy.init_node('Finished Navigating!!')