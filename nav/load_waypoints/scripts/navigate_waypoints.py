#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

import rospy, os, json, sys 
import rospkg
import std_msgs.msg
import actionlib
import tf

class NavigateWaypoints:
    def __init__(self, static_waypoint_file, max_time_for_transform):
        self.waypoints = dict() 
        self.static_waypoint_file = static_waypoint_file
        self.max_time_for_transform = max_time_for_transform
        self.waited_for_transform = False # Initialize the boolean for whether or waiting has timed out 

        self.populate_waypoint_dict() 
        self.curr_waypoint_idx = 0 

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
            self.waypoints[last_coord_idx] = {
                'id': last_coord_idx, 'longitude': gps_info.longitude, 'latitude': gps_info.latitude, 'description': 'Initial start location'}

            # Show waypoints 
            rospy.loginfo("Successfully loaded waypoints dict")
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
        
        ns = rospy.get_namespace()

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
                    # Wait for transform from /caffeine/map to /utm
                    listener.waitForTransform(ns+"map", "/utm", now, rospy.Duration(5.0))
                    rospy.loginfo("Transform found. Time waited for transform: %s s"%(rospy.get_time() - start_time))
                    waited_for_transform = True
                    break
                except:
                    pass
            rate.sleep()
        
        return waited_for_transform
    
    def get_next_waypoint(self):
        waypoint = self.waypoints[self.curr_waypoint_idx]
        self.curr_waypoint_idx += 1
        return waypoint
            
    def get_pose_from_gps(self, longitude, latitude, frame, pose_test_var = None):
        # Awaiting For Issue #12's Completion

        # ==> TESTING CODE
        return pose_(pose_test_var[0], pose_test_var[1], pose_test_var[2], pose_test_var[3], pose_test_var[4], pose_test_var[5])
        
    def send_and_wait_goal_to_move_base(self, pose, frame):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        action_client.wait_for_server()

        # ==> UNSUPPORTED CODE, WAITING ON OTHER CODE SUBMISSION
        # pose = self.get_pose_from_gps(curr_waypoint["longitude"], curr_waypoint["latitude"], curr_waypoint["frame_id"])

        # ==> TESTING CODE
        pose = self.get_pose_from_gps(None, None, None, pose_test_var = pose)

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set goal position (x, y, and z are in meters)
        goal.target_pose.pose.position.x = pose.x
        goal.target_pose.pose.position.y = pose.y
        goal.target_pose.pose.position.z = pose.z

        # Convert roll, pitch, yaw to quaternion and set pose with roll, pitch and yaw in radians
        q = quaternion_from_euler(pose.roll, pose.pitch, pose.yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # Sends goal and waits until the action is completed (or aborted if it is impossible)
        action_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        wait = action_client.wait_for_result()

        rospy.loginfo("Reached nav goal")

    def navigate_waypoints(self):
        while True:
            curr_waypoint = self.get_next_waypoint()

            # ==> TESTING CODE
            pose = curr_waypoint["pose_test"]

            self.send_and_wait_goal_to_move_base(pose, curr_waypoint["frame_id"])
   
            if (self.curr_waypoint_idx >= len(self.waypoints)):
                break

class pose_:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

if __name__ == "__main__":
    static_waypoint_file = 'static_waypoints.json' # File name for static waypoints (provided at competition-time) 
    max_time_for_transform = 60.0 # Maximum time to wait for the transform. The node times out and shut down if this limit is exceeded.
    
    rospy.init_node('navigate_waypoints')
    waypoints = NavigateWaypoints(static_waypoint_file, max_time_for_transform)
    waypoints.navigate_waypoints() 