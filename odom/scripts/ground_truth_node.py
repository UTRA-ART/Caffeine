#!/usr/bin/env python3

import rospy
import numpy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

#to run node -> python3 ground_truth_node.py

class GroundTruth:
    def __init__(self):
        rospy.init_node("ground_truth_pub")

        self.ground_truth_msg = Odometry()
        self.rate = rospy.Rate(50) #gazebo plugin for ground truth is set to update of 50 Hz

        self.gazebo_roll = 0
        self.gazebo_pitch = 0
        self.gazebo_yaw = 0
        
        self.gazebo_orientation_list = []
        
        self.corrected_orientation_list = []

        self.publisher = rospy.Publisher("ground_truth_odom", Odometry, queue_size=1)
        rospy.Subscriber("ground_truth/state", Odometry, self.callback)

    def callback(self, gazebo_position):
        self.ground_truth_msg=gazebo_position
        self.ground_truth_msg.header.stamp=rospy.Time.now()

        #adjust message pose parameters here

        self.gazebo_orientation_list = [gazebo_position.pose.pose.orientation.x, gazebo_position.pose.pose.orientation.y, gazebo_position.pose.pose.orientation.z, gazebo_position.pose.pose.orientation.w]

        (self.gazebo_roll,self.gazebo_pitch,self.gazebo_yaw)=euler_from_quaternion(self.gazebo_orientation_list)
        
        self.corrected_yaw = self.gazebo_yaw - 1.5707 #subtract 90 degrees since gazebo spawns with yaw of +90
        
        self.corrected_orientation_list = quaternion_from_euler(self.gazebo_roll,self.gazebo_pitch,self.corrected_yaw)

        #transform coordinates manually with rotation of 90 degrees and translation offset

        self.ground_truth_msg.pose.pose.position.y = -(gazebo_position.pose.pose.position.x) + 19.5
        self.ground_truth_msg.pose.pose.position.x = gazebo_position.pose.pose.position.y
        
        self.ground_truth_msg.pose.pose.orientation.x = self.corrected_orientation_list[0]
        self.ground_truth_msg.pose.pose.orientation.y = self.corrected_orientation_list[1]
        self.ground_truth_msg.pose.pose.orientation.z = self.corrected_orientation_list[2]
        self.ground_truth_msg.pose.pose.orientation.w = self.corrected_orientation_list[3]

        self.publisher.publish(self.ground_truth_msg)

if __name__ == '__main__':
    node = GroundTruth()
    rospy.spin()