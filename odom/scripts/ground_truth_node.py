#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class GroundTruth:
    def __init__(self):
        rospy.init_node("ground_truth_pub")

        self.ground_truth_msg = Odometry()
        self.rate = rospy.Rate(50) #gazebo plugin for ground truth is set to update of 50 Hz

        self.gazebo_roll = 0
        self.gazebo_pitch = 0
        self.gazebo_yaw = 0
        self.corrected_yaw = 0

        self.publisher = rospy.Publisher("ground_truth_odom", Odometry, queue_size=1)
        rospy.Subscriber("ground_truth/state", Odometry, self.callback)

    def callback(gazebo_position):
        self.ground_truth_msg=gazebo_position
        self.ground_truth_msg.header.stamp=rospy.Time.now()

        #adjust message pose parameters here

        (self.gazebo_roll,self.gazebo_pitch,self.gazebo_yaw)=euler_from_quaternion(gazebo_position.pose.pose.orientation)
        
        self.corrected_yaw = gazebo_yaw - 1.5707 #subtract 90 degrees since gazebo spawns with yaw of +90

        self.ground_truth_msg.pose.pose.position.x = gazebo_position.pose.pose.position.x + 19.5
        self.ground_truth_msg.pose.pose.orientation = quaternion_from_euler(self.gazebo_roll,self.gazebo_pitch,self.corrected_yaw)
        self.publisher.publish(ground_truth_msg)

if __name__ == '__main__':
    node = GroundTruth()
    rospy.spin()