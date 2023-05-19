#!/usr/bin/env python3

import rospy
import numpy
import math
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#to run node -> python3 ground_truth_node.py

class GroundTruth:
    def __init__(self):
        rospy.init_node("ground_truth_pub")
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ground_truth_msg = Odometry()
        self.rate = rospy.Rate(50) #gazebo plugin for ground truth is set to update of 50 Hz

        self.publisher = rospy.Publisher("ground_truth_odom", Odometry, queue_size=1)
        rospy.Subscriber("ground_truth/state", Odometry, self.callback)

    def callback(self, gazebo_position):
        #transform coordinates with rotation of 90 degrees and translation offset

        self.gazebo_pose = tf2_geometry_msgs.PoseStamped()

        self.gazebo_pose.pose = gazebo_position.pose.pose

        self.gazebo_pose.header.frame_id = 'world'
        self.gazebo_pose.header.stamp = rospy.Time.now()

        self.ground_truth_pose = self.tf_buffer.transform(self.gazebo_pose,'ground_truth',rospy.Duration(1.0))

        self.ground_truth_msg.pose.pose = self.ground_truth_pose.pose

        self.ground_truth_msg.header.stamp=rospy.Time.now()
        self.ground_truth_msg.header.frame_id = 'ground_truth'

        self.publisher.publish(self.ground_truth_msg)

if __name__ == '__main__':
    node = GroundTruth()
    rospy.spin()