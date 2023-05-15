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

        # self.gazebo_roll = 0  
        # self.gazebo_pitch = 0
        # self.gazebo_yaw = 0
        
        # self.gazebo_orientation_list = []
        
        # self.corrected_orientation_list = []

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

        # self.gazebo_point = tf2_geometry_msgs.PointStamped()
        # self.gazebo_orientation = tf2_geometry_msgs.QuaternionStamped()

        # self.gazebo_point.point = gazebo_position.pose.pose.position
        # self.gazebo_orientation.quaternion = gazebo_position.pose.pose.orientation

        # self.gazebo_point.header.frame_id = 'world'
        # self.gazebo_point.header.stamp = rospy.Time.now()
        # self.gazebo_orientation.header.frame_id = 'world'
        # self.gazebo_orientation.header.stamp = rospy.Time.now()

        # self.ground_truth_point = self.tf_buffer.transform(self.gazebo_point, 'ground_truth',rospy.Duration(1.0))
        # self.ground_truth_orientation = self.tf_buffer.transform(self.gazebo_orientation, 'ground_truth', rospy.Duration(1.0))

        # self.ground_truth_msg.pose.pose.position = self.ground_truth_point.point
        # self.ground_truth_msg.pose.pose.orientation = self.ground_truth_orientation.quaternion

        # self.ground_truth_msg.header.stamp=rospy.Time.now()
        # self.ground_truth.msg.header.frame_id = 'ground_truth'

        # self.publisher.publish(self.ground_truth_msg)


        # self.ground_truth_msg=gazebo_position
        # self.ground_truth_msg.header.stamp=rospy.Time.now()

        # #adjust message pose parameters here

        # self.gazebo_orientation_list = [gazebo_position.pose.pose.orientation.x, gazebo_position.pose.pose.orientation.y, gazebo_position.pose.pose.orientation.z, gazebo_position.pose.pose.orientation.w]

        # (self.gazebo_roll,self.gazebo_pitch,self.gazebo_yaw)=euler_from_quaternion(self.gazebo_orientation_list)
        
        # self.corrected_yaw = self.gazebo_yaw - 1.5707 #subtract 90 degrees since gazebo spawns with yaw of +90
        
        # self.corrected_orientation_list = quaternion_from_euler(self.gazebo_roll,self.gazebo_pitch,self.corrected_yaw)

        # #transform coordinates manually with rotation of 90 degrees and translation offset

        # self.ground_truth_msg.pose.pose.position.y = -(gazebo_position.pose.pose.position.x + 19.5)
        # self.ground_truth_msg.pose.pose.position.x = gazebo_position.pose.pose.position.y
        
        # self.ground_truth_msg.pose.pose.orientation.x = self.corrected_orientation_list[0]
        # self.ground_truth_msg.pose.pose.orientation.y = self.corrected_orientation_list[1]
        # self.ground_truth_msg.pose.pose.orientation.z = self.corrected_orientation_list[2]
        # self.ground_truth_msg.pose.pose.orientation.w = self.corrected_orientation_list[3]

if __name__ == '__main__':
    node = GroundTruth()
    rospy.spin()