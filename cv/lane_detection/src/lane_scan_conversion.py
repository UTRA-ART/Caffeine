#!/usr/bin/env python3

import rospy

import tf
import tf2_ros
from tf import TransformListener

from cv.msg import FloatArray, FloatList
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

import numpy as np
import math

class LaneScanConversion:
    def _init_(self):
        # initialize node and publisher
        rospy.init_node('lane_scan_conversion', anonymous=True)
        self.pub = rospy.Publisher('cv/lane_detections/scan', LaserScan, queue_size=10)

        # listen for transform from camera to lidar frames
        self.listener = tf.TransformListener()
        listener.waitForTransform("/left_camera_link_optical", "/base_laser", rospy.Time(), rospy.Duration(10.0))

    def run(self):
        # subscribe to lane detection topic
        rospy.Subscriber("cv/lane_detections", FloatArray, self.process_lanes)
        # run
        rospy.spin()

    def process_lanes(self, data):
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'base_laser'
        # Look into filling in the header.stamp and header.seq (for time of message)

        # array of angle values
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment, scan_msg.angle_increment)

        # iterate through 
        for lane in data.list:
            for point in lanes:

                old_pose = PoseStamped()
                old_pose.header.frame_id = data.header.frame_id
                old_pose.pose.position.x = point.x
                old_pose.pose.position.x = point.y
                old_pose.pose.position.x = point.z
                old_pose.pose.orientation.w = 1.0

                # get x, y, z in lidar frame
                new_pose = self.listener.transformPose('/base_laser', old_pose) # from frame /left_camera_link_optical to /base_laser
                
                # Calculate the angle and distance
                x = new_pose.pose.position.x
                y = new_pose.pose.position.y

                theta = np.arctan(x / y) - (np.pi / 2) # angle from x axis
                d = math.sqrt(x**2 + y**2) # distance from lidar

                # Find the index in scan_msg.ranges whose angle matches with angle calculated 
                # Set the value at that index as the distance
                index = np.abs(angles - theta).argmin()

                # if the value at scan_msg.ranges[index] has not been set yet, just change it, else choose the smaller value between the current value in scan_msg.ranges[index] and the value, d
                if scan_msg.ranges[index] == float('inf'):
                    scan_msg.ranges[index] = d
                else:
                    scan_msg.ranges[index] = min(d, scan_msg.ranges[index])

                # Let Loop iterate
                
        # nearest-neighbour interpolation for the angles in scan_msg that don't have a distance value
        ranges_interp = np.copy(scan_msg.ranges)
        inf_indices = np.isinf(ranges_interp)
        valid_indices = np.logical_not(inf_indices)
        nearest_indices = np.abs(valid_indices[:, None] - inf_indices).argmin(axis=0)
        ranges_interp[inf_indices] = ranges_interp[valid_indices][nearest_indices]
        scan_msg.ranges = ranges_interp

        # publish the scan_msg
        self.pub.publish(scan_msg)


if __name__ == "__main__":
    conversion = LaneScanConversion()
    conversion.run()




    
        