#!/usr/bin/env python3

import rospy

import tf
import tf2_ros
from tf import TransformListener

from cv.msg import FloatArray, FloatList
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class LaneScanConversion:
    def _init_(self):
        rospy.init_node('lane_scan_conversion', anonymous=True)
        self.pub = rospy.Publisher('cv/lane_detections/scan', LaserScan, queue_size=10)

        self.listener = tf.TransformListener()
        listener.waitForTransform("/left_camera_link_optical", "/base_laser", rospy.Time(), rospy.Duration(10.0))

    def run(self):
        rospy.Subscriber("cv/lane_detections", FloatArray, self.process_lanes)
        rospy.spin()

    def process_lanes(self, data):
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'base_laser'
         # Look into filling in the header.stamp and header.seq (for time of message)

        for lane in data.list:
            for point in lanes:

                old_pose = PoseStamped()
                old_pose.header.frame_id = data.header.frame_id
                old_pose.pose.position.x = point.x
                old_pose.pose.position.x = point.y
                old_pose.pose.position.x = point.z
                old_pose.pose.orientation.w = 1.0

                new_pose = self.listener.transformPose('/base_laser', old_pose) # from frame /left_camera_link_optical to /base_laser
                
                # Calculate the angle and distance

                # Find the index in scan_msg.ranges whose angle matches with angle calculated 
                # Set the value at that index as the distance

                # Let Loop iterate
                
        # Look into doing interpolation for the angles in scan_msg that don't have a distance value
        #publish the scan_msg
        self.pub.publish(scan_msg)


if __name__ == "__main__":
    conversion = LaneScanConversion()
    conversion.run()




    
        