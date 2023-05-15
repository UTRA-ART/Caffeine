#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry


if __name__ == "__main__":
    rospy.init_node('zero_odom')
    pub = rospy.Publisher("zero_odom", Odometry, queue_size=1)
    zero_odom = Odometry()
    zero_odom.header.frame_id = "map"
    zero_odom.child_frame_id = "base_link"
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        pub.publish(zero_odom)
        rate.sleep()
