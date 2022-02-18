#!/usr/bin/env python

import rospy
from cv_pkg.msg import cv_msg
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('update', cv_msg, queue_size=10)
    rospy.init_node('cv_node', anonymous=True)
    rate = rospy.Rate(2) # 2hz
    count = 0
    while not rospy.is_shutdown():
        count += 1
        msg = cv_msg()
        pt = Point()
        pt.x = 1 + count
        pt.y = 2 + count
        pt.z = 3 + count
        msg.points.append(pt)
        msg.another_field = 100 + count

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass