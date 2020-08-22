#!/usr/bin/env python  
import roslib
roslib.load_manifest('load_waypoints')
import rospy
import math
import tf
import std_msgs.msg

if __name__ == '__main__':
    rospy.init_node('gps_transform_listener')
    start_time = rospy.get_time()

    listener = tf.TransformListener()

    gps_ready = rospy.Publisher('gps_ready', std_msgs.msg.Bool, queue_size=1)

    rate = rospy.Rate(10.0)

    available = False 
    rospy.sleep(1)
    listener.waitForTransform("/caffeine/map", "/utm", rospy.Time(), rospy.Duration(50.0))

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(50.0))
            available = True 
            print(rospy.get_time() - start_time)

        except (tf.LookupException, tf.ConnectivityException):
            continue

        gps_ready.publish(available)

        rate.sleep()