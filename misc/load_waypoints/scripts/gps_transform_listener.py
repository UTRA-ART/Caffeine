#!/usr/bin/env python  
import roslib
roslib.load_manifest('load_waypoints')
import rospy
import math
import tf
# import geometry_msgs.msg
import std_msgs.msg
# import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('gps_transform_listener')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    gps_ready = rospy.Publisher('gps_ready', std_msgs.msg.Bool, queue_size=1)

    rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    # print('fda')
    # rospy.loginfo('fsdas')

    available = False 
    rospy.sleep(1)
    listener.waitForTransform("/caffeine/map", "/utm", rospy.Time(), rospy.Duration(50.0))
    while not rospy.is_shutdown():
        # print(available)
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(50.0))
            # (trans,rot) = listener.lookupTransform("/caffeine/map", "/utm", now)
            available = True 
            # print(trans,rot,rospy.Time.now())
            # rospy.loginfo('fds')
        except (tf.LookupException, tf.ConnectivityException):
            # print('fda')
            # rospy.loginfo('fsdas')
            continue

        gps_ready.publish(available)
        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)

        rate.sleep()