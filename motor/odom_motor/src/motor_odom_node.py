#!/usr/bin/env python2
import math

import rospy 
import tf
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class MotorOdom:
    def __init__(self):
        self.x = 0 
        self.y = 0
        self.th = 0

        rospy.init_node('odometry_publisher')
        
        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.rate = rospy.Rate(100)
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("speed_feedback", Int16MultiArray, self.update_odom)

    def run(self):
        rospy.spin()

    def update_odom(self, data):
        self.current_time = rospy.Time.now() 
        dt = (self.current_time - self.last_time).to_sec()

        L, R = data
        v = (L + R) / 2
        w = (R - L) / 2

        self.x += v*math.cos(self.th)
        self.y += v*math.sin(self.th)
        self.th += w*dt

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        self.odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        self.r.sleep()


if __name__=='__main__':
    node = MotorOdom()
    node.run()