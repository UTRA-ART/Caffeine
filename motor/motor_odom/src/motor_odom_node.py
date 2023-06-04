#!/usr/bin/env python3
import math
import numpy as np
import rospy 
import tf
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class MotorOdom:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.WHEELBASE = 0.89

        rospy.init_node('odometry_publisher')
        
        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.rate = rospy.Rate(100)
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.launch_state = rospy.get_param('/motor_odom_node/launch_state')
        if self.launch_state == "sim":
            rospy.Subscriber("/right_wheel/feedback", Float64, self.update_right_speed, queue_size=1)
            rospy.Subscriber("/left_wheel/feedback", Float64, self.update_left_speed, queue_size=1)
        else:
            rospy.Subscriber("/cmd_vel", Twist, self.update_speed, queue_size=1)

        self.R = 0
        self.L = 0

    def update_left_speed(self, data):
        self.L = data.data
    def update_right_speed(self, data):
        self.R = data.data

    def update_speed(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        self.R = v + ((self.WHEELBASE * w) / 2.0)
        self.L = v - ((self.WHEELBASE * w) / 2.0)

    def run(self):
        while not rospy.is_shutdown():
            self.update_odom()

    def update_odom(self):
        self.current_time = rospy.Time.now() 
        dt = (self.current_time - self.last_time).to_sec()

        v = (self.L + self.R) / 2.0
        w = (self.R - self.L) / 2.0

        vx = v*math.cos(self.th)
        vy = v*math.sin(self.th)

        self.x += vx * dt
        self.y += vy * dt
        self.th += w * dt

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        if self.launch_state == "sim":
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )

        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, w))

        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        self.rate.sleep()


if __name__=='__main__':
    node = MotorOdom()
    node.run()
