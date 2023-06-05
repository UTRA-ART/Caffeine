#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

class Vel_Cmds_to_EKF:
    def __init__(self):
        rospy.init_node("motor_cmd_to_ekf_pub")
        rospy.Subscriber("twist_mux/cmd_vel", Twist, self.callback)
        self.publisher = rospy.Publisher("motor_cmd_for_ekf", TwistWithCovarianceStamped,queue_size=10)

    def callback(self, cmd_msg):
        self.msg_for_ekf = TwistWithCovarianceStamped()
        self.msg_for_ekf.twist.twist = cmd_msg
        self.msg_for_ekf.header.stamp = rospy.Time.now()
        self.msg_for_ekf.header.frame_id = "base_link"

        self.msg_for_ekf.twist.covariance = [0 0 0 0 0 0
                                             0 0 0 0 0 0
                                             0 0 0 0 0 0
                                             0 0 0 0 0 0
                                             0 0 0 0 0 0]

        self.publisher.publish(self.msg_for_ekf)

if __name__ == '__main__':
    node = Vel_Cmds_to_EKF()
    rospy.spin()