#!/usr/bin/env python3

import math

import matplotlib.pyplot as plt
import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class OdomPlotterNode:
    def __init__(self):
        self.ground_truth_sub = message_filters.Subscriber(
            "ground_truth_odom", Odometry, queue_size=10
        )
        self.tracked_pose_sub = message_filters.Subscriber(
            "tracked_pose", PoseStamped, queue_size=10
        )

        self.time_sync = message_filters.TimeSynchronizer(
            [self.ground_truth_sub, self.tracked_pose_sub], 10
        )

        self.time_sync.registerCallback(self.on_callback)

        self.ground_truth_states = []
        self.tracked_poses = []

    def on_callback(self, odom_msg, pose_msg):
        self.ground_truth_states.append(odom_msg)
        self.tracked_poses.append(pose_msg)

    def plot(self):
        gt_x, gt_y, gt_z = [], [], []
        for gt_odom in self.ground_truth_states:
            gt_x.append(gt_odom.pose.pose.position.x)
            gt_y.append(gt_odom.pose.pose.position.y)
            gt_z.append(gt_odom.pose.pose.position.z)

        tracked_x, tracked_y, tracked_z = [], [], []
        for tracked in self.tracked_poses:
            tracked_x.append(tracked.pose.position.x)
            tracked_y.append(tracked.pose.position.y)
            tracked_z.append(tracked.pose.position.z)

        N = range(len(self.tracked_poses))
        plt.plot(N, tracked_x, label="cartographer")
        plt.plot(N, gt_x, label="gt")
        plt.xlabel("Timestep")
        plt.ylabel("x position")
        plt.title("x vs timestep")
        plt.legend()
        plt.savefig("/tmp/x_position.png")
        plt.clf()

        plt.plot(N, tracked_y, label="cartographer")
        plt.plot(N, gt_y, label="gt")
        plt.xlabel("Timestep")
        plt.ylabel("y position")
        plt.title("y vs timestep")
        plt.legend()
        plt.savefig("/tmp/y_position.png")
        plt.clf()

        x_rmse = list(math.sqrt((x - x_hat) ** 2) for x, x_hat in zip(gt_x, tracked_x))
        plt.plot(N, x_rmse)
        plt.xlabel("Timestep")
        plt.ylabel("rmse")
        plt.title("x rmse vs timestep")
        plt.savefig("/tmp/x_rmse.png")
        plt.clf()

        y_rmse = list(math.sqrt((y - y_hat) ** 2) for y, y_hat in zip(gt_y, tracked_y))
        plt.plot(N, y_rmse)
        plt.xlabel("Timestep")
        plt.ylabel("rmse")
        plt.title("y rmse vs timestep")
        plt.savefig("/tmp/y_rmse.png")
        plt.clf()

        """
        plt.title("Cartographer position (2d)")
        plt.plot(tracked_x, tracked_y)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.savefig("/tmp/cartographer_position.png")
        plt.clf()

        plt.title("Ground truth position (2d)")
        plt.plot(gt_x, gt_y)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.savefig("/tmp/ground_truth_position.png")
        plt.clf()
        """


def main():
    rospy.init_node("odom_plotter")
    odom_plotter_node = OdomPlotterNode()
    rospy.spin()
    odom_plotter_node.plot()


if __name__ == "__main__":
    main()
