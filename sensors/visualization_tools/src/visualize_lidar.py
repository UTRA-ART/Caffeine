#!/usr/bin/env python2
import roslib
roslib.load_manifest('visualization_tools')
import rospy
import cv2 
import numpy as np

import os
print(os.getcwd())
from visualization_tools.msg import Status

# # Visualizes lidar, up is forward
# def get_xy(heading, _range):
#     x = _range * np.sin(heading)
#     y = _range * np.cos(heading)
#     return np.array([x, y], dtype=int)

def visualize(data):
    print(data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scan', Status, visualize)

    rospy.spin()

if __name__=='__main__':
    listener()