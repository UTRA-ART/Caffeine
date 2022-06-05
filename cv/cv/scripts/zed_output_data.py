#!/usr/bin/env python2
import struct
import sys

import cv2
import numpy as np
import redis
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

'''
Outputs example data from the zed cemera. zed_wrapper zed.launch must be running. 
'''

class ZedWrapperServer:
    def __init__(self):
        self.bridge = CvBridge()
        self.delay = 5
        self.counter = 0

    def run(self):
        rospy.init_node('zed_data_output')
        rospy.Subscriber("image", Image, self.save_image)
        rospy.spin()

    def save_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')  

        if self.delay == 0: 
            cv2.imwrite('/media/art-jetson/SSD/caffeine/comp__data/' + str(self.counter) + '.png', img)
            self.delay = 5
            self.counter += 1
        else:
            self.delay -= 1

if __name__ == '__main__':
    wrapper = ZedWrapperServer()
    wrapper.run()