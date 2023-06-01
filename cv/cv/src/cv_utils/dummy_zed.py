#!/usr/bin/env python3
import struct

import cv2
import numpy as np
import redis
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

'''
    Dummy node for testing cv pipeline without zed camera. Just uploads webcam data in its place
'''
class DummyZed:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/zed/zed_node/rgb/image_rect_color", Image)
        self.cap = cv2.VideoCapture(0)

    def run(self):
        rospy.init_node('dummy_zed')
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret, img = self.cap.read()

            if ret: 
                img_msg = self.bridge.cv2_to_imgmsg(img, encoding='passthrough')

                self.pub.publish(img_msg)
            else:
                print "Unable to get picture from webcam"
            rate.sleep()

if __name__ == '__main__':
    wrapper = DummyZed()
    wrapper.run()