#!/usr/bin/env python2
import struct
import sys

import cv2
import numpy as np
import redis
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ZedWrapperServer:
    def __init__(self):
        self.bridge = CvBridge()
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=0)

    def run(self):
        rospy.init_node('zed_server_wrapper')
        rospy.Subscriber("image", Image, self.forward_image)
        rospy.spin()

    def forward_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self._toRedis(img, "zed/preprocessed")

    def _toRedis(self,img,name):
        """Store given Numpy array 'img' in Redis under key 'name'"""
        h, w = img.shape[:2]
        shape = struct.pack('>II',h,w)

        retval, buffer = cv2.imencode('.png', img)
        img_bytes = np.array(buffer).tostring()

        encoded = shape + img_bytes

        # Store encoded data in Redis
        self.redis.set(name,encoded)

        return

if __name__ == '__main__':
    wrapper = ZedWrapperServer()
    wrapper.run()