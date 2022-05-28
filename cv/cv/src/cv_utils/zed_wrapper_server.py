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
<<<<<<< HEAD
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=0)
=======
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=3)
>>>>>>> f635010e4ff6d636b5b227b6d413a0e255cad00f

    def run(self):
        rospy.init_node('zed_server_wrapper')
        rospy.Subscriber("image", Image, self.forward_image)
        rospy.spin()

    def forward_image(self, data):
<<<<<<< HEAD
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self._toRedis(img, "zed/preprocessed")
=======
        if data != []:
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self._toRedis(img, "zed/preprocessed")
>>>>>>> f635010e4ff6d636b5b227b6d413a0e255cad00f

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