#!/usr/bin/env python2
import struct
import sys
import json

import numpy as np
import redis
import rospy

import camera_projection

class CVWrapperClient:
    def __init__(self):
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=0)
        # self.pub = rospy.Publisher('cv/lane_detections', String, queue_size=10)
        rospy.init_node('cv_wrapper_client')
        self.r = rospy.Rate(10) # 10hz

    def run(self):
        while not rospy.is_shutdown():
            lanes = self._fromRedis('lane_detection')

            # TODO: Publish lane data 
            


            # self.pub.publish("hello world")
            self.r.sleep()

    def forward_image(self):
        lanes = self._fromRedis("lane_detection")

    def _fromRedis(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        lanes = json.loads(encoded)
        return lanes

if __name__ == '__main__':
    wrapper = CVWrapperClient()
    wrapper.run()


    