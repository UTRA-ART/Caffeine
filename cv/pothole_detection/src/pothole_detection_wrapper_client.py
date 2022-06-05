#!/usr/bin/env python2
import struct
import sys
import json

import numpy as np
import redis
import rospy

from cv.msg import FloatArray, FloatList
from geometry_msgs.msg import Point
from cv_utils import camera_projection

class CVWrapperClient:
    def __init__(self):
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=3)
        self.pub = rospy.Publisher('cv/pothole_detections', FloatArray, queue_size=10)
        rospy.init_node('pothole_detection_wrapper_client')
        self.r = rospy.Rate(10) # 10hz  
        self.projection = camera_projection.CameraProjection()

    def run(self):
        while not rospy.is_shutdown():
            potholes = self._fromRedis('pothole_detection')
            potholes_msgs = []
            for lane in potholes:
                project_points = self.projection(np.array(lane, dtype=np.int))

                lane_msg = FloatList()
                pts_msg = []
                for pt in project_points:
                    pt_msg = Point()
                    pt_msg.x = pt[0]
                    pt_msg.y = pt[1]
                    pt_msg.z = pt[2]
                    pts_msg += [pt_msg]
                lane_msg.elements = pts_msg
                potholes_msgs += [lane_msg]

            msg = FloatArray()
            msg.lists = potholes_msgs
            self.pub.publish(msg)

            self.r.sleep()

    def _fromRedis(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        potholes = json.loads(encoded)
        return potholes

if __name__ == '__main__':
    wrapper = CVWrapperClient()
    wrapper.run()
