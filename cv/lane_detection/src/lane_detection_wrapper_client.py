#!/usr/bin/env python2
import struct
import sys
import json

import numpy as np
import redis
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from cv.msg import FloatArray, FloatList
from geometry_msgs.msg import Point
from cv_utils import camera_projection
from datetime import datetime

class CVWrapperClient:
    def __init__(self):
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=3)
        self.pub = rospy.Publisher('cv/lane_detections', FloatArray, queue_size=10)
        self.pub_raw = rospy.Publisher('cv/model_output', Image, queue_size=10)
        rospy.init_node('lane_detection_wrapper_client')
        self.r = rospy.Rate(30) # 30hz

        self.bridge = CvBridge()
        self.projection = camera_projection.CameraProjection()
        self.last_timestamp = str(datetime.utcnow())


    def run(self):
        while not rospy.is_shutdown():
            ts = self._fromRedisImgTimeStamp('lane_detection_timestamp')
            if self.last_timestamp == ts:
                continue
            else:
                self.last_timestamp = ts


            lanes = self._fromRedisLanes('lane_detection')
            mask = self._fromRedisImg('cv/model/output')
      
            if mask is not None:
                img = self.bridge.cv2_to_imgmsg(mask*255, encoding='passthrough')
                self.pub_raw.publish(img)

            if lanes is not None:
                lanes_msgs = []
                for lane in lanes:
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
                    lanes_msgs += [lane_msg]

                msg = FloatArray()
                msg.lists = lanes_msgs
                self.pub.publish(msg)

            self.r.sleep()

    def _fromRedisLanes(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        if encoded is None:
            return None
        lanes = json.loads(encoded)
        return lanes

    def _fromRedisImg(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        if encoded is None:
            return None
        h, w = struct.unpack('>II',encoded[:8])
        a = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8, offset=8), 1).reshape(h,w,3)
        return a

    def _fromRedisImgTimeStamp(self, name):
        time = self.redis.get(name)
        
        if time is None:
            return None
        return time

if __name__ == '__main__':
    wrapper = CVWrapperClient()
    wrapper.run()
