#!/usr/bin/env python3
import struct
import sys 
import time
import json
import os

import cv2
import numpy as np
import onnx
import onnxruntime as ort 

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rospkg
import rospy

from cv.msg import FloatArray, FloatList
from geometry_msgs.msg import Point
from cv_utils import camera_projection

from line_fitting import fit_lanes
from unet_lane.Inference import Inference

from threshold_lane.threshold import lane_detection

class CVModelInferencer:
    def __init__(self):
        rospy.init_node('lane_detection_model_inference')
        
        self.pub = rospy.Publisher('cv/lane_detections', FloatArray, queue_size=10)
        self.pub_raw = rospy.Publisher('cv/model_output', Image, queue_size=10)

        self.bridge = CvBridge()
        self.projection = camera_projection.CameraProjection()
        
        rospack = rospkg.RosPack()
        self.model_path = rospack.get_path('lane_detection') + '/models/competition_model_4c_128.pt'

        # Get the parameter to decide between deep learning and classical
        self.classical_mode = rospy.get_param('~lane_detection_mode')
        self.Inference = None
        self.lane_detection = None

        if self.classical_mode == 1:
            self.lane_detection = lane_detection
            rospy.loginfo("Lane Detection node initialized with CLASSICAL... ")
        else:
            self.Inference = Inference(self.model_path, False)
            rospy.loginfo("Lane Detection node initialized with DEEP LEARNING... ")



    def run(self):
        rospy.Subscriber("image", Image, self.process_image)
        rospy.spin()

    def process_image(self, data):
        if data == []:
            return
            
        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        if raw is not None:
            # Get the image
            input_img = raw.copy()

            # Do model inference 
            mask = None

            if self.classical_mode:
                mask = self.lane_detection(input_img)
            else:
                mask = self.Inference.inference(input_img)



            # Publish to /cv/model_output
            img_msg = self.bridge.cv2_to_imgmsg(mask, encoding='passthrough')
            if img_msg is not None:
                self.pub_raw.publish(img_msg)
            

            '''The following code is needed for virtual layers'''
            # Publish to /cv/lane_detection
            lanes = fit_lanes(mask)
            if lanes is not None:
                # rospy.loginfo("lanes: %s", lanes)
                lanes_msgs = []
                for lane in lanes:
                    project_points = self.projection(np.array(lane, dtype=np.uint8))

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


if __name__ == '__main__':
    wrapper = CVModelInferencer()
    wrapper.run()