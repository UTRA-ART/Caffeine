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

from std_msgs.msg import Header
from cv.msg import FloatArray, FloatList
from geometry_msgs.msg import Point
from cv_utils import camera_projection

from line_fitting import fit_lanes
from unet_lane.Inference import Inference

from threshold_lane.threshold import lane_detection

import open3d as o3d
from sensor_msgs.msg import CameraInfo


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
        
        # Load in the depth matrix
        # depth_dir = rospack.get_path('lane_detection') + '/config/depth_sim.npy'
        # depth_matrix_np = np.load(depth_dir)
        # depth_matrix_np = depth_matrix_np * 1000 # Convert from (m) to (mm)
        # depth_matrix_np = cv2.resize(depth_matrix_np, (330, 180))
        # self.depth_matrix = o3d.geometry.Image(depth_matrix_np.astype(np.float32))
    
        if self.classical_mode == 1:
            self.lane_detection = lane_detection
            rospy.loginfo("Lane Detection node initialized with CLASSICAL... ")
        else:
            self.Inference = Inference(self.model_path, False)
            rospy.loginfo("Lane Detection node initialized with DEEP LEARNING... ")


        
    def run(self):
        rospy.Subscriber("image", Image, self.process_image)
        rospy.spin()
   
    def lane_transform(self, img):
        length = img.shape[0]
        width = img.shape[1]
        new_width = int(width/8)


        input_pts = np.float32([[int(width/2-new_width),0], 
                                [int(width/2+new_width),0], 
                                [width,length],
                                [0,length] ])
        output_pts = np.float32([[new_width, 0],
                                [width-new_width, 0],
                                [int(width/2)+new_width,length],
                                [int(width/2)-new_width,length]])
        M2 = cv2.getPerspectiveTransform(input_pts,output_pts)
        out = cv2.warpPerspective(img,M2,(width, length),flags=cv2.INTER_LINEAR)
        # plt.imshow(out)
        # cv2.imshow('test', out)
        # cv2.waitKey(0)
        return out


    def process_image(self, data):
        if data == []:
            return
            
        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        if raw is not None:
            # Get the image
            input_img = raw.copy()
            
            # cv2.imwrite(r'/home/ammarvora/utra/caffeine-ws/src/Caffeine/cv/lane_detection/src' + 'frame.png', input_img)
            # Do model inference 
            output = None

            if self.classical_mode:
                output = self.lane_detection(input_img)
            else:
                output = self.Inference.inference(input_img)

            mask = np.where(output > 0.5, 1., 0.)
            mask = mask.astype(np.uint8)
            mask = cv2.resize(mask, (330, 180))

            # Publish to /cv/model_output
            img_msg = self.bridge.cv2_to_imgmsg(output, encoding='passthrough')
            if img_msg is not None:
                self.pub_raw.publish(img_msg)
            

            '''The following code is needed for virtual layers'''

            rows = np.where(mask==1)[0].reshape(-1,1)
            cols = np.where(mask==1)[1].reshape(-1,1)
            lane_table = np.concatenate((cols,rows),axis=1)

            # print(lane_table)

            
            projected_lanes = self.projection(lane_table)

            # Build the message
            lane_msg = FloatList()
            pts_msg = []

            for pt in projected_lanes:

                pt_msg = Point()
                pt_msg.x = pt[0]
                pt_msg.y = pt[2]
                pt_msg.z = pt[1]

                pts_msg.append(pt_msg)
            lane_msg.elements = pts_msg


            msg_header = Header(frame_id='left_camera_link_optical')
            msg = FloatArray(header=msg_header, lists=[lane_msg])
            self.pub.publish(msg)

                


if __name__ == '__main__':
    wrapper = CVModelInferencer()
    wrapper.run()