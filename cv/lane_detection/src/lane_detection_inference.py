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
import torch

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
from ultralytics import YOLO

from threshold_lane.threshold import lane_detection

# import open3d as o3d
from sensor_msgs.msg import CameraInfo


class CVModelInferencer:
    def __init__(self):
        rospy.init_node('lane_detection_model_inference')
        
        self.pub = rospy.Publisher('cv/lane_detections', FloatArray, queue_size=10)
        self.pub_raw = rospy.Publisher('cv/model_output', Image, queue_size=10)

        self.bridge = CvBridge()
        self.projection = camera_projection.CameraProjection()
        
        rospack = rospkg.RosPack()
        # self.model_path = rospack.get_path('lane_detection') + '/models/competition_model_4c_128.pt'
        self.model_path = rospack.get_path('lane_detection') + '/models/best.pt'

        # Get the parameter to decide between deep learning and classical
        self.classical_mode = rospy.get_param('/lane_detection_inference/lane_detection_mode')
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
            self.Inference = YOLO(self.model_path)
            # self.Inference = Inference(self.model_path, False)

            rospy.loginfo("Lane Detection node initialized with DEEP LEARNING...\nCUDA status: %s ", torch.cuda.is_available())

        # self.hack = cv2.imread(r'/home/ammarvora/utra/caffeine-ws/src/Caffeine/cv/lane_detection/src/lane.png')

        # print(self.hack.shape)


        
    def run(self):
        rospy.Subscriber("/image", Image, self.process_image)
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
            # input_img = self.hack
            
            # input_img = cv2.resize(raw.shape[1], raw.shape[0])
            
            # cv2.imwrite(r'/home/ammarvora/utra/caffeine-ws/src/Caffeine/cv/lane_detection/src' + 'frame.png', input_img)
            # Do model inference 
            output = None
            mask = None

            if self.classical_mode:
                output = self.lane_detection(input_img)

                mask = np.where(output > 0.5, 1., 0.)
                mask = mask.astype(np.uint8)
                mask = cv2.resize(mask, (330, 180))

            else:
                # output = self.Inference.inference(input_img)
                # input_img = cv2.resize(input_img, (330, 180))
                cv2.rectangle(input_img, (0,0), (input_img.shape[1],int(input_img.shape[0] / 10)), (0,0,0), -1) 
                # cv2.imwrite(r'/home/tsyh/Downloads/test.jpg', input_img.squeeze())
            
                output = self.Inference(input_img)
                confidence_threshold = 0.5
                # number_masks = sum(1 for box in results[0].boxes if float(box.conf) > confidence_threshold)
                # print("number masks: ", number_masks)

                labels = {}
                output_image = np.zeros_like(input_img[:,:,0], dtype=np.uint8)

                if output[0].masks:
                    for k in range(len(output[0].masks)):
                        mask = np.array(output[0].masks[k].data.cpu() if torch.cuda.is_available() else output[0].masks[k].data)  # Convert tensor to numpy array
                        label = output[0].names[int(output[0].boxes[k].cls)]

                        if label not in labels:
                            labels[label] = np.zeros((180,330), dtype=np.uint8)

                        if float(output[0].boxes[k].conf) > confidence_threshold:  # Check confidence level
                            if label == 'lane':
                                img = np.where(mask > 0.5, 255, 0).astype(np.uint8)
                                img = cv2.resize(img.squeeze(), (output_image.shape[1], output_image.shape[0]))
                                output_image = np.maximum(output_image, img)

                            resize_mask = np.where(mask > 0.5, 1., 0.).astype(np.uint8)
                            resize_mask = cv2.resize(resize_mask.squeeze(), (330, 180))

                            labels[label] = np.maximum(labels[label], resize_mask)
                output = output_image
                mask = labels['lane'] if 'lane' in labels else np.zeros((330,180), dtype=np.uint8)



            # mask = np.where(output > 0.5, 1., 0.)
            # mask = mask.astype(np.uint8)
            # mask = cv2.resize(mask, (330, 180))

            # Publish to /cv/model_output
            img_msg = self.bridge.cv2_to_imgmsg(output, encoding='passthrough')
            img_msg.header.stamp = data.header.stamp
            # img_msg.header.stamp = data.header.stamp
            if img_msg is not None:
                self.pub_raw.publish(img_msg)
            

            '''The following code is needed for virtual layers'''
            rows = np.where(mask==1)[0].reshape(-1,1)
            cols = np.where(mask==1)[1].reshape(-1,1)
            lane_table = np.concatenate((cols,rows),axis=1)

            # print(lane_table)

            # ta = time.time()
            projected_lanes = self.projection(lane_table)
            # tb = time.time()

            # print(f'PROJECTION FPS: {1 / (tb - ta)}')

            # Build the message
            lane_msg = FloatList()
            pts_msg = []

            for pt in projected_lanes:

                pt_msg = Point()
                pt_msg.x = pt[0]
                pt_msg.y = pt[1]
                pt_msg.z = pt[2]

                pts_msg.append(pt_msg)
            lane_msg.elements = pts_msg


            msg_header = Header(frame_id='left_camera_link_optical')
            msg = FloatArray(header=msg_header, lists=[lane_msg])
            msg.header.stamp = data.header.stamp
            self.pub.publish(msg)

                


if __name__ == '__main__':
    wrapper = CVModelInferencer()
    wrapper.run()