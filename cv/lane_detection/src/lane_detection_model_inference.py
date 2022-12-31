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

class CVModelInferencer:
    def __init__(self):
        rospy.init_node('lane_detection_model_inference')
        
        self.pub = rospy.Publisher('cv/lane_detections', FloatArray, queue_size=10)
        self.pub_raw = rospy.Publisher('cv/model_output', Image, queue_size=10)

        rospy.loginfo("Node initialized")

        self.bridge = CvBridge()
        self.projection = camera_projection.CameraProjection()
        
        rospack = rospkg.RosPack()
        model_path = rospack.get_path('lane_detection') + '/models/unet_v1.onnx'

        self.model = onnx.load(model_path)
        onnx.checker.check_model(self.model)

        self.ort_session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])

    def run(self):
        rospy.Subscriber("image", Image, self.process_image)
        rospy.spin()

    def process_image(self, data):
        rospy.loginfo("Triggered one image topic callback")
        if data == []:
            return
            
        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        if raw is not None:
            img = get_input(raw.copy())

            # Do model inference 
            output = self.ort_session.run(None, {'Inputs': img.astype(np.float32)})[0][0][0]
            mask = np.where(output > 0.5, 1., 0.)
            mask = mask.astype(np.uint8)

            # Publish to /cv/model_output
            img_msg = self.bridge.cv2_to_imgmsg(mask*255, encoding='passthrough')
            if img_msg is not None:
                self.pub_raw.publish(img_msg)
            
            # Publish to /cv/lane_detection
            lanes = fit_lanes(mask)
            if lanes is not None:
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

            # toshow = np.concatenate([cv2.resize(raw, (330, 180)), np.tile(mask[...,np.newaxis]*255, (1, 1, 3))], axis=1).astype(np.uint8)
            # cv2.imshow("image", toshow)
            # cv2.waitKey(1)

def find_edge_channel(img):
    edges_mask = np.zeros((img.shape[0],img.shape[1]),dtype=np.uint8)
    width = img.shape[1]
    height = img.shape[0]

    gray_im = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # gray_im = cv2.GaussianBlur(gray_im,(3,3),0)
    # Separate into quadrants
    med1 = np.median(gray_im[:height//2,:width//2])
    med2 = np.median(gray_im[:height//2,width//2:])
    med3 = np.median(gray_im[height//2:,width//2:])
    med4 = np.median(gray_im[height//2:,:width//2])

    l1 = int(max(0,(1-0.205)*med1))
    u1 = int(min(255,(1+0.205)*med1))
    e1 = cv2.Canny(gray_im[:height//2,:width//2],l1,u1)

    l2 = int(max(0,(1-0.205)*med2))
    u2 = int(min(255,(1+0.205)*med2))
    e2 = cv2.Canny(gray_im[:height//2,width//2:],l2,u2)

    l3 = int(max(0,(1-0.205)*med3))
    u3 = int(min(255,(1+0.205)*med3))
    e3 = cv2.Canny(gray_im[height//2:,width//2:],l3,u3)

    l4 = int(max(0,(1-0.205)*med4))
    u4 = int(min(255,(1+0.205)*med4))
    e4 = cv2.Canny(gray_im[height//2:,:width//2],l4,u4)

    # Stitch the edges together
    edges_mask[:height//2,:width//2] = e1
    edges_mask[:height//2,width//2:] = e2
    edges_mask[height//2:,width//2:] = e3
    edges_mask[height//2:,:width//2] = e4

    edges_mask_inv = cv2.bitwise_not(edges_mask)

    return edges_mask, edges_mask_inv


def get_input(frame):
    #frame = cv2.resize(frame,(1280,720),interpolation=cv2.INTER_AREA)
    frame_copy = np.copy(frame)

    gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)
    gradient_map = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=-1) # Gradient map along x
    #         gradient_map = cv2.Laplacian(gray, cv2.CV_64F)
    gradient_map = np.uint8(np.absolute(gradient_map))
    test_edges, test_edges_inv = find_edge_channel(frame_copy)


    frame_copy = np.zeros((gray.shape[0],gray.shape[1],4),dtype=np.uint8)   
    frame_copy[:,:,0] = gray
    frame_copy[:,:,1] = test_edges
    frame_copy[:,:,2] = test_edges_inv
    frame_copy[:,:,3] = gradient_map

    frame_copy = cv2.resize(frame_copy, (256, 160))



    input = (frame_copy/255.).transpose(2,0,1).reshape(1,4,160,256)
    return input


if __name__ == '__main__':
    wrapper = CVModelInferencer()
    wrapper.run()