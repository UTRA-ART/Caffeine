#!/usr/bin/env python3
import pothole_helpers
import rospy
from sensor_msgs.msg import Image

from cv.msg import FloatArray, FloatList 
from geometry_msgs.msg import Point
from cv_utils import camera_projection

from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

from cv_utils import camera_projection


import rospkg
import os



def read():
    print('HERE')
    rootpath = '/home/parallels/caffeine-ws/src/Caffeine/cv/pothole_detection/src/TestPothole/'
    outpath = '/home/parallels/caffeine-ws/src/Caffeine/cv/pothole_detection/src/OutputMask/'
    print(os.path.abspath(rootpath))
    for root, dirs, files in os.walk(rootpath, topdown=False):
        for name in files:
            test = cv2.imread(rootpath + name)
            masked = pothole_helpers.multi_thres(test)
          
            cv2.imwrite(outpath + name, masked)
            print(name)

i = 0
outpath = '/home/parallels/caffeine-ws/src/Caffeine/cv/pothole_detection/src/OutputMask/'

class CV2Pothole:
    def __init__(self):
        rospy.init_node('pothole_node')
        self.pub_raw = rospy.Publisher('poth_mask', Image, queue_size=10)
        self.pub = rospy.Publisher('potholes', Float32MultiArray, queue_size=10)
        
        self.bridge = CvBridge()
        # self.projection = camera_projection.CameraProjection()
        
        # rospack = rospkg.RosPack()

    def run(self):
        rospy.Subscriber("poth", Image, self.callback)
        
        print('run')
        
        # read()
        
        rospy.spin()

    def callback(self, data):
        print('call')
        if data == []:
            return	
	
        bridge = CvBridge()

        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        except CvBridgeError as e:
            print(e)	
	
        if img is None:
            return
        
        masked, boxes = pothole_helpers.multi_thres(img)
        
        # cv2.imwrite(outpath + name, masked)
        img_msg = bridge.cv2_to_imgmsg(masked, "mono8")
        self.pub_raw.publish(img_msg)
        
        
        boxs_msg = Float32MultiArray()
        boxs_msg.data = [item for box in boxes for item in box]
        
        self.pub.publish(boxs_msg)
        
        return masked
 


if __name__ == '__main__':
    node = CV2Pothole()
    node.run()



