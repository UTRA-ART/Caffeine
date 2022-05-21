#!/usr/bin/env python2
import struct
import sys
import json 

import cv2
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

'''
Generates an estimate for z values for each y value of a zed input image. Used by taking 
average stereo depth over time. To use, ensure the whole camera field of view is on a flat, 
feature heavy surface. Exports depth estimates to lane_detection/config/depth_vals.json.

Should be run everytime the camera is adjusted on Caffeine. zed_wrapper zed.launch must be running. 
'''

class ZedWrapperServer:
    def __init__(self):
        self.bridge = CvBridge()
        self.averages = []

    def run(self):
        rospy.init_node('zed_data_output')
        rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.get_depths)
        rospy.spin()

        avg_depth_vals = np.mean(self.averages, axis=0)
        
        rospack = rospkg.RosPack()
        save_path = rospack.get_path('lane_detection') + '/config/depth_vals.json'
        print("Depth values recorded. Saved to", save_path)
        json.dump(avg_depth_vals.tolist(), open(save_path, 'w'))

    def get_depths(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')  
        print(img.shape)
        depth_values = np.mean(np.ma.masked_invalid(img), axis=1)
        self.averages += [depth_values]

        # depth_legend = np.tile(depth_values, (25, 1)).T
        # scale = 255 / np.max(np.ma.masked_invalid(img))
        # copy = img.copy()
        # copy[:,:25] = depth_legend
        # cv2.imwrite('/home/spencer/Documents/ART/caffeine-ws/' + 'DELETE.PNG', copy * scale)



if __name__ == '__main__':
    wrapper = ZedWrapperServer()
    wrapper.run()