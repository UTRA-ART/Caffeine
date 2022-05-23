#!/usr/bin/env python2
from audioop import avg
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

        avg_depth_vals = np.ma.mean(self.averages, axis=0)
        avg_depth_vals = interpolate_nans(avg_depth_vals)

        output = {}
        for i, _y in enumerate(range(0, 180, 30)):
            for j, _x in enumerate(range(0, 330, 30)):
                for y in range(_y, _y+30):
                    for x in range(_x, _x+30):
                        output[str((x, y))] = avg_depth_vals[i, j]
        
        rospack = rospkg.RosPack()
        save_path = rospack.get_path('lane_detection') + '/config/depth_vals.json'
        print("Depth values recorded. Saved to", save_path)
        json.dump(output, open(save_path, 'w'), indent=4)

    def get_depths(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')  
        depth_values = np.ma.masked_invalid(cv2.resize(img, (330, 180)))

        to_add = []
        for _y in range(0, 180, 30):
            _xtoadd = []
            for _x in range(0, 330, 30):
                _xtoadd += [np.ma.mean(depth_values[_y:_y+30, _x:_x+30])]
            to_add += [_xtoadd]
        self.averages += [to_add]

        # depth_legend = np.tile(depth_values, (25, 1)).T
        # scale = 255 / np.max(np.ma.masked_invalid(img))
        # copy = img.copy()
        # copy[:,:25] = depth_legend
        # cv2.imwrite('/home/spencer/Documents/ART/caffeine-ws/' + 'DELETE.PNG', copy * scale)

def interpolate_nans(X):
    """Overwrite NaNs with column value interpolations."""
    for j in range(X.shape[0]):
        mask_j = np.isnan(X[j,:])
        X[j,mask_j] = np.interp(np.flatnonzero(mask_j), np.flatnonzero(~mask_j), X[j, ~mask_j])
    return X

if __name__ == '__main__':
    wrapper = ZedWrapperServer()
    wrapper.run()