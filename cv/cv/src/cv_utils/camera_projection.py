import json 
import os 

import cv2
import numpy as np
import rospy
from image_geometry import PinholeCameraModel
import rospkg

from sensor_msgs.msg import CameraInfo

class CameraProjection:
    def __init__(self):
        rospack = rospkg.RosPack()
        save_path = rospack.get_path('cv') + '/config/depth_vals.json'
        if os.path.exists(save_path):
            print("Using depth map values from", save_path)
            self.depth_map = json.load(open(save_path, 'r'))
        else:
            self.depth_map = self.gen_depth_to_y_map(1.5, 4)

        self.depth_values = []
        self.keys = {}
        for i, key in enumerate(self.depth_map):
            self.keys[key] = i
            self.depth_values += [self.depth_map[key]]
        self.depth_values = np.array(self.depth_values)

        camera_info = rospy.wait_for_message('/zed/zed_node/rgb/camera_info', CameraInfo)
        self.camera = PinholeCameraModel()
        
        # Update camera intrinsics to account for the resized new images. I think it's 720x1280 -> 180x330
        camera_info[0,0] = camera_info[0,0] * 180/720
        camera_info[0,2] = camera_info[0,2] * 180/720
        camera_info[1,1] = camera_info[1,1] * 330/1280
        camera_info[1,2] = camera_info[1,2] * 330/1280
        self.camera.fromCameraInfo(camera_info)


    def __call__(self, pts):
        return self.project_2D_to_3D_camera(pts[0], pts[1])


    def project_2D_to_3D_camera(self, x_array, y_array):
        ''' Given the pixel image coordinates and a constant mapping of depth to y-values,
            calculate the resulting projection of points in the camera coordinate frame.
            Requires the focal lengths in x and y and the optical centers
        '''
        points = []
        for i in range(len(x_array)):
            vec = self.camera.projectPixelTo3dRay((x_array[i], y_array[i]))
            z_val = self.depth_map[str((x_array[i], y_array[i]))]
            point3D = [vec[i] * z_val for i in range(3)]
            points += [point3D]

        return points


    def gen_depth_to_y_map(self, min_z, max_z):
        ''' Create the constant depth to y map '''

        depth_map = {}
        delta_z = (max_z-min_z)/6

        for i, _y in enumerate(range(150, -30, -30)):
            for _x in range(0, 330, 30):
                for y in range(_y, _y+30):
                    for x in range(_x, _x+30):
                        depth_map[str((x, y))] = delta_z*i

        for n,y in enumerate(range(180-1,-1,-1),1):
            depth_map[y] = delta_z*n

        return depth_map
