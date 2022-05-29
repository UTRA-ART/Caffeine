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

        camera_info = rospy.wait_for_message('/zed/zed_node/left/camera_info', CameraInfo)

        # Update camera intrinsics to account for the resized new images
        scale_x = 330.0/camera_info.width
        scale_y = 180.0/camera_info.height

        K = list(camera_info.K)
        K[0] = K[0] * scale_x
        K[2] = K[2] * scale_x
        K[4] = K[4] * scale_y
        K[5] = K[5] * scale_y
        camera_info.K = tuple(K)

        P = list(camera_info.P)
        P[0] = P[0] * scale_x
        P[2] = P[2] * scale_x
        P[3] = P[3] * scale_x
        P[5] = P[5] * scale_y
        P[6] = P[6] * scale_y
        P[7] = P[7] * scale_y
        camera_info.P = tuple(P)

        camera_info.roi.x_offset = int(camera_info.roi.x_offset * scale_x)
        camera_info.roi.y_offset = int(camera_info.roi.y_offset * scale_y)
        camera_info.roi.width = int(camera_info.roi.width * scale_x)
        camera_info.roi.height = int(camera_info.roi.height * scale_y)

        self.camera = PinholeCameraModel()

        self.camera.fromCameraInfo(camera_info)


    def __call__(self, pts):
        '''
        [[x, y], [x, y]]
        '''
        return self.project_2D_to_3D_camera(pts)


    def project_2D_to_3D_camera(self, pts):
        ''' Given the pixel image coordinates and a constant mapping of depth to y-values,
            calculate the resulting projection of points in the camera coordinate frame.
            Requires the focal lengths in x and y and the optical centers
        '''
        points = []
        for i in range(len(pts)):
            if pts[i][0] < 0 or pts[i][1] < 0:# or pts[i][0] >  or pts[i][1] < 0:
                continue
            vec = self.camera.projectPixelTo3dRay((pts[i][0], pts[i][1]))
            z_val = self.depth_map[str((pts[i][0], pts[i][1]))]
            point3D = [vec[i] * z_val/vec[2] for i in range(3)]
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
