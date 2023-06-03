import json 
import os 

import cv2
import numpy as np
import rospy
from image_geometry import PinholeCameraModel
import rospkg

from sensor_msgs.msg import CameraInfo
import math
class CameraProjection:
    def __init__(self):
        rospack = rospkg.RosPack()
        save_path = rospack.get_path('cv') + '/config/depth_vals_fixed.json'
        if os.path.exists(save_path):
            print("Using depth map values from", save_path)
            # self.depth_map = json.load(open(save_path, 'r'))
            self.depth_map = None
        else:
            self.depth_map = self.gen_depth_to_y_map(1, 4)


        rospack = rospkg.RosPack()
        hard_dir = rospack.get_path('cv') + '/config/depth_sim3.npy'
        depth_matrix = np.load(hard_dir)
        self.depth_values = np.nan_to_num(cv2.resize(depth_matrix, (330, 180)), nan=10000.)



        # self.depth_values = []
        # self.keys = {}
        # for i, key in enumerate(self.depth_map):
        #     self.keys[key] = i
        #     self.depth_values += [self.depth_map[key]]
        # self.depth_values = np.array(self.depth_values)

        camera_info = rospy.wait_for_message('/zed/zed_node/left/camera_info', CameraInfo)

        # Update camera intrinsics to account for the resized new images
        scale_x = 330.0/camera_info.width
        scale_y = 180.0/camera_info.height

        camera_info.width = 330
        camera_info.height = 180

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

        self.focalx = K[0]
        self.focaly = K[5]
        self.cx = K[2]
        self.cy = K[6]




        self.camera = PinholeCameraModel()
        print(camera_info)

        self.camera.fromCameraInfo(camera_info)


    def __call__(self, pts):
        '''
        [[x, y], [x, y]]
        '''
        return self.project_2D_to_3D_camera(pts)

    def convert_from_uvd(self, u, v, d):
        # d *= self.pxToMetre
        # d *= 0.10
        x_over_z = (self.cx - u) / self.focalx
        y_over_z = (self.cy - v) / self.focaly
        z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
        x = x_over_z * z
        y = y_over_z * z
        return [x, y, z]

    def project_2D_to_3D_camera(self, pts):
        ''' Given the pixel image coordinates and a constant mapping of depth to y-values,
            calculate the resulting projection of points in the camera coordinate frame.
            Requires the focal lengths in x and y and the optical centers
        '''
        points = []
        for i in range(len(pts)):
            if pts[i][0] < 0 or pts[i][1] < 0 or pts[i][0] >= 330 or pts[i][1] >= 180:
                continue
            vec = self.camera.projectPixelTo3dRay((pts[i][0], pts[i][1]))
            # z_val = self.depth_map[str((pts[i][0], pts[i][1]))]

            z_val = self.depth_values[pts[i][1], pts[i][0]]
            if z_val > 10:
                continue
            point3D = [vec[i] * z_val/np.linalg.norm(vec) for i in range(3)]
            points += [point3D]

            # z_val *= pts[i][0] * 0.1 + 1

            # theta = -0.593412 # 34 degrees in radians
            # theta = 0
            # rY = np.array([
            #     np.array([1, 0., 0]),
            #     np.array([0., math.cos(theta), -math.sin(theta)]),
            #     np.array([0, math.sin(theta), math.cos(theta)]),

            # ], dtype=np.float32)

            # point3D = np.array(point3D, dtype=np.float32)
            # point3D = np.matmul(rY, point3D)
            # point3D = rY * point3D
            # print(f'BEFORE: {z_val}, AFTER: {point3D[2]} ')

            # point3D = self.convert_from_uvd(pts[i][1], pts[i][0], z_val)

            # point3D = [point3D[2], -point3D[0], -point3D[1]]
            # print(point3D)

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
