import json 
import os 

import cv2
import numpy as np
import rospkg


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


        '''
        Raw parameters (found in /usr/local/zed/settings)
        fx = 1400.13
        fy = 1400.13
        cx = 1188.06
        cy = 645.894

        image size: 360, 640, 3

        detination image size: 180, 330
        ratio: 0.5, 0.515625

        TODO: Remove estimates 
        '''

        self.fx = (360 + 640) / 2
        self.fy = (360 + 640) / 2
        self.ox = 640 / 2
        self.oy = 360 / 2


    def __call__(self, pts):
        return self.project_2D_to_3D_camera(pts[0], pts[1])


    def project_2D_to_3D_camera(self, x_array, y_array):
        ''' Given the pixel image coordinates and a constant mapping of depth to y-values,
            calculate the resulting projection of points in the camera coordinate frame.
            Requires the focal lengths in x and y and the optical centers
        '''
        x_cam_coords = []
        y_cam_coords = []
        z_cam_coords = []

        z_map_indeces = []
        for i in range(len(x_array)):
            z_map_indeces += [self.keys[str((x_array[i], y_array[i]))]]
        z_map_indeces = np.array(z_map_indeces)

        x_cam_coords = (x_array - self.ox) / self.fx * self.depth_values[z_map_indeces]
        y_cam_coords = (y_array - self.oy) / self.fy * self.depth_values[z_map_indeces]
        z_cam_coords = self.depth_values[z_map_indeces]

        return x_cam_coords, y_cam_coords, z_cam_coords


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