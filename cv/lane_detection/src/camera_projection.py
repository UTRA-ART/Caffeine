import json 
import os 

import cv2
import numpy as np
import rospkg


class CameraProjection:
    def __init__(self):
        rospack = rospkg.RosPack()
        save_path = rospack.get_path('lane_detection') + '/config/depth_vals.json'
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


    def transform_cam_to_world(self, M_ext, x_array, y_array, z_array):
        ''' Transform your points from camera coordinates back to world coordinates
            This is done via reversing the rotation and then translating
            M_ext = |R T|   ====> Upper 3 x 3 matrix is the rotation
                    |0 1|   ====> Last col, first 3 rows is the translation
            Assuming that your R matrix is orthogonal, R_inv = R_T = transpose(R)
            The reverse of the translation is to simply add the -T vector to the points
            You should be fine in the sense of using 3D coordinates and not homogeneous
            (We aren't taking the inverse of M_ext)
            So, to reverse, do R_inv(transpose([X Y Z]_cam) - T) = transpose([X Y Z]_world)
        '''

        R_inv = np.array(M_ext[0:3,0:3]).T  # Assuming orthgonality
        T = np.array([M_ext[0:3,3]]).T

        x_world = []
        y_world = []
        z_world = []

        for x_c,y_c,z_c in zip(x_array,y_array,z_array):
            cam_coords = np.array([[x_c,y_c,z_c]]).T
            world_coords = np.matmul(R_inv,cam_coords - T)
            x_world.append(world_coords[0,0])
            y_world.append(world_coords[1,0])
            z_world.append(world_coords[2,0])

        return x_world, y_world, z_world


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