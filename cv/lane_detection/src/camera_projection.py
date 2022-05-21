import json 

import cv2
import numpy as np
import rospkg


class CameraProjection:
    def __init__(self):
        rospack = rospkg.RosPack()
        save_path = rospack.get_path('lane_detection') + '/config/depth_vals.json'
        self.depth_to_y_map = self.load_depth_to_y_map(save_path)


    def project_2D_to_3D_camera(self, x_array, y_array, f_x, f_y, o_x, o_y):
        ''' Given the pixel image coordinates and a constant mapping of depth to y-values,
            calculate the resulting projection of points in the camera coordinate frame.
            Requires the focal lengths in x and y and the optical centers
        '''
        x_cam_coords = []
        y_cam_coords = []
        z_cam_coords = []

        for x,y in zip(x_array,y_array):
            x_project = (x - o_x)/f_x*self.depth_to_y_map[y]
            y_project = (y - o_y)/f_y*self.depth_to_y_map[y]
            z_project = self.depth_to_y_map[y]

            x_cam_coords.append(x_project)
            y_cam_coords.append(y_project)
            z_cam_coords.append(z_project)

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

    def gen_depth_to_y_map(self, min_z, max_z, num_y_pixels):
        ''' Create the constant depth to y map '''

        depth_to_y_map = []
        delta_z = (max_z-min_z)/num_y_pixels

        for n,y in enumerate(range(num_y_pixels-1,-1,-1),1):
            depth_to_y_map[y] = delta_z*n

        return depth_to_y_map

    def load_depth_to_y_map(self, file_name):
        values = json.load(open(file_name, 'r'))
        return values

