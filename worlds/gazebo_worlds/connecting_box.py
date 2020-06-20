import sys
import math

# PURPOSE: To calculate where the next wall to generate for the igvc 
# wall based on the last wall desired yaw, and new box dimensions

# Notes: No roll or pitch or height for any wall for xyzrpy, 
# and no need for yz for dimensions

# Sample input: python connecting_box.py [6,2.0,0.1] [1.2,-22.6,0,0,0,0.07] [1.5,0.1,1] [0.36]
# --> No spaces                           (Old dims)        (Old pose)      (New dims) (New Yaw)

def sys_to_list(sys_idx):
    return [float(i) for i in sys.argv[sys_idx].replace(" ", "").replace("[", "").replace("]","").split(",")]

def gen_new_xy(prev_dim_x, prev_pose_x, prev_pose_y, prev_pose_yaw, new_dim_x, new_pose_yaw):
    link_coord = [prev_pose_x - 0.5*prev_dim_x*math.cos(prev_pose_yaw), prev_pose_y - 0.5*prev_dim_x*math.sin(prev_pose_yaw)]
    new_center_coord = [link_coord[0] - 0.5*new_dim_x*math.cos(new_pose_yaw), link_coord[1] - 0.5*new_dim_x*math.sin(new_pose_yaw)]
    return new_center_coord

if __name__ == "__main__":
    # Check correct number of arguments
    if len(sys.argv) != 5:
        print("Incorrect Number of Arguments")
        sys.exit()

    # Convert system arguments to list
    prev_dim = sys_to_list(1)           # Box Dimension Format: XYZ
    prev_pose = sys_to_list(2)          # Box Pose Format: xyzrpy
    new_dim = sys_to_list(3)            # Box Dimension Format: XYZ
    new_yaw = sys_to_list(4)            # Box Desired Yaw
    
    # Check correct number of values in each argument
    if len(prev_dim) != 3:
        print("Wrong prev link dimension format")
        sys.exit()

    if len(prev_pose) != 6:
        print("Wrong prev link pose format")
        sys.exit()

    if len(new_dim) != 3:
        print("Wrong new link dimension format")
        sys.exit()
    
    if len(new_yaw) != 1:
        print("Wrong new link yaw format")
        sys.exit()
    else:
        new_yaw = new_yaw[0]

    # Always connect old box's neg y face with new box's pos y face
    new_center = gen_new_xy(prev_dim[0], prev_pose[0], prev_pose[1], prev_pose[5], new_dim[0], new_yaw)
    print("Given Yaw: ", new_yaw)
    print("New X (pose): ", new_center[0])
    print("New Y (pose): ", new_center[1])
