# Description #

This folder contains files relevant to the description of caffeine (e.g. model files, urdf files, rviz files and more) as well as files to launch caffeine in gazebo.

## Usage ## 

To use this package, use the following command:
```
roslaunch description simulate_robot.launch
```
This launches the robot (including the tf tree & ZED camera emulation) in the IGVC world inside the Gazebo simulator. The odom and twist_mux packages are also launched.

In order to launch a GUI for Gazebo, use this command:
```
roslaunch description simulate_robot.launch use_gui:=true
```

This simulate_robot launch file also supports arguments for switching the robot between the following worlds:
- IGVC test
- IGVC plain
- IGVC with ramps
- IGVC with walls
- IGVC full (default)

The default world is the world that is expected to be in the competition.

In order to launch rviz, this command should be used:
```
roslaunch description start_rviz.launch
```

## Relevant Packages & Topics ##

The following internal packages are relevant:
- odom
- twist_mux

See package.xml and CMakeLists.txt for any external dependancies. 

This package outputs a number of topics. The ones relevant to using the robot  are summarized below.

### cmd_vel ### 
This topic allows for issuing velocity commands, in the form of a twist, to the robot.

### zed ### 
The topics associated with zed provides camera functionality, such as the raw images, disparity map and more.

### scan ### 
This topic provides LIDAR data.

### imu ### 
This topic provides IMU data.

### gps ### 
This provides GPS data.

### odom ### 
This topic provides odometry data. Note: this data should not be used, since the odom package is used to fuse multiple sensors to provide a better odometry estimate. Instead odometry/local and odometry/gps should be used. 

