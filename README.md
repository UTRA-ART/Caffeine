# Caffeine on ROS #

This project runs on ROS noetic for Ubuntu 20.04 LTS. Caffeine is a robot being built to compete in IGVC.

## Setting up the ROS Environment ##

### Install Ubuntu 20.04 LTS ###
This is dependent on what OS and computer is currently used. The [wiki](https://github.com/UTRA-ART/Caffeine/wiki) has a section on how to dual boot.

### Install ROS Noetic ###
The following instructions are taken from the ROS wiki [install noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) page:
```
# Configure Ubuntu repositories
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted

# Set up sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up keys
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Noetic Desktop-Full Install
sudo apt install ros-noetic-desktop-full

# Environment setup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Dependencies for buiding packages
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

### Install catkin_tools ###
Provides the tools for working with the catkin build system and workspaces. More information can be found here: [Docs](https://catkin-tools.readthedocs.io/en/latest/). We use this to build packages with `catkin build`.
```
sudo apt-get install python3-catkin-tools
```

### Install the Navigation Package ###
Provides the [Navigation Stack](http://wiki.ros.org/navigation) package which is used for autonomous navigation.
```
sudo apt-get install ros-noetic-navigation
```

### Install RVIZ Plugins ###
Provides the [RVIZ Sensor Plugins](http://wiki.ros.org/rviz_imu_plugin) package which is used for RVIZ data visualization
```
sudo apt-get install ros-noetic-rviz-imu-plugin
```

### Install Robot Localization ###
Provides the [Robot Localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) package which is used for localizing the robot.
```
sudo apt-get install ros-noetic-robot-localization
```

### Install DWA Local Planner ###
Provides the [DWA Local Planner](http://wiki.ros.org/dwa_local_planner) package which is used for local planning in navigation.
```
sudo apt-get install ros-noetic-dwa-local-planner
```

### Install RTAB Map ###
Provides the [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) package which is used for performing RGB-D SLAM.
```
sudo apt-get install ros-noetic-rtabmap-ros
```

### Install IMU dependencies ###
Provides the [phidgets_imu](http://wiki.ros.org/phidgets_imu) package dependencies which is used for publishing data from a phidget IMU.
```
sudo apt-get install ros-noetic-imu-transformer
sudo apt-get install ros-noetic-imu-filter-madgwick
```

### Install Hector Gazebo Plugins ###
Provides the [Hector Gazebo Plugins](http://wiki.ros.org/hector_gazebo_plugins) package for our GPS
```
sudo apt-get install ros-noetic-hector-gazebo-plugins
```

### Other Dependencies
Provides the GPS conversion to better work with latitude and longitude values.
```
pip3 install utm
sudo apt-get install ros-noetic-geodesy
```

### Install IGVC World ###
Custom built world(s) representing the IGVC competition can be found in the [`/worlds`](./worlds) package. To install them for use in the Gazebo simulator, run the `./install_models.sh` script found in the `/worlds/models` folder.
> **NOTE:** The install script copies specific contents of `/worlds/models` to `~/.gazebo/models`

### Installing CV dependencies ###
The cv pipeline has several pip dependencies that need to be installed for both python 2 and  python 3. Run the following commands. 
```
pip3 install onnx onnxruntime opencv-python rospkg scikit-learn scipy
```

## Cloning this repository ##
Before cloning this repository, create a ROS workspace:
```
mkdir -p caffeine-ws/src
cd caffeine-ws
catkin build
```
After, clone this repository into the `/src` folder.

## Installing Cartographer (SLAM) ##
Cartographer provides Mapping and Localization services and requires building from source. Instructions are taken from the [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/) documentation.
```
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow

# Clone the Cartographer Repos into src folder
cd caffeine-ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

# Use rosdep to install Cartographer dependencies
sudo rosdep init # This will print an error if you have already executed it before, the error can be ignored
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y # Ignore warnings about libabseil

# Other dependencies
sudo apt-get install libceres-dev
sudo apt-get install liblua5.2-dev

# Install the abseil-cpp library
src/cartographer/scripts/install_abseil.sh

# Apply patch to make cartographer work with costmap
cd src/cartographer # Make sure you are in caffeine-ws/src/cartographer
git apply ../Caffeine/misc/cartographer_costmap.patch

# Build and install
cd ..
catkin build
```

## Cleaning the ROS Workspace ##
Every once in a while it is necessary to clear unnecesary logs that are saved from tests that have been run. These logs can quickly add up to the GB range, and can slow down ROS. To check how many logs you have run:

```
rosclean check
```

This should give you how much memory is consumed by logs. If nothing is returned you have no logs.

To delete logs run:

```
rosclean purge
```

## Verification ##
Here are the common commands we run that will verify that your setup is correct (after performing ```catkin_make``` in any terminal once and ```source devel/setup.bash``` in every terminal you use)
```
# Terminal 1: Gazebo (World+Robot Simulation With GUI)
roslaunch description simulate.launch use_gui:=true

# Terminal 2: RVIZ (Data Visualization)
roslaunch description view.launch

# Terminal 3: move_base (Navigation Stack)
roslaunch nav_stack move_base.launch
```
After running all 3 commands, set a 2d Nav Goal in RVIZ, and if your robot moves in both Gazebo and RVIZ, you are good

## Other Launch Files
```
# To launch the CV Pipeline
roslaunch cv pipeline.launch

# To activate auto-navigation 
roslaunch load_waypoints load_waypoints.launch 

# To use teleop to manually control caffeine's movement
roslaunch teleop_twist_keyboard keyboard_teleop.launch
```

## Notes ##

When running `simulate.launch`, an error saying that the `spawn_model` node failed will appear. This occurs because both the gazebo world and the urdf are loaded in the same `roslaunch` file (IGVC takes too long to load before model is spawned). The spawner will automatically retry and spawn Caffeine properly, so this error can be safely ignored.
> A solution is to spawn Caffeine only once the gazebo (IGVC) world has been loaded, but this requires a new `roslaunch` file and thus a new terminal - which is excessive at this point.

## Useful Commands ##
```
# Generates real-time flow diagram of the transform tree
rosrun rqt_tf_tree rqt_tf_tree

# Echos the tf transform from frame_1 -> frame_2
rosrun tf tf_echo /frame_1 /frame_2

# Generates real-time flow diagram of the topics, nodes, and the connections
rosrun rqt_graph rqt_graph 
```

---
<p align="center">
<img src="https://raw.githubusercontent.com/UTRA-ART/SLAM/dev/docs/res/utra-logo.png" alt="UTRA logo" width="200"/>
</p>
<p align = "center"><b>University of Toronto Robotics Association</b></p>
<p align = "center">Autonomous Rover Team</p>
