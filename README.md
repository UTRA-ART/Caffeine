# Caffeine on ROS #

This project runs on ROS melodic for Ubuntu 18.04 LTS. Caffeine is a robot being built to compete in IGVC.

## Setting up the ROS Environment ##

### Install Ubuntu 18.04 LTS ###
This is dependent on what OS and computer is currently used. The [wiki](https://github.com/UTRA-ART/Caffeine/wiki) has a section on how to dual boot.

### Install ROS Melodic ###
The following instructions are taken from the ROS wiki [install melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) page:
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

# Melodic Desktop-Full Install
sudo apt install ros-melodic-desktop-full

# Environment setup
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Dependencies for buiding packages
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

### Install the Navigation Package ###
Provides the [Navigation Stack](http://wiki.ros.org/navigation) package which is used for autonomous navigation.
```
sudo apt-get install ros-melodic-navigation
```

### Install RVIZ Plugins ###
Provides the [RVIZ Sensor Plugins](http://wiki.ros.org/rviz_imu_plugin) package which is used for RVIZ data visualization
```
sudo apt-get install ros-melodic-rviz-imu-plugin
```

### Install Robot Localization ###
Provides the [Robot Localization](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package which is used for localizing the robot.
```
sudo apt-get install ros-melodic-robot-localization
```

### Install RTAB Map ###
Provides the [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) package which is used for performing RGB-D SLAM.
```
sudo apt-get install ros-melodic-rtabmap-ros
```

### Install Hector Gazebo Plugins ###
Provides the [Hector Gazebo Plugins](http://wiki.ros.org/hector_gazebo_plugins) package for our GPS
```
sudo apt-get install ros-melodic-hector-gazebo-plugins
```

### Install IGVC World ###
Custom built world(s) representing the IGVC competition can be found in the [`/worlds`](./worlds) package. To install them for use in the Gazebo simulator, run the `./install_models.sh` script found in the `/worlds/models` folder.
> **NOTE:** The install script copies specific contents of `/worlds/models` to `~/.gazebo/models`

## Cloning this repository ##
Before cloning this repository, create a ROS workspace:
```
mkdir -p caffeine-ws/src
cd caffeine-ws
catkin_make
```
After, clone this repository into the `/src` folder.

### Cleaning the ROS Workspace ###
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

## Notes ##

When running `simulate.launch`, an error saying that the `spawn_model` node failed will appear. This occurs because both the gazebo world and the urdf are loaded in the same `roslaunch` file (IGVC takes too long to load before model is spawned). The spawner will automatically retry and spawn Caffeine properly, so this error can be safely ignored.
> A solution is to spawn Caffeine only once the gazebo (IGVC) world has been loaded, but this requires a new `roslaunch` file and thus a new terminal - which is excessive at this point.

---
<p align="center">
<img src="https://raw.githubusercontent.com/UTRA-ART/SLAM/dev/docs/res/utra-logo.png" alt="UTRA logo" width="200"/>
</p>
<p align = "center"><b>University of Toronto Robotics Association</b></p>
<p align = "center">Autonomous Rover Team</p>
