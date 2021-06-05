# Caffeine on ROS #

This project runs on ROS melodic for Ubuntu 18.04 LTS. Caffeine is a robot being built to compete in IGVC.

## Setting up the ROS Environment ##

### Install Ubuntu 18.04 LTS ###
This is dependent on what OS and computer is currently used. The [wiki](https://github.com/UTRA-ART/Caffeine/wiki) has a section on how to dual boot.

### Install ROS Melodic ###
The following instructions are taken from the ROS wiki [install melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) page:
```
# Set up sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

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
Custom built world(s) representing the IGVC competition can be found in the [`/worlds`](./worlds) package. To install them for use in the Gazebo simulator, run the `install_models.sh` script found in the `/worlds/models` folder.
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
## Notes ##

When running `caffeine_gazebo.launch`, an error saying that the `spawn_model` node failed will appear. This occurs because both the gazebo world and the urdf are loaded in the same `roslaunch` file (IGVC takes too long to load before model is spawned). The spawner will automatically retry and spawn Caffeine properly, so this error can be safely ignored.
> A solution is to spawn Caffeine only once the gazebo (IGVC) world has been loaded, but this requires a new `roslaunch` file and thus a new terminal - which is excessive at this point.

---
<p align="center">
<img src="https://raw.githubusercontent.com/UTRA-ART/SLAM/dev/docs/res/utra-logo.png" alt="UTRA logo" width="200"/>
</p>
<p align = "center"><b>University of Toronto Robotics Association</b></p>
<p align = "center">Autonomous Rover Team</p>
