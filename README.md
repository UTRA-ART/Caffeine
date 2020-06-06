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

### Install husky-desktop and husky-simulator Packages ###
Husky is a robot that can be simulated in Gazebo and provides us with a few pre-built worlds.
```
sudo apt-get install ros-melodic-husky-desktop
sudo apt-get install ros-melodic-husky-simulator
```

### Install the Navigation Package ###
Provides the [Navigation Stack](http://wiki.ros.org/navigation) which is used for autonomous navigation.
```
sudo apt-get install ros-melodic-navigation
```

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

---
<p align="center">
<img src="https://raw.githubusercontent.com/UTRA-ART/SLAM/dev/docs/res/utra-logo.png" alt="UTRA logo" width="200"/>
</p>
<p align = "center"><b>University of Toronto Robotics Association</b></p>
<p align = "center">Autonomous Rover Team</p>
