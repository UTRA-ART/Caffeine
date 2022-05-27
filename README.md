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

### Install IMU dependencies ###
Provides the [phidgets_imu](http://wiki.ros.org/phidgets_imu) package dependencies which is used for publishing data from a phidget IMU.
```
sudo apt-get install ros-melodic-imu-transformer
sudo apt-get install ros-melodic-imu-filter-madgwick
```

### Install Hector Gazebo Plugins ###
Provides the [Hector Gazebo Plugins](http://wiki.ros.org/hector_gazebo_plugins) package for our GPS
```
sudo apt-get install ros-melodic-hector-gazebo-plugins
```

### Install IGVC World ###
Custom built world(s) representing the IGVC competition can be found in the [`/worlds`](./worlds) package. To install them for use in the Gazebo simulator, run the `./install_models.sh` script found in the `/worlds/models` folder.
> **NOTE:** The install script copies specific contents of `/worlds/models` to `~/.gazebo/models`

### Install ONNXRuntime ###
To perform inference, we leverage the onnxruntime's C++ API. To run inference,
a NVIDIA card capable of using CUDA is required.

First install NVIDIA drivers and CUDA. The steps roughly from the steps from [here](https://gist.github.com/mcvarer/30041141c8fe70ea5fe13f839330bc5a). We assume that there is no NVIDIA driver installation on the system. We highly recommend following an online guide as the specific details may vary.
```
sudo apt update && sudo apt upgrade
sudo apt-get install g++ freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libglu1-mesa libglu1-mesa-dev
sudo add-apt-repository ppa:graphics-drivers/ppa

# Distro = ubuntu1804 or ubuntu2004 (depends on version installed)
# Arch = x86_64
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/$distro/$arch/3bf863cc.pub
# Example: sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub

# Variables follow from above
echo "deb https://developer.download.nvidia.com/compute/cuda/repos/$distro/$arch /" | sudo tee /etc/apt/sources.list.d/cuda.list
sudo apt-get update

# Pick CUDA version; for example CUDA 11.X, where X is the version
sudo apt install cuda-11-X cuda-drivers

# Prepare paths, where X is the version
echo 'export PATH=/usr/local/cuda-11.X/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.X/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
sudo ldconfig

# Reboot machine

# Verify install
nvidia-smi
nvcc -V
```


Afterwards, install onnxruntime by following a comment on an GitHub issue
[here](https://github.com/microsoft/onnxruntime/issues/3124#issuecomment-676239644).
This is required because onnxruntime currently does not have good CMake
integration merged yet.

Also, make sure you have cuDNN installed. This can be installed using:
```
sudo apt install libcudnn8-dev
```

## Install geodesy ##
Provides the [Geodesy] (http://wiki.ros.org/geodesy) package to transform between GPS Coordinates (lat/lon) and UTM Coordinates.
```
sudo apt-get install ros-melodic-geodesy
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
