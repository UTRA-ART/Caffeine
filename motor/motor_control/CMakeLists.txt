cmake_minimum_required(VERSION 2.8.3)
project(motor_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy 
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Create exectuable
add_executable(cmd_vel_to_motor src/cmd_vel_to_motor.cpp)
target_link_libraries(cmd_vel_to_motor ${catkin_LIBRARIES})

# add_executable(motor_control_node src/motor_control.py)
# target_link_libraries(motor_control_node ${catkin_LIBRARIES})
