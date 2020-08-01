cmake_minimum_required(VERSION 3.0.2)
project(laser_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    laser_filters
)

catkin_package(
    CATKIN_DEPENDS laser_filters
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
)
