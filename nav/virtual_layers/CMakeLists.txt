cmake_minimum_required(VERSION 3.0.2)
project(virtual_layers)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs #for topic subscription
  cv_pkg
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(virtual_layers src/virtual_layer.cpp)
add_definitions(-DROSCPP_USE_TCP_NODELAY)

## Mark cpp header files for installation

