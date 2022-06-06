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
  cv
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(virtual_layers_local src/virtual_layer_local.cpp)
add_library(virtual_layers_global src/virtual_layer_global.cpp)
add_dependencies(virtual_layers_local ${catkin_EXPORTED_TARGETS})
add_dependencies(virtual_layers_global ${catkin_EXPORTED_TARGETS})

## Mark cpp header files for installation

