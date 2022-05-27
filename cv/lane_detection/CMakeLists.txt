cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  rospy 
  std_msgs 
  geometry_msgs
  )

# Generate cmake config files via catkin_package
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  )
  