cmake_minimum_required(VERSION 3.0.2)
project(nav_stack)

find_package(catkin REQUIRED COMPONENTS
  move_base
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

add_definitions(-DROSCPP_USE_TCP_NODELAY)

