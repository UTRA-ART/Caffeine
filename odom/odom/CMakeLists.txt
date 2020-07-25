cmake_minimum_required(VERSION 3.0.2)
project(odom)

find_package(catkin REQUIRED COMPONENTS
  robot_localization
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

