cmake_minimum_required(VERSION 3.0.2)
project(motor_odom)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  # geometry_msgs
  # nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
