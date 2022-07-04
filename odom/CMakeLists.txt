cmake_minimum_required(VERSION 3.0.2)
project(odom)

find_package(catkin REQUIRED COMPONENTS
  robot_localization
  rospy
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

generate_messages(
  DEPENDENCIES
  odom
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/zero_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

