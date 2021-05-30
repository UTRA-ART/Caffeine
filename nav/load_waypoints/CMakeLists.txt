cmake_minimum_required(VERSION 3.0.2)
project(load_waypoints)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  tf
  std_msgs
  sensor_msgs
  move_base_msgs
)

## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    load_waypoints  # Or other packages containing msgs
  )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/navigate_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
