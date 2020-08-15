cmake_minimum_required(VERSION 2.8.3)
project(nav_goal)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  rospy
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
catkin_python_setup()

## Sets up Python package and installs it in the correct location
catkin_install_python(PROGRAMS bin/movebase_client DESTINATION ${PROJECT_NAME}/bin)
catkin_install_python(PROGRAMS bin/movebase_client_map DESTINATION ${PROJECT_NAME}/bin)

