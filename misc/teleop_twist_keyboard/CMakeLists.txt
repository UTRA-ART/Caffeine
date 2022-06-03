cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_keyboard)

find_package(catkin REQUIRED COMPONENTS
   move_base
)

catkin_package(
   CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
   src/teleop_twist_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)