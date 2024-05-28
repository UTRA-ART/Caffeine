cmake_minimum_required(VERSION 3.0.2)
project(motor_odom)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2 
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Create exectuable
add_executable(cmd_vel_to_odom src/cmd_vel_to_odom.cpp)
target_link_libraries(cmd_vel_to_odom ${catkin_LIBRARIES})
