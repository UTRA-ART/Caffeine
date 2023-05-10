cmake_minimum_required(VERSION 3.0.2)
project(odom)

find_package(catkin REQUIRED COMPONENTS
  robot_localization
  roscpp
  std_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS 
    robot_localization
    roscpp
    std_msgs 
    nav_msgs 
    geometry_msgs 
    tf2 
    tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(wheel_odom_pub src/odom_pub.cpp)
target_link_libraries(wheel_odom_pub ${catkin_LIBRARIES})