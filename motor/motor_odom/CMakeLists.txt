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

# add_executable(wheel_odom_pub src/hall_effect_odom_pub.cpp)
# target_link_libraries(wheel_odom_pub ${catkin_LIBRARIES})

add_executable(wheel_odom_pub src/odom_pub2.cpp)
target_link_libraries(wheel_odom_pub ${catkin_LIBRARIES})