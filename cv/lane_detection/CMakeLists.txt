cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  rospy 
  std_msgs 
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
  FloatList.msg
  FloatArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

# Generate cmake config files via catkin_package
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime
)
 