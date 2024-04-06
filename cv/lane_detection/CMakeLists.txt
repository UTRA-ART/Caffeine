cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp
  std_msgs 
  geometry_msgs
  message_generation
  tf
  )

add_message_files(
  FILES
  FloatList.msg
  FloatArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# Generate cmake config files via catkin_package
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  CATKIN_DEPENDS message_runtime
  roscpp
  tf2_ros
  tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(scan src/scan.cpp)
target_link_libraries(scan ${catkin_LIBRARIES})

install(TARGETS scan
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
