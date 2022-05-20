cmake_minimum_required(VERSION 3.0.2)
project(filter_lidar_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  message_filters
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    std_msgs
    message_filters)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ignore_lidar_node src/ignore_lidar_node.cpp)
target_link_libraries(ignore_lidar_node ${catkin_LIBRARIES})

install(TARGETS ignore_lidar_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})