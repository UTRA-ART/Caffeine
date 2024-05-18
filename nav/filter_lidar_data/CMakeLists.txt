cmake_minimum_required(VERSION 3.0.2)
project(filter_lidar_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  message_filters
  geodesy
  laser_geometry)
find_package(PCL REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    std_msgs
    message_filters
    geodesy)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ignore_lidar_node src/ignore_lidar_node.cpp)
target_link_libraries(ignore_lidar_node ${catkin_LIBRARIES})

install(TARGETS ignore_lidar_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(dual_lidar_filter_node src/dual_lidar_filter_node.cpp)
target_link_libraries(dual_lidar_filter_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS dual_lidar_filter_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})