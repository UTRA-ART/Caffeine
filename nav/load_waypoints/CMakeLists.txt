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
    INCLUDE_DIRS
    CATKIN_DEPENDS
    roscpp
    nav_msgs
    sensor_msgs
    std_msgs
    message_filters
    geodesy)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ramp_navigate src/ramp_navigate.cpp)
target_link_libraries(ramp_navigate ${PCL_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ramp_navigate
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
