cmake_minimum_required(VERSION 3.0.2)
project(float_array_to_pc2)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv pcl_conversions)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs cv pcl_conversions)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(float_array_to_pc2_node src/float_array_to_pc2_node.cpp)
target_link_libraries(float_array_to_pc2_node ${catkin_LIBRARIES})

install(TARGETS float_array_to_pc2_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
