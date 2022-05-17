cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport
                                        sensor_msgs nodelet)

# Find system dependencies
find_package(onnxruntime REQUIRED)
find_package(OpenCV REQUIRED)

# Generate cmake config files via catkin_package
catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  sensor_msgs
  cv_bridge
  image_transport
  DEPENDS
  onnxruntime
  OpenCV)

# Global include of necessary header files
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
                    ${onnxruntime_INCLUDE_DIRS})

# Build executables
add_library(lane_detection_inferencer src/lane_detection_inferencer.cpp)
target_link_libraries(lane_detection_inferencer ${onnxruntime_LIBRARIES}
                      ${OpenCV_LIBRARIES})

add_executable(lane_detection_inference_node
               src/lane_detection_inference_node.cpp)
target_link_libraries(lane_detection_inference_node ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES} lane_detection_inferencer)

# Install
install(TARGETS lane_detection_inference_node lane_detection_inferencer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
