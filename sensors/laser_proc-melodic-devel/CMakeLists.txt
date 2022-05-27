cmake_minimum_required(VERSION 3.0.2)
project(laser_proc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs rosconsole nodelet pluginlib)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_proc_library laser_publisher laser_transport laser_proc_ROS LaserProcNodelet
  CATKIN_DEPENDS roscpp sensor_msgs rosconsole nodelet
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Library that does actual conversion
add_library(laser_proc_library src/LaserProc.cpp)
target_link_libraries(laser_proc_library ${catkin_LIBRARIES})

## Publisher library
add_library(laser_publisher src/LaserPublisher.cpp)
target_link_libraries(laser_publisher laser_proc_library ${catkin_LIBRARIES})

## Factory for making LaserPublishers
add_library(laser_transport src/LaserTransport.cpp)
target_link_libraries(laser_transport laser_publisher ${catkin_LIBRARIES})

## ROS Library
add_library(laser_proc_ROS src/LaserProcROS.cpp)
target_link_libraries(laser_proc_ROS laser_transport ${catkin_LIBRARIES})

## Nodelet wrapper
add_library(LaserProcNodelet src/LaserProcNodelet.cpp)
target_link_libraries(LaserProcNodelet laser_proc_ROS ${catkin_LIBRARIES})

## Node wrapper
add_executable(laser_proc src/laser_proc.cpp)
target_link_libraries(laser_proc laser_proc_ROS ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS laser_proc_library laser_publisher laser_transport laser_proc_ROS LaserProcNodelet laser_proc
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
