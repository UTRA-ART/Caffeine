cmake_minimum_required(VERSION 3.0.2)
project(urg_c)

# Set library locations to better handle if upstream locations change
set(LIBRARY_DIR current)
set(LIBRARY_INCLUDE_DIR ${LIBRARY_DIR}/include)
set(LIBRARY_SRC_DIR ${LIBRARY_DIR}/src)
set(LIBRARY_SAMPLE_DIR ${LIBRARY_DIR}/samples)

find_package(catkin REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${LIBRARY_INCLUDE_DIR}
  LIBRARIES liburg_c
  CATKIN_DEPENDS
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${LIBRARY_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

## Declare libraries
add_library(liburg_c ${LIBRARY_SRC_DIR}/urg_sensor.c
	                 ${LIBRARY_SRC_DIR}/urg_utils.c
	                 ${LIBRARY_SRC_DIR}/urg_debug.c
	                 ${LIBRARY_SRC_DIR}/urg_connection.c
	                 ${LIBRARY_SRC_DIR}/urg_ring_buffer.c
	                 ${LIBRARY_SRC_DIR}/urg_serial.c
	                 ${LIBRARY_SRC_DIR}/urg_serial_utils.c
	                 ${LIBRARY_SRC_DIR}/urg_tcpclient.c
	                 ${LIBRARY_SRC_DIR}/urg_time.c

)
if(NOT ANDROID)
	target_link_libraries(liburg_c -lrt -lm)
endif(NOT ANDROID)

add_library(open_urg_sensor ${LIBRARY_SAMPLE_DIR}/open_urg_sensor.c)
target_link_libraries(open_urg_sensor -lm liburg_c ${catkin_LIBRARIES})

## Declare samples
add_executable(angle_convert_test ${LIBRARY_SAMPLE_DIR}/angle_convert_test.c)
target_link_libraries(angle_convert_test liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(calculate_xy ${LIBRARY_SAMPLE_DIR}/calculate_xy.c)
target_link_libraries(calculate_xy liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(find_port ${LIBRARY_SAMPLE_DIR}/find_port.c)
target_link_libraries(find_port liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(get_distance ${LIBRARY_SAMPLE_DIR}/get_distance.c)
target_link_libraries(get_distance liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(get_distance_intensity ${LIBRARY_SAMPLE_DIR}/get_distance_intensity.c)
target_link_libraries(get_distance_intensity liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(get_multiecho ${LIBRARY_SAMPLE_DIR}/get_multiecho.c)
target_link_libraries(get_multiecho liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(get_multiecho_intensity ${LIBRARY_SAMPLE_DIR}/get_multiecho_intensity.c)
target_link_libraries(get_multiecho_intensity liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(reboot_test ${LIBRARY_SAMPLE_DIR}/reboot_test.c)
target_link_libraries(reboot_test liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(sensor_parameter ${LIBRARY_SAMPLE_DIR}/sensor_parameter.c)
target_link_libraries(sensor_parameter liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(sync_time_stamp ${LIBRARY_SAMPLE_DIR}/sync_time_stamp.c)
target_link_libraries(sync_time_stamp liburg_c open_urg_sensor ${catkin_LIBRARIES})

add_executable(timeout_test ${LIBRARY_SAMPLE_DIR}/timeout_test.c)
target_link_libraries(timeout_test liburg_c open_urg_sensor ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS liburg_c open_urg_sensor angle_convert_test calculate_xy find_port
                get_distance get_distance_intensity get_multiecho get_multiecho_intensity
	            reboot_test sensor_parameter sync_time_stamp timeout_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${LIBRARY_INCLUDE_DIR}/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
