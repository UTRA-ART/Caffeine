cmake_minimum_required(VERSION 3.0.2)
project(description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
    urdf
    xacro
    rtabmap_ros
    stereo_image_proc
)

find_package(gazebo REQUIRED)


catkin_package(
    CATKIN_DEPENDS urdf xacro gazebo_ros
    )


include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch urdf/meshes rviz urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

