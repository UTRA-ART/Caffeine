cmake_minimum_required(VERSION 3.0.2)
project(cv)

# # Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  rospy 
  std_msgs 
  message_generation
  geometry_msgs
  )

catkin_python_setup()

add_message_files(
    FILES 
    FloatArray.msg
    FloatList.msg
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
  message_runtime
  )