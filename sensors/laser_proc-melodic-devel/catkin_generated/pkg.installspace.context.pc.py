# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;sensor_msgs;rosconsole;nodelet".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-llaser_proc_library;-llaser_publisher;-llaser_transport;-llaser_proc_ROS;-lLaserProcNodelet".split(';') if "-llaser_proc_library;-llaser_publisher;-llaser_transport;-llaser_proc_ROS;-lLaserProcNodelet" != "" else []
PROJECT_NAME = "laser_proc"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.6"
