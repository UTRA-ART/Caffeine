# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "dynamic_reconfigure;laser_proc;message_runtime;nodelet;rosconsole;roscpp;sensor_msgs;std_msgs;std_srvs;urg_c".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lurg_c_wrapper;-lurg_node_driver".split(';') if "-lurg_c_wrapper;-lurg_node_driver" != "" else []
PROJECT_NAME = "urg_node"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.16"
