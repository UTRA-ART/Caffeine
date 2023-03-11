#include "ros/ros.h"

int main(int argc, char **argv) {
    // distance_to_second_lidar
    ros::init(argc, argv, "dual_lidar_filter_node");
    ros::NodeHandle nh("~");

    while (true) {
        float laser_dist;
        nh.getParam("distance_to_second_lidar", laser_dist);
        ROS_INFO("FUCK I HATE THIS: %f", laser_dist);
        ros::Duration(1.0).sleep();
    }
}