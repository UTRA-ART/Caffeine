#include <boost/bind/placeholders.hpp>

#include "../include/ignore_lidar_node.h"

IgnoreLidarNode::IgnoreLidarNode(ros::NodeHandle nh)
    : nh_{nh}
{
    bounds_[0].latitude = second_waypoint_.latitude + 0.00002;
    bounds_[0].longitude = second_waypoint_.longitude - 0.000025;
    bounds_[1].latitude = third_waypoint_.latitude - 0.00002;
    bounds_[1].longitude = third_waypoint_.longitude + 0.00002;

    lidar_sub_ = nh_.subscribe("/scan", 10, &IgnoreLidarNode::lidarCallback, this);
    gps_sub_ = nh_.subscribe("/gps/filtered", 10, &IgnoreLidarNode::gpsCallback, this);

    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);
    
    ROS_INFO("Connected to Node");
}    

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg)
{
    if (!is_gps_init) {
        return;
    }

    if (cur_gps_.latitude <= bounds_[0].latitude && 
        cur_gps_.latitude >= bounds_[1].latitude && 
        cur_gps_.longitude >= bounds_[0].longitude && 
        cur_gps_.longitude <= bounds_[1].longitude) {       
        
        ROS_INFO("At Second Waypoint");
        sensor_msgs::LaserScan output_msg;
        output_msg.header = lidar_msg->header;
        ignore_lidar_pub_.publish(output_msg);
    } else {
        ROS_INFO("NOT at Second Waypoint");
        ignore_lidar_pub_.publish(lidar_msg);
    }
}

void IgnoreLidarNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg)
{
    if (is_gps_init == false) {
        is_gps_init = true;
    }
    cur_gps_.latitude = gps_msg->latitude;
    cur_gps_.longitude = gps_msg->longitude;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ignore_lidar_node");
    ros::NodeHandle nh("~");
    IgnoreLidarNode ignore_lidar_node(nh);
    ros::spin();
    return 0;
}