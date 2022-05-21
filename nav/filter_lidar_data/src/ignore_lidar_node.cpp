#include <boost/bind/placeholders.hpp>

#include "../include/ignore_lidar_node.h"

IgnoreLidarNode::IgnoreLidarNode(ros::NodeHandle nh)
    : nh_{nh}, time_sync_(time_sync_policy(10), lidar_sub_, gps_sub_)
{
    bounds_[0].latitude = second_waypoint_.latitude + 0.00002;
    bounds_[0].longitude = second_waypoint_.longitude - 0.000025;
    bounds_[1].latitude = third_waypoint_.latitude - 0.00002;
    bounds_[1].longitude = third_waypoint_.longitude + 0.00002;

    lidar_sub_.subscribe(nh_, "/scan", 10);
    gps_sub_.subscribe(nh_, "/gps/filtered", 10);

    time_sync_.registerCallback(boost::bind(&IgnoreLidarNode::lidarCallback, this, _1, _2));

    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);
    
    ROS_INFO("Connected to Node");
}    

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg, 
                                    const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
    if (gps_msg->latitude <= bounds_[0].latitude && 
        gps_msg->latitude >= bounds_[1].latitude && 
        gps_msg->longitude >= bounds_[0].longitude && 
        gps_msg->longitude <= bounds_[1].longitude) {       
        
        ROS_INFO("At Second Waypoint");
        sensor_msgs::LaserScan output_msg;
        output_msg.header = lidar_msg->header;
        ignore_lidar_pub_.publish(output_msg);
    } else {
        ROS_INFO("NOT at Second Waypoint");
        ignore_lidar_pub_.publish(lidar_msg);
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ignore_lidar_node");
    ros::NodeHandle nh("~");
    IgnoreLidarNode ignore_lidar_node(nh);
    ros::spin();
    return 0;
}