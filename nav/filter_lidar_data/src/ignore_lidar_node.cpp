#include <boost/bind/placeholders.hpp>

#include "../include/ignore_lidar_node.h"

IgnoreLidarNode::IgnoreLidarNode(ros::NodeHandle nh)
    : nh_{nh}, time_sync_(time_sync_policy(10), lidar_sub_, odom_sub_)
{
    lidar_sub_.subscribe(nh_, "/scan", 10);
    odom_sub_.subscribe(nh_, "/odom", 10);

    time_sync_.registerCallback(boost::bind(&IgnoreLidarNode::lidarCallback, this, _1, _2));

    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);
    
    ROS_INFO("Connected to Node");
}    

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg, 
                                    const nav_msgs::OdometryConstPtr &odom_msg)
{
    if ((odom_msg->pose).pose.position.x >= -7.0  && 
        (odom_msg->pose).pose.position.x <= 5.74   && 
        (odom_msg->pose).pose.position.y >= -42.0 && 
        (odom_msg->pose).pose.position.y <= -36.85) {       
        
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

