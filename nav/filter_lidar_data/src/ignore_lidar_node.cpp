#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>
#include <limits>

#include "../include/ignore_lidar_node.h"

IgnoreLidarNode::IgnoreLidarNode(ros::NodeHandle nh)
    : nh_{nh}
{
    ROS_INFO("Connected to Node");

    // Subscribe to recieve lidar data and the target waypoint
    lidar_sub_ = nh_.subscribe("/scan", 10, &IgnoreLidarNode::lidarCallback, this);
    waypoint_sub_ = nh_.subscribe("/waypoint_target", 10, &IgnoreLidarNode::waypointCallback, this);

    // Publish to a new topic containing filtered lidar data
    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);

}    

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg)
{
    if (!is_odom_init) {
        return; // Do not do calculations without a pose to work with
    }

    if (waypoint_target_ == 3) {
        
        if (debug) {
            ROS_INFO("At Second Waypoint and aiming for the third");
        }
        
        sensor_msgs::LaserScan output_msg = *lidar_msg; // Make a copy of lidar_msg

        // Set each element in ranges to inf (No obstacles present) to clear costmap
        double inf = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 1080; i++) {
            output_msg.ranges[i] = inf;
        }
        
        ignore_lidar_pub_.publish(output_msg);

    } else {
        
        if (debug) {
            ROS_INFO("NOT between Second and Third waypoint");
        }
        
        ignore_lidar_pub_.publish(lidar_msg); // Republish the same msg. No changes to data.
    }
}

void IgnoreLidarNode::waypointCallback(const std_msgs::UInt8ConstPtr &waypoint_msg)
{
    waypoint_target_ = waypoint_msg->data;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ignore_lidar_node");
    ros::NodeHandle nh("~");
    IgnoreLidarNode ignore_lidar_node(nh);
    ros::spin();
    return 0;
}
