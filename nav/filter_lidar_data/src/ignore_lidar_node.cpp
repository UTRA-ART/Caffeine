#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>
#include <limits>
#include <std_msgs/Bool.h>

#include "../include/ignore_lidar_node.h"

IgnoreLidarNode::IgnoreLidarNode(ros::NodeHandle nh)
    : nh_{nh}
{
    ROS_INFO("Connected to Ignore Lidar Node");

    // Subscribe to recieve lidar data and current pose
    lidar_sub_ = nh_.subscribe("/scan", 10, &IgnoreLidarNode::lidarCallback, this);
    waypoint_sub_ = nh_.subscribe("/waypoint_int", 10, &IgnoreLidarNode::WaypointCallback, this);

    // Publish to a new topic containing filtered lidar data
    ignore_lidar_ = false;
    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);

}    

void IgnoreLidarNode::WaypointCallback(const std_msgs::Bool& msg){
    ignore_lidar_ = msg.data;
}

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg)
{
    if (ignore_lidar_) {
        
        sensor_msgs::LaserScan output_msg = *lidar_msg; // Make a copy of lidar_msg

        // Set each element in ranges to inf (No obstacles present) to clear costmap
        double inf = std::numeric_limits<double>::infinity();
        for (int i = 0; i < sizeof(output_msg.ranges)/sizeof(output_msg.ranges[0]); i++) {
            output_msg.ranges[i] = inf;
        }
        
        ignore_lidar_pub_.publish(output_msg);

    } else {
        ignore_lidar_pub_.publish(lidar_msg); // Republish the same msg. No changes to data.
    }

}

geometry_msgs::PoseStamped IgnoreLidarNode::getPoseFromGps(double latitude, double longitude, 
                                            std::string target_frame)
{
    // Prepare lat/lon for transformation by using a GeoPoint msg
    geographic_msgs::GeoPoint geo_pt; 
    geo_pt.latitude = latitude;
    geo_pt.longitude = longitude;
    geo_pt.altitude = 0;

    geodesy::UTMPoint utm_pt(geo_pt); // Transforms gps lat/lon to utm coordinates

    ROS_INFO("Easting: %f, Northing: %f", utm_pt.easting, utm_pt.northing);
    
    tf::TransformListener listener;
    
    //Prepare utm coordinates for transformation using PoseStamped msg
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "utm";
    pose_stamped.pose.position.x = utm_pt.easting;
    pose_stamped.pose.position.y = utm_pt.northing;
    pose_stamped.pose.orientation.w = 1.0;
    
    listener.waitForTransform(target_frame, "/utm", ros::Time(0), ros::Duration(4.0)); // Wait for the transform to be uploaded
    listener.transformPose(target_frame, pose_stamped, pose_stamped); //Transform utm coords to "odom" frame

    return pose_stamped;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ignore_lidar_node");
    ros::NodeHandle nh("~");
    IgnoreLidarNode ignore_lidar_node(nh);
    ros::spin();
    return 0;
}
