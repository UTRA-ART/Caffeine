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

    // Obtain waypoint gps coordingates from launch file
    nh_.getParam("second_waypoint_latitude", second_waypoint_.latitude);
    nh_.getParam("second_waypoint_longitude", second_waypoint_.longitude);
    ROS_INFO("Second Waypoint Coordinates: %f, %f", second_waypoint_.latitude, second_waypoint_.longitude);
    nh_.getParam("third_waypoint_latitude", third_waypoint_.latitude);
    nh_.getParam("third_waypoint_longitude", third_waypoint_.longitude);
    ROS_INFO("Third Waypoint Coordinates: %f, %f", third_waypoint_.latitude, third_waypoint_.longitude);

    // Transform Gps waypoints into pose
    geometry_msgs::PoseStamped second_pose_stamped = getPoseFromGps(second_waypoint_.latitude, 
                                                            second_waypoint_.longitude, "/odom");
    ROS_INFO("second_waypoint transformed: x: %f, y: %f", second_pose_stamped.pose.position.x,
                                                          second_pose_stamped.pose.position.y);
    geometry_msgs::PoseStamped third_pose_stamped = getPoseFromGps(third_waypoint_.latitude, 
                                                            third_waypoint_.longitude, "/odom");
    ROS_INFO("third_waypoint transformed: x: %f, y: %f", third_pose_stamped.pose.position.x,
                                                          third_pose_stamped.pose.position.y);
    
    // Set rectangular bounds where lidar data needs to be adjusted. Based on the 2nd and 3rd waypoint
    // TODO: Tune padding  
    bounds_[0].x = second_pose_stamped.pose.position.x + 3.0;
    bounds_[0].y = second_pose_stamped.pose.position.y + 3.0;
    bounds_[1].x = third_pose_stamped.pose.position.x - 3.0;
    bounds_[1].y = third_pose_stamped.pose.position.y - 3.0;

    // Subscribe to recieve lidar data and current pose
    lidar_sub_ = nh_.subscribe("/scan", 10, &IgnoreLidarNode::lidarCallback, this);
    odom_sub_ = nh_.subscribe("/odometry/global", 10, &IgnoreLidarNode::odomCallback, this);

    // Publish to a new topic containing filtered lidar data
    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);

}    

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg)
{
    if (!is_odom_init) {
        return; // Do not do calculations without a pose to work with
    }

    if (cur_odom_.x <= bounds_[0].x && 
        cur_odom_.x >= bounds_[1].x && 
        cur_odom_.y <= bounds_[0].y && 
        cur_odom_.y >= bounds_[1].y) {
        
        if (debug) {
            ROS_INFO("At Second Waypoint. Currently at: x:%f, y:%f", cur_odom_.x, cur_odom_.y);
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
            ROS_INFO("NOT at Second Waypoint. Currently at: x:%f, y:%f", cur_odom_.x, cur_odom_.y);
        }
        
        ignore_lidar_pub_.publish(lidar_msg); // Republish the same msg. No changes to data.
    }
}

void IgnoreLidarNode::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    if (is_odom_init == false) {
        is_odom_init = true; // Indicates that the current pose is initialized
    }
    cur_odom_.x = odom_msg->pose.pose.position.x;
    cur_odom_.y = odom_msg->pose.pose.position.y;
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
