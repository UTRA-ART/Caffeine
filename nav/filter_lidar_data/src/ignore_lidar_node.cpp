#include <boost/bind/placeholders.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

#include "../include/ignore_lidar_node.h"

IgnoreLidarNode::IgnoreLidarNode(ros::NodeHandle nh)
    : nh_{nh}
{
    // Transform Gps waypoints into pose
    
    geometry_msgs::PoseStamped second_pose_stamped = getPoseFromGps(second_waypoint_.latitude, 
                                                            second_waypoint_.longitude, "/odom");
    ROS_INFO("second_waypoint transformed: x: %f, y: %f", second_pose_stamped.pose.position.x,
                                                          second_pose_stamped.pose.position.y);
    geometry_msgs::PoseStamped third_pose_stamped = getPoseFromGps(third_waypoint_.latitude, 
                                                            third_waypoint_.longitude, "/odom");
    ROS_INFO("third_waypoint transformed: x: %f, y: %f", third_pose_stamped.pose.position.x,
                                                          third_pose_stamped.pose.position.y);
    
    bounds_[0].x = second_pose_stamped.pose.position.x + 1.0;
    bounds_[0].y = second_pose_stamped.pose.position.y + 2.0;
    bounds_[1].x = third_pose_stamped.pose.position.x;
    bounds_[1].y = third_pose_stamped.pose.position.y - 1.0;

    lidar_sub_ = nh_.subscribe("/scan", 10, &IgnoreLidarNode::lidarCallback, this);
    odom_sub_ = nh_.subscribe("/odometry/local", 10, &IgnoreLidarNode::odomCallback, this);

    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);
    
    ROS_INFO("Connected to Node");
}    

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg)
{
    if (!is_odom_init) {
        return;
    }

    if (cur_odom_.x <= bounds_[0].x && 
        cur_odom_.x >= bounds_[1].x && 
        cur_odom_.y <= bounds_[0].y && 
        cur_odom_.y >= bounds_[1].y) {

        ROS_INFO("At Second Waypoint");
        sensor_msgs::LaserScan output_msg;
        output_msg.header = lidar_msg->header;
        ignore_lidar_pub_.publish(output_msg);
    
    } else {
        ROS_INFO("NOT at Second Waypoint");
        ignore_lidar_pub_.publish(lidar_msg);
    }
}

void IgnoreLidarNode::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    if (is_odom_init == false) {
        is_odom_init = true;
    }
    cur_odom_.x = odom_msg->pose.pose.position.x;
    cur_odom_.y = odom_msg->pose.pose.position.y;
}

geometry_msgs::PoseStamped IgnoreLidarNode::getPoseFromGps(double latitude, double longitude, 
                                            std::string target_frame)
{
    geographic_msgs::GeoPoint geo_pt;
    geo_pt.latitude = latitude;
    geo_pt.longitude = longitude;
    geo_pt.altitude = 0;

    geodesy::UTMPoint utm_pt(geo_pt);
    tf::TransformListener listener;
    
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "utm";
    pose_stamped.pose.position.x = utm_pt.easting;
    pose_stamped.pose.position.y = utm_pt.northing;
    pose_stamped.pose.orientation.w = 1.0;

    ROS_INFO("Easting: %f, Northing: %f", utm_pt.easting, utm_pt.northing);

    geometry_msgs::PoseStamped pose_stamped_out;
    
    listener.waitForTransform(target_frame, "/utm", ros::Time(0), ros::Duration(4.0));
    listener.transformPose(target_frame, pose_stamped, pose_stamped);
    

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