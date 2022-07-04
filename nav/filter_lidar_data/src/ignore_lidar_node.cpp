#include <limits>
#include "../include/ignore_lidar_node.h"

IgnoreLidarNode::IgnoreLidarNode(ros::NodeHandle nh)
    : nh_{nh}
{
    ROS_INFO("Connected to Node");
    /*
    // Set rectangular bounds where lidar data needs to be adjusted. Based on the 2nd and 3rd waypoint
    // TODO: Tune padding  
    bounds_[0].x = second_pose_stamped.pose.position.x + 3.0;
    bounds_[0].y = second_pose_stamped.pose.position.y + 3.0;
    bounds_[1].x = third_pose_stamped.pose.position.x - 3.0;
    bounds_[1].y = third_pose_stamped.pose.position.y - 3.0;
    */
    // Subscribe to recieve lidar data and current pose
    lidar_sub_ = nh_.subscribe("/scan", 10, &IgnoreLidarNode::lidarCallback, this);
    waypoint_sub_ = nh_.subscribe("/waypoint_int", 10, &IgnoreLidarNode::WaypointCallback, this);

    // Publish to a new topic containing filtered lidar data
    ignore_lidar_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_modified", 1);

}    

void IgnoreLidarNode::WaypointCallback(const std_msgs::Bool& msg){
    if (msg.data) {
        passthrough = true;
    } else {
        passthrough = false;
    }
        
}

void IgnoreLidarNode::lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg)
{

    if (!passthrough) {
        
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

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ignore_lidar_node");
    ros::NodeHandle nh("~");
    IgnoreLidarNode ignore_lidar_node(nh);
    ros::spin();
    return 0;
}
