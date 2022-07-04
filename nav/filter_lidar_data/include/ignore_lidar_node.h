#include <array>
#include <string.h>
#include <optional>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Bool.h>

class IgnoreLidarNode
{
    public:
        IgnoreLidarNode(ros::NodeHandle nh);

    private:
        ros::NodeHandle nh_;
        ros::Publisher ignore_lidar_pub_;

        ros::Subscriber lidar_sub_;
        ros::Subscriber waypoint_sub_;
        bool passthrough = true;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void WaypointCallback(const std_msgs::Bool& msg);
};
