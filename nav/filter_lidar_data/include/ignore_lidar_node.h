#include <string.h>
#include <optional>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/UInt8.h"

class IgnoreLidarNode
{
    public:
        IgnoreLidarNode(ros::NodeHandle nh);

    private:
        ros::NodeHandle nh_;
        ros::Publisher ignore_lidar_pub_;

        bool debug = true;

        int waypoint_target_ = 1;

        ros::Subscriber lidar_sub_;
        ros::Subscriber waypoint_sub_;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void waypointCallback(const std_msgs::UInt8ConstPtr &waypoint_msg);
};
