#include <array>
#include <string.h>
#include <optional>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class IgnoreLidarNode
{
    public:
        IgnoreLidarNode(ros::NodeHandle nh);

    private:
        ros::NodeHandle nh_;
        ros::Publisher ignore_lidar_pub_;

        bool debug = false;

        struct GpsCoord {
            double latitude{};
            double longitude{};
        };

        struct OdomCoord {
            double x;
            double y;
        };

        ros::Subscriber lidar_sub_;
        ros::Subscriber odom_sub_;
        bool ignore_lidar_ = true;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void WaypointCallback(const std_msgs::Bool& msg);
};
