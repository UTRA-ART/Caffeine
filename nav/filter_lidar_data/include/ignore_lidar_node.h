#include <array>
#include <string.h>
#include <optional>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt8.h"

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
        
        GpsCoord second_waypoint_;
        GpsCoord third_waypoint_;

        int waypoint_target_ = 1;

        std::array<OdomCoord, 2> bounds_;

        OdomCoord cur_odom_;
        bool is_odom_init = false;

        ros::Subscriber lidar_sub_;
        ros::Subscriber waypoint_sub_;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void waypointCallback(const std_msgs::UInt8ConstPtr &waypoint_msg);
};
