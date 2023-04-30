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
        
        // GpsCoord second_waypoint_;
        // GpsCoord third_waypoint_;

        // std::array<OdomCoord, 2> bounds_;

        // OdomCoord cur_odom_;
        // bool is_odom_init = false;

        ros::Subscriber lidar_sub_;
        ros::Subscriber odom_sub_;
        bool passthrough = true;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void WaypointCallback(const std_msgs::Bool& msg);
        // void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
        // geometry_msgs::PoseStamped getPoseFromGps(double latitude, double longitude, std::string target_frame);
};
