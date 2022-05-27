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

        struct GpsCoord {

            double latitude{};
            double longitude{};
        };

        struct odomCoord {

            double x;
            double y;
        };
        
        GpsCoord second_waypoint_ = {43.6571667, -79.3903788}; //Temporarily set to in SIM Gps coordinates. CHANGE HERE LATER
        GpsCoord third_waypoint_ = {43.6571678, -79.3902588};

        std::array<odomCoord, 2> bounds_;

        odomCoord cur_odom_;
        bool is_odom_init = false;

        ros::Subscriber lidar_sub_;
        ros::Subscriber odom_sub_;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
        geometry_msgs::PoseStamped getPoseFromGps(double latitude, double longitude, std::string target_frame);
};
