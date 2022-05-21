#include <array>
#include <optional>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"

using namespace sensor_msgs;

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
        
        GpsCoord second_waypoint_ = {10, 20};
        GpsCoord third_waypoint_ = {10, 20};

        std::array<GpsCoord, 2> bounds_;

        GpsCoord cur_gps_;
        bool is_gps_init = false;

        ros::Subscriber lidar_sub_;
        ros::Subscriber gps_sub_;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg);
};