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
        
        GpsCoord second_waypoint_ = {43.65716861, -79.3903337}; //Temporarily set to in SIM Gps coordinates. CHANGE HERE LATER
        GpsCoord third_waypoint_ = {43.65717098, -79.39020002};

        std::array<GpsCoord, 2> bounds_;

        GpsCoord cur_gps_;
        bool is_gps_init = false;

        bool is_sim = true; //SWITCH AT COMP OR DELETE EXTRA CODE

        ros::Subscriber lidar_sub_;
        ros::Subscriber gps_sub_;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg);
        void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg);
};