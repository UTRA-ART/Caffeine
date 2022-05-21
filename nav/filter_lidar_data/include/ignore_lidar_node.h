#include <array>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace message_filters;

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

        message_filters::Subscriber<LaserScan> lidar_sub_;
        message_filters::Subscriber<NavSatFix> odom_sub_;

        typedef sync_policies::ApproximateTime<LaserScan, NavSatFix> time_sync_policy;

        Synchronizer<time_sync_policy> time_sync_; 

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg, const sensor_msgs::NavSatFixConstPtr &gps_msg);
};