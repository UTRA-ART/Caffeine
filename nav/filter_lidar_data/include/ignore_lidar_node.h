#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;

class IgnoreLidarNode
{
    public:
        IgnoreLidarNode(ros::NodeHandle nh);

    private:
        ros::NodeHandle nh_;
        ros::Publisher ignore_lidar_pub_;

        message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;

        typedef sync_policies::ApproximateTime<LaserScan, Odometry> time_sync_policy;

        Synchronizer<time_sync_policy> time_sync_; 

        //message_filters::TimeSynchronizer<sensor_msgs::LaserScan, nav_msgs::Odometry> timeSync_;

        void lidarCallback(const sensor_msgs::LaserScanConstPtr &lidar_msg, const nav_msgs::OdometryConstPtr &odom_msg);
};