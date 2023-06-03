#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "lane_detection/FloatArray.h"

#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include <limits>
#include <cmath>
#include <stdbool.h>

class LaneScan {
public:
    void begin(ros::NodeHandle _nh) {
        nh = _nh;
        
        lane_sub = nh.subscribe(lane_float_topic, 10, &LaneScan::laneCallback, this);
        out = nh.advertise<sensor_msgs::LaserScan>(lane_laser_topic, 1);

        // Lidar callback
        lidar_sub = nh.subscribe(lidar_laser_topic, 10, &LaneScan::lidarCallback, this);
        lidar_out = nh.advertise<sensor_msgs::LaserScan>(lidar_laser_topic_out, 1);



        listener.waitForTransform("/base_laser", "/left_camera_link_optical", ros::Time(), ros::Duration(4.0)); //Wait for the transform to be uploaded

        last_scan_msg_time = ros::Time::now();
        last_scan_found = false;
        done_lane_lidar_copy = false;

        std::cout << "Starting CPP Lidar" << std::endl;
    }


private:
    ros::NodeHandle nh;
    ros::Subscriber lane_sub;
    ros::Publisher out;

    ros::Subscriber lidar_sub;
    ros::Publisher lidar_out;



    std::string lane_float_topic = "/cv/lane_detections";
    std::string lane_laser_topic = "/cv/lane_detections_scan2";
    std::string lidar_laser_topic = "/scan_modified";
    std::string lidar_laser_topic_out = "/scan_merged";

    


    tf::TransformListener listener;

    sensor_msgs::LaserScan last_scan_msg; // Last lane lidar message
    ros::Time last_scan_msg_time; // Last time lane lidar published
    bool last_scan_found; // Has lane lidar published once?
    bool done_lane_lidar_copy;



    void laneCallback(const lane_detection::FloatArray& float_array) {
        
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = "base_laser";
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.angle_min = -2.3561899662017822;
        scan_msg.angle_max = 2.3561899662017822;
        scan_msg.angle_increment = 0.0043673585169017315;
        scan_msg.range_min = 0.05999999865889549;
        scan_msg.range_max = 4.09499979019165;

        // Generate the angles with infinity
        std::vector<double> angles;
        std::vector<double> ranges;
        for (double i = scan_msg.angle_min; i <= scan_msg.angle_max; i+= scan_msg.angle_increment) {
            angles.push_back(i);
            scan_msg.ranges.push_back(std::numeric_limits<double>::infinity());
        }
        // scan_msg.ranges = std::copy(ranges);

        bool point_added = false;
        if (float_array.lists.size() != 0) {
            // std::cout << "Starting FOR LOOP" << std::endl;

            int pts_size = float_array.lists[0].elements.size();
            for (int i = 0; i < pts_size; i++) {
                auto point = float_array.lists[0].elements[i];

                geometry_msgs::PoseStamped old_pose;
                old_pose.header.frame_id = float_array.header.frame_id;
                old_pose.pose.position.x = point.x;
                old_pose.pose.position.y = point.y;
                old_pose.pose.position.z = point.z;
                old_pose.pose.orientation.w = 1.0;

                geometry_msgs::PoseStamped new_pose;

                listener.transformPose("/base_laser", old_pose, new_pose); //Transform utm coords to "odom" frame

                double x = new_pose.pose.position.x;
                double y = new_pose.pose.position.y;

                double theta = atan2(y, x);
                double distance = sqrt((x*x) + (y*y));

                // std::cout << "New Pose: " << theta << " " << distance << std::endl;

                // Find which index it goes to
                int idx = 0;
                for (double angle = scan_msg.angle_min; angle <= theta && angle <= scan_msg.angle_max; angle += scan_msg.angle_increment) {
                    idx++;
                }

                if (idx == scan_msg.ranges.size())
                    idx--;

                // std::cout << "Adding Point at " << idx << "with "  << scan_msg.ranges.size() << std::endl;
                if (scan_msg.ranges[idx] > 100.0 ) { // If at infinity
                    // std::cout << "new point" << std::endl;
                    scan_msg.ranges[idx] = distance;
                    point_added = true;
                    // std::cout << "POINT ADDED" << std::endl;
                } else {
                    // std::cout << "ELSE new point" << std::endl;
                    scan_msg.ranges[idx] = (distance < scan_msg.ranges[idx]) ? distance : scan_msg.ranges[idx];
                    point_added = true;

                }
            }
        }


        if (point_added) {
            out.publish(scan_msg);
            last_scan_msg = scan_msg;
            last_scan_msg_time = ros::Time::now();

            last_scan_found = true;
        } else {
            return;
        }

        // if (point_added) {
        //     out.publish(scan_msg);
        //     last_scan_msg = scan_msg;
        //     last_scan_msg_time = ros::Time::now();

        // } else if ((ros::Time::now() - last_scan_msg_time) < ros::Duration(1)) {
        //     out.publish(scan_msg);
        // } else {
        //     return;
        // }

    }


    void lidarCallback(const sensor_msgs::LaserScan& lidar_scan) {

        // if (done_lane_lidar_copy) {
        //     return;
        // }

        bool point_found = false;

        for (int i = 0; i < lidar_scan.ranges.size(); i++) {

            if (lidar_scan.ranges[i] < 1000) {
                point_found = true;
                break;
            }
        }

        if (!point_found && last_scan_found) {
            last_scan_msg.header.stamp = ros::Time::now();
            lidar_out.publish(last_scan_msg);
            // done_lane_lidar_copy = true;
        } else {
            lidar_out.publish(lidar_scan);
        }
        

    }


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane_scan");
    ros::NodeHandle nh("~");

    LaneScan node;
    node.begin(nh);

    ros::spin();
}