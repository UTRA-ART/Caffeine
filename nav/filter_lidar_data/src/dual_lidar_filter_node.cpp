#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>

// Method is based on computing expected difference in depth between two lidar detected
// given ramp angle and lidar height difference

float get_expected_ramp_depth(const float theta_degrees, const float lidar_distance) {
    return lidar_distance / std::tan(theta_degrees * M_PI / 180.0);
}

class DualLidarFilterNode {
public:
    DualLidarFilterNode() {}
    void begin(ros::NodeHandle _nh) {
        nh = _nh;
        get_constants();
        init_ramp_depth();
        init_upper_lidar_search_constants();

        // Initiate Subscribers/Publishers
        ramp_seg_pub = nh.advertise<geometry_msgs::PoseArray>("/ramp_seg", 1); // Location of the ramp

        lidar_sub = nh.subscribe(main_lidar_topic_name, 10, &DualLidarFilterNode::lidarCallback, this);
        upper_lidar_sub = nh.subscribe(upper_lidar_topic_name, 10, &DualLidarFilterNode::upperLidarCallback, this);
        out = nh.advertise<sensor_msgs::LaserScan>(out_topic_name, 1);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Subscriber upper_lidar_sub;
    ros::Publisher out;

    ros::Publisher ramp_seg_pub;
    tf::TransformListener tfListener;
    laser_geometry::LaserProjection projector;

    std::string main_lidar_topic_name;
    std::string upper_lidar_topic_name;
    std::string out_topic_name;
    float lidar_dist;
    float max_theta_degrees;
    int comp_lidar_tol_secs;
    
    int upper_lidar_angular_range;
    int upper_lidar_start_index;
    int upper_lidar_stop_index;
    int main_lidar_angular_range;

    float min_ramp_depth;

    std::vector<float> last_upper_ranges;
    int last_upper_stamp;

    int second_len;
    int init_lidar_fill = 0;

    std::function<int(int, int)> second_idx_fn;

    void lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg) {
        const int delt_secs_ab = std::abs<int>(lidar_msg->header.stamp.sec - last_upper_stamp);
        const bool acceptable_record_time_diff = delt_secs_ab < comp_lidar_tol_secs;
        if (last_upper_ranges.size() >= second_len && acceptable_record_time_diff) {
            sensor_msgs::LaserScan out_msg = *lidar_msg;
            ramp_filter(out_msg.ranges, lidar_msg->ranges.data());
            out.publish(out_msg);

            std::string frame = "map";
            sensor_msgs::PointCloud cloud;
            sensor_msgs::LaserScan refitted_msg = *lidar_msg;

            // Clamp values, otherewise all laserscan points are not converted
            // If the lidar data point is beyond the min or max range, bring the value back into range
            for (auto& v : refitted_msg.ranges) {
                if (v <= refitted_msg.range_min) {
                    v = refitted_msg.range_min + 0.01;
                } else if (v >= refitted_msg.range_max) {
                    v = refitted_msg.range_max - 0.01;
                }
            }

            // Transform Lidar msg to a Point Cloud in /map frame
            try {
                ros::Time now = ros::Time::now();
                tfListener.waitForTransform("/base_laser", "/map", now, ros::Duration(2.0));
                projector.transformLaserScanToPointCloud(frame, refitted_msg, cloud, tfListener);
            } catch (tf2::LookupException e) { // These exceptions may happen
                return;
            } catch (tf2::ConnectivityException e) {
                return;
            } catch (tf2::ExtrapolationException e) {
                return;
            }

            auto indices = get_all_deeper(lidar_msg->ranges.data(), out_msg.ranges.size());

            int largest_i = 0;
            if (indices.size() > 0) {
                for (int i = 0; i < indices.size(); i += 1) { 
                    if (indices[largest_i].size() < indices[i].size()) {
                        largest_i = i;
                    }
                }
                // Send to ramp_navigate node via /ramp_seg topic
                geometry_msgs::PoseArray ramp_msg;
                ramp_msg.header.stamp = ros::Time::now();
                ramp_msg.header.frame_id = "map";
                ramp_msg.poses.reserve(indices.size());
                for (const int i : indices[largest_i]) {
                    geometry_msgs::Pose pose;
                    pose.position.x = cloud.points[i].x;
                    pose.position.y = cloud.points[i].y;
                    pose.position.z = cloud.points[i].z;
                    ramp_msg.poses.push_back(pose);
                }
                ramp_seg_pub.publish(ramp_msg);
            }

        } else { // Default case, just use main lidar as it is
            out.publish(lidar_msg);
        }
    }

    // Create 2D vector containing all individual segments of a ramp (imagining ramp detection as a line)
    std::vector<std::vector<int>> get_all_deeper(const float main[], int size) {
        std::vector<std::vector<int>> segments;
        bool in_ramp = false; // If in process of seeing ramp
        
        for (int i = 0; i < size; i++) {
            const float comp_depth = last_upper_ranges[second_idx_fn(i, size)];
            float depth = comp_depth - main[i];
            if (depth > min_ramp_depth) {
                // Add/update new ramp segments
                if (!in_ramp) {
                    segments.push_back(std::vector<int>());
                }
                in_ramp = true;
                segments.back().push_back(i);
            } else {
                in_ramp = false;
            }
        }
        return segments;
    }

    void upperLidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg) {
        last_upper_ranges = lidar_msg->ranges;
        last_upper_stamp = lidar_msg->header.stamp.sec;
    }

    // Get 2nd depth for depth comparison
    int get_2nd_idx_from_first_idx(int prim_idx, int main_lidar_len) {
        const float ang_diff = ((float)prim_idx / (float)main_lidar_len - 0.5) * (float)main_lidar_angular_range;
        // const float second_i_delt = ang / (float)upper_lidar_angular_range * (upper_lidar_stop_index - upper_lidar_start_index);
        // const  = upper_lidar_stop_index - upper_lidar_start_index;
        const float second_i_diff = ang_diff / (float)upper_lidar_angular_range;
        const float second_i_delt = (second_i_diff + 0.5) * (float)second_len;
        const int second_i = round(second_i_delt + upper_lidar_start_index);
        return second_i;
    }

    // Modify main array with original lidar to contain filtered values, based on comparing with upper sensor
    void ramp_filter(std::vector<float>& out, const float main[]) {
        const float inf = std::numeric_limits<float>::infinity();
        bool all_inf = true; // for carto fix below
        for (int i = 0; i < out.size(); i++) {
            const float comp_depth = last_upper_ranges[second_idx_fn(i, out.size())];
            float depth = comp_depth - main[i];
            if (depth > min_ramp_depth) {
                out[i] = inf;
            } else {
                out[i] = main[i];
            }
            if (!std::isinf(out[i])) {
                all_inf = false;
            }
        }

        // If lidar senses all inf, set one point to 3.0 for ~50 msgs to make cartographer initialize
        if (all_inf) { // if all inf, carto seems to not like it!
            std::fill(out.begin(), out.end(),NAN);
            if (init_lidar_fill < 50) {
                out[0] = 3.0;
                init_lidar_fill += 1;
            }
        }
    }

    void get_constants() {
        nh.getParam("main_lidar_topic", main_lidar_topic_name);
        nh.getParam("upper_lidar_topic", upper_lidar_topic_name);
        nh.getParam("out_topic", out_topic_name);
        nh.getParam("distance_to_second_lidar", lidar_dist);
        nh.getParam("max_theta_degrees", max_theta_degrees);
        nh.getParam("compare_lidar_time_tolerance_seconds", comp_lidar_tol_secs);

        nh.getParam("upper_lidar_angular_total_range", upper_lidar_angular_range);
        nh.getParam("upper_lidar_start_index", upper_lidar_start_index);      
        nh.getParam("upper_lidar_stop_index", upper_lidar_stop_index);
        nh.getParam("main_lidar_angular_total_range", main_lidar_angular_range);
    }

    void init_ramp_depth() {
        min_ramp_depth = get_expected_ramp_depth(max_theta_degrees, lidar_dist);
    }
    
    void init_upper_lidar_search_constants() {
        second_idx_fn = std::bind(&DualLidarFilterNode::get_2nd_idx_from_first_idx, this, std::placeholders::_1, std::placeholders::_2);
        second_len = upper_lidar_stop_index - upper_lidar_start_index;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "dual_lidar_filter_node");
    ros::NodeHandle nh("~");

    DualLidarFilterNode node;
    node.begin(nh);

    ros::spin();
}