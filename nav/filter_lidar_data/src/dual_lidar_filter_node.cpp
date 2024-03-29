#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>

// const int LIDAR_BUF_LEN = 1080;

// method is based on computing expected difference in depth between two lidar detected
// given ramp angle and lidar height difference
float get_expected_ramp_depth(const float theta_degrees, const float lidar_distance) {
    return lidar_distance / std::tan(theta_degrees * M_PI / 180.0);
}

class DualLidarFilterNode {
public:
    void begin(ros::NodeHandle _nh) {
        nh = _nh;
        get_constants();
        init_ramp_depth();
        init_upper_lidar_search_constants();
        
        lidar_sub = nh.subscribe(main_topic_name, 10, &DualLidarFilterNode::lidarCallback, this);
        upper_lidar_sub = nh.subscribe(upper_topic_name, 10, &DualLidarFilterNode::upperLidarCallback, this);
        out = nh.advertise<sensor_msgs::LaserScan>(out_topic_name, 1);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Subscriber upper_lidar_sub;
    ros::Publisher out;

    std::string main_topic_name;
    std::string upper_topic_name;
    std::string out_topic_name;
    float lidar_dist;
    float max_theta_degrees;
    int comp_lidar_tol_secs;
    
    // int upper_total_points;
    int upper_lidar_angular_range;
    int upper_lidar_start_index;
    int upper_lidar_stop_index;
    int main_lidar_angular_range;

    float min_ramp_depth;

    // sensor_msgs::LaserScan last_upper;
    std::vector<float> last_upper_ranges;
    int last_upper_stamp;

    int second_len;

    // bool is_upper_valid{false};
    std::function<int(int, int)> second_idx_fn;

    void lidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg) {
        const int delt_secs_ab = std::abs<int>(lidar_msg->header.stamp.sec - last_upper_stamp);
        const bool acceptable_record_time_diff = delt_secs_ab < comp_lidar_tol_secs;
        if (last_upper_ranges.size() >= second_len && acceptable_record_time_diff) {
            sensor_msgs::LaserScan out_msg = *lidar_msg;
            ramp_filter(out_msg.ranges, lidar_msg->ranges.data());
            out.publish(out_msg);
        } else { // default case, just use main lidar as it is
            out.publish(lidar_msg);
        }
    }
    void upperLidarCallback(const sensor_msgs::LaserScanConstPtr& lidar_msg) {
        last_upper_ranges = lidar_msg->ranges;
        last_upper_stamp = lidar_msg->header.stamp.sec;
    }
    // get 2nd depth for depth comparison
    int get_2nd_idx_from_first_idx(int prim_idx, int main_lidar_len) {
        const float ang_diff = ((float)prim_idx / (float)main_lidar_len - 0.5) * (float)main_lidar_angular_range;
        // const float second_i_delt = ang / (float)upper_lidar_angular_range * (upper_lidar_stop_index - upper_lidar_start_index);
        // const  = upper_lidar_stop_index - upper_lidar_start_index;
        const float second_i_diff = ang_diff / (float)upper_lidar_angular_range;
        const float second_i_delt = (second_i_diff + 0.5) * (float)second_len;
        const int second_i = round(second_i_delt + upper_lidar_start_index);
        return second_i;
    }

    // modify main array with original lidar to contain filtered values, based on comparing with upper sensor
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
        if (all_inf) { // if all inf, carto seems to not like it!
            std::fill(out.begin(), out.end(), NAN);
        }
    }
    void get_constants() {
        nh.getParam("main_topic", main_topic_name);
        nh.getParam("upper_topic", upper_topic_name);
        nh.getParam("out_topic", out_topic_name);
        nh.getParam("distance_to_second_lidar", lidar_dist);
        nh.getParam("max_theta_degrees", max_theta_degrees);
        nh.getParam("compare_lidar_time_tolerance_seconds", comp_lidar_tol_secs);

        // nh.getParam("upper_lidar_count", upper_total_points);
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