#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
// for this piece of shit below, if Eigen stops compilation:
// do sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
// or something of that sort, suck my dick
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// #include <ros/console.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>

// const int LIDAR_BUF_LEN = 1080;

// method is based on computing expected difference in depth between two lidar detected
// given ramp angle and lidar height difference
float get_expected_ramp_depth(const float theta_degrees, const float lidar_distance) {
    return lidar_distance / std::tan(theta_degrees * M_PI / 180.0);
}

class DualLidarFilterNode {
public:
    DualLidarFilterNode() : ac("/move_base", true) {}
    void begin(ros::NodeHandle _nh) {
        nh = _nh;
        get_constants();
        init_ramp_depth();
        init_upper_lidar_search_constants();

        // ros::Duration(10.0).sleep(); // wait for the dumb ass tree to beocme connected, whore

        // bullshit whore fuck you
        fuck = nh.advertise<nav_msgs::Path>("/fuck", 1);
        fuckListener.setExtrapolationLimit(ros::Duration(0));
        fucker = nh.advertise<std_msgs::String>("/fucker", 1);
        markfucker = nh.advertise<visualization_msgs::Marker>("/markfucker", 1);

        lidar_sub = nh.subscribe(main_topic_name, 10, &DualLidarFilterNode::lidarCallback, this);
        upper_lidar_sub = nh.subscribe(upper_topic_name, 10, &DualLidarFilterNode::upperLidarCallback, this);
        out = nh.advertise<sensor_msgs::LaserScan>(out_topic_name, 1);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Subscriber upper_lidar_sub;
    ros::Publisher out;

    // cunt
    ros::Publisher fuck;
    tf::TransformListener fuckListener;
    laser_geometry::LaserProjection fuckProjector;
    ros::Publisher fucker;
    ros::Publisher markfucker;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

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

            // all my bullshit is here
            std::string frame = "map";
            // sensor_msgs::PointCloud2 cloudO, cloudI;
            sensor_msgs::PointCloud cloud;
            sensor_msgs::LaserScan refitted_msg = *lidar_msg;
            // clamp values, otherewise all laserscan points are not converted
            for (auto& v : refitted_msg.ranges) {
                if (v <= refitted_msg.range_min) {
                    v = refitted_msg.range_min + 0.01;
                } else if (v >= refitted_msg.range_max) {
                    v = refitted_msg.range_max - 0.01;
                }
                // v = std::clamp(v, 0.06, 4.095);
            }
            try {
                ros::Time now = ros::Time::now();
                fuckListener.waitForTransform("/base_laser", "/map", now, ros::Duration(2.0));
                fuckProjector.transformLaserScanToPointCloud(frame, refitted_msg, cloud, fuckListener);

                while(!ac.waitForServer(ros::Duration(0.0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
            } catch (tf2::LookupException e) { // these little shits can happen at the bginning
                return;
            } catch (tf2::ConnectivityException e) {
                return;
            } catch (tf2::ExtrapolationException e) {
                return;
            }
            // tf::StampedTransform transform;
            // fuckListener.lookupTransform("map", "base_laser", ros::Time(0), transform);
            // pcl_ros::transformPointCloud("map", transform, cloudI, cloudO);
            // sensor_msgs::convertPointCloud2ToPointCloud(cloudO, cloud);

            auto ramp_ends = get_longest_strip(lidar_msg->ranges.data(), out_msg.ranges.size());
            ramp_ends = std::make_pair(0, 300);
            auto indices = get_all_deeper(lidar_msg->ranges.data(), out_msg.ranges.size());

            std_msgs::String peepee;
            fucker.publish(peepee);
            peepee.data = std::to_string(cloud.points.size()) + " santa lauc " + std::to_string(lidar_msg->ranges.size());
            fucker.publish(peepee);
            peepee.data = std::to_string(lidar_msg->range_min) + " santa lauc " + std::to_string(lidar_msg->range_max);
            fucker.publish(peepee);

            nav_msgs::Path paff;
            paff.header.stamp = ros::Time::now();
            paff.header.frame_id = frame;
            geometry_msgs::PoseStamped penis;
            int largest_i = 0;
            if (indices.size() > 0) {
                for (int i = 0; i < indices.size(); i += 1) { 
                    if (indices[largest_i].size() < indices[i].size()) {
                        largest_i = i;
                    }
                }
                const auto& front = cloud.points[indices[largest_i].front()];
                const auto& back = cloud.points[indices[largest_i].back()];
                const float x_len = back.x - front.x;
                const float ylen = back.y - front.y;
                const float slope = ylen / x_len;
                const float xmid = x_len * 0.5 + front.x;
                const float ymid = x_len * 0.5 * slope + front.y;

                const float px = xmid - 1;
                const float py = ymid + 1 / slope; // NOTE: watch for division by ~0, like when line is vertical on x-y plane

                // navigate dumb fuck
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = frame;
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = px;
                goal.target_pose.pose.position.y = py;
                goal.target_pose.pose.orientation.w = 1;
                ac.sendGoal(goal);
                ac.waitForResult(); // suck my dick

                visualization_msgs::Marker mom;
                mom.header.stamp = ros::Time::now();
                mom.header.frame_id = frame;
                mom.action = 0;
                mom.id = 69;
                mom.type = visualization_msgs::Marker::CUBE;
                mom.pose.position.x = px;
                mom.pose.position.y = py;
                mom.pose.position.z = 1;
                mom.scale.x = 0.2;
                mom.scale.y = 0.2;
                mom.scale.z = 0.2;
                mom.color.r = 1;
                mom.color.g = 0;
                mom.color.b = 0;
                mom.color.a = 1;
                mom.lifetime = ros::Duration(0);
                mom.frame_locked = true;
                markfucker.publish(mom);

                for (int i : indices[largest_i]) {
                    auto& anal = cloud.points[i];
                    penis.pose.position.x = anal.x;
                    penis.pose.position.y = anal.y;
                    penis.pose.position.z = anal.z;
                    paff.poses.push_back(penis);
                }
            }

            // for (int i = ramp_ends.first; i < ramp_ends.second; i += 1) {
            //     auto& anal = cloud.points[i];
            //     penis.pose.position.x = anal.x;
            //     penis.pose.position.y = anal.y;
            //     penis.pose.position.z = anal.z;
            //     paff.poses.push_back(penis);
            // }
            // only two endpoints to describe ramp
            // auto& anal = cloud.points[ramp_ends.first];
            // penis.pose.position.x = anal.x;
            // penis.pose.position.y = anal.y;
            // penis.pose.position.z = anal.z;
            // paff.poses.push_back(penis);
            // auto& painal = cloud.points[ramp_ends.second-1];
            // penis.pose.position.x = painal.x;
            // penis.pose.position.y = painal.y;
            // penis.pose.position.z = painal.z;
            // paff.poses.push_back(penis);

            // penis.pose.position.x = 0;
            // penis.pose.position.y = 0;
            // penis.pose.position.z = 0;
            // paff.poses.push_back(penis);
            // penis.pose.position.x = 100000;
            // penis.pose.position.y = 100000;
            // penis.pose.position.z = 100000;
            // paff.poses.push_back(penis);
            
            fuck.publish(paff);

        } else { // default case, just use main lidar as it is
            out.publish(lidar_msg);
        }
    }
    // return index pair in scan corresponding to largest ramp-like entity
    // second index is exclusive
    std::pair<int, int> get_longest_strip(const float main[], int size) {
        // const float inf = std::numeric_limits<float>::infinity();
        // bool all_inf = true; // for carto fix below

        int l_a = -1, l_b = -1;
        int a = -1, b = -1;
        bool in_ramp = false; // if in process of seeing ramp
        // size = 300;
        for (int i = 0; i < size; i++) {
            const float comp_depth = last_upper_ranges[second_idx_fn(i, size)];
            float depth = comp_depth - main[i];
            if (depth > min_ramp_depth) {
                // out[i] = inf;
                if (!in_ramp) {
                    a = i;
                }
                in_ramp = true;
            } else {
                // out[i] = main[i];
                if (in_ramp) {
                    b = i;
                    if ((l_b - l_a) < (b - a)) { // replace with longest
                        l_a = a;
                        l_b = b;
                    }
                }
                in_ramp = false;
            }
        }
        return std::make_pair(l_a, l_b);
    }
    // all da damn indices w deeper thing
    std::vector<std::vector<int>> get_all_deeper(const float main[], int size) {
        std::vector<std::vector<int>> cockandballs;
        bool in_ramp = false; // if in process of seeing ramp
        // size = 300;
        for (int i = 0; i < size; i++) {
            const float comp_depth = last_upper_ranges[second_idx_fn(i, size)];
            float depth = comp_depth - main[i];
            if (depth > min_ramp_depth) {
                // out[i] = inf;
                if (!in_ramp) {
                    cockandballs.push_back(std::vector<int>());
                }
                in_ramp = true;
                cockandballs.back().push_back(i);
            } else {
                // out[i] = main[i];
                // if (in_ramp) { // exclusive, so even tho this ponit isn't in it follows exclusive end shit
                //     cockandballs.back().push_back(i);
                // }
                in_ramp = false;
            }
        }
        return cockandballs;
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