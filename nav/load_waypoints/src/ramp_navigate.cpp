#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>
#include <cmath>

// If Eigen stops compilation:
// Try: sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

enum State {
    no_ramp,    // normal
    to_ramp,    // navigating to front of ramp
    on_ramp,    // ramp traverse
};

class RampNavigateNode {
public:
    RampNavigateNode() : ac("/move_base", true) {}
    void begin(ros::NodeHandle _nh) {
        nh = _nh;

        state = no_ramp;
        ramps_to_cross = 1;

        ramp_seg_sub = nh.subscribe("/ramp_seg", 10, &RampNavigateNode::rampFrontCallback, this); // Stores location of ramp as line segments
        ramp_routine_pub = nh.advertise<std_msgs::Bool>("/ramp_routine", 1); // Tells that we are currently crossing ramp (not just approaching)
        ramp_naving_pub = nh.advertise<std_msgs::Bool>("/ramp_naving", 1); // For navigate_waypoints to know to temporarily stop and let ramp nav take over
    }

private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    ros::Subscriber ramp_seg_sub;
    ros::Publisher ramp_naving_pub;
    tf::TransformListener tfListener;
    ros::Publisher ramp_routine_pub;
    int ramps_to_cross;

    State state;
    int pre_ramp_detections = 0;
    int no_ramp_period = 0; // How many times pre_ramp_detections is NOT incremented
    float slope, xmid, ymid, px, py;
    Eigen::Matrix2d ramp2map;

    void rampFrontCallback(const geometry_msgs::PoseArrayConstPtr& ramp_seg) {
        if (state == on_ramp || ramps_to_cross <= 0) {
            return;
        }

        // If the length of ramp segment is not within expected range, do not proceed
        // Also make sure the ramp is detected for more than one time point (confirm it's actually there)
        if (!pass_length(ramp_seg->poses)) {
            if (pre_ramp_detections > 0) {
                no_ramp_period += 1;
                if (no_ramp_period > 3) {
                    pre_ramp_detections = 0;
                    no_ramp_period = 0;
                }
            }
            return;
        }
        pre_ramp_detections += 1; // WORK HERE

        // Obtain base_link -> map transform
        tf::StampedTransform transform;
        tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);
        const auto& caff = transform.getOrigin();

        // Find the middle point in front of ramp
        const auto& front = ramp_seg->poses.front().position;
        const auto& back = ramp_seg->poses.back().position;
        const float x_len = back.x - front.x;
        const float y_len = back.y - front.y;
        const float len = sqrt(x_len*x_len + y_len*y_len); // Length of detected ramp segment

        Eigen::Matrix2d ramp2map_;
        ramp2map_ << y_len, x_len,
                    -x_len, y_len;
        ramp2map_ = ramp2map_ / len;
        Eigen::Vector2d mid(-1, 0.5 * len);
        Eigen::Vector2d midmap = ramp2map * mid + Eigen::Vector2d(front.x, front.y);

        // Make the goal closer to ramp until within proximity
        // to mitigate  error caused by calculating the front of ramp to be too far away
        const float goal_dist2 = (midmap[0] - caff.x())*(midmap[0] - caff.x()) + (midmap[1] - caff.y())*(midmap[1] - caff.y());
        if (goal_dist2 > 1.5*1.5) {
            midmap = ramp2map * Eigen::Vector2d(0, 0.5 * len) + Eigen::Vector2d(front.x, front.y);
        }

        const float mvavg = 0.5;
        const float mvavg_st = 1 - mvavg;

        ramp2map = ramp2map * mvavg_st + mvavg * ramp2map_;
        xmid = xmid * mvavg_st + mvavg * midmap[0];
        ymid = ymid * mvavg_st + mvavg * midmap[1];

        // Goal x, y in map frame
        px = xmid;
        py = ymid;

        if (state == no_ramp) {
            if (pre_ramp_detections < 10) {
                return;
            } else {
                state = to_ramp;
                pre_ramp_detections = 0;
                std_msgs::Bool naving_msg;
                naving_msg.data = true;
                ramp_naving_pub.publish(naving_msg); // Send message that we are current in ramp navigation mode
            }
        }

        // Set a move_base goal for the middle front of ramp
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = px;
        goal.target_pose.pose.position.y = py;
        goal.target_pose.pose.orientation.w = 1;
        while (!ac.waitForServer(ros::Duration(0.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ac.sendGoal(goal);
        
        const float goalerror2 = (px - caff.x()) * (px - caff.x()) + (py - caff.y()) * (py - caff.y());
        if (goalerror2 < 0.5) {
            state = on_ramp;
            cross(goal, xmid, ymid, ramp2map); // Continue rest of navigation across ramp

            std_msgs::Bool naving_msg;
            naving_msg.data = false;
            ramp_naving_pub.publish(naving_msg);
            ramps_to_cross -= 1;
        }
    }
    
    void cross(move_base_msgs::MoveBaseGoal goal, const float xmid, const float ymid, const Eigen::Matrix2d ramp2map) {
        std_msgs::Bool is_on_ramp;
        is_on_ramp.data = true;
        ramp_routine_pub.publish(is_on_ramp); // Send message that we are currently crossing ramp

        const float ramp_traverse_dist = 6; // Total distance to traverse 
        const int traverse_count = 6; // How many goal points to set along the ramp
        const Eigen::Vector2d incr = ramp2map * Eigen::Vector2d(ramp_traverse_dist / traverse_count, 0); // Make it a tiny bit past the ramp

        float px = xmid;
        float py = ymid;

        // Set goals at repeated small increments across ramp
        for (int i = 0; i < traverse_count; i += 1) {
            goal.target_pose.header.stamp = ros::Time::now();
            px += incr[0];
            py += incr[1];
            goal.target_pose.pose.position.x = px;
            goal.target_pose.pose.position.y = py;

            ac.sendGoal(goal);
            ac.waitForResult();
        }
        state = no_ramp; // After we finish crossing ramp
    }

    // Check for length of segments (between min and max lengthpass)
    bool pass_length(const std::vector<geometry_msgs::Pose>& seg) { // If ramp segment ends pass minimum length
        const auto& front = seg.front().position;
        const auto& back = seg.back().position;
        const float dx = front.x - back.x;
        const float dy = front.y - back.y;
        const float dz = front.z - back.z;
        const float incline_len2 = dx*dx + dy*dy + dz*dz;
        const float min_len = 2.5;
        const float max_len = 4;
        return incline_len2 >= min_len*min_len && incline_len2 <= max_len*max_len;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ramp_navigate");
    ros::NodeHandle nh("~");

    RampNavigateNode node;
    node.begin(nh);

    ros::spin();
}