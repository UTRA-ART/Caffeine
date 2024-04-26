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
        ramps_to_cross = 1; // paramterize this dipsiht

        ramp_seg_sub = nh.subscribe("/ramp_seg", 10, &RampNavigateNode::rampFrontCallback, this);
        ramp_routine_pub = nh.advertise<std_msgs::Bool>("/ramp_routine", 1); // for dual_lidar to know to fuck off
        ramp_naving_pub = nh.advertise<std_msgs::Bool>("/ramp_naving", 1); // for navigate_waypoints to know to fuck off

        fuck = nh.advertise<nav_msgs::Path>("/fuck", 1);
        markfucker = nh.advertise<visualization_msgs::Marker>("/markfucker", 1);
        caffPoseFck = nh.advertise<geometry_msgs::Pose>("/fuckmypose", 1);
        laotzu = nh.advertise<std_msgs::String>("/laotzu", 1);
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
    float slope, xmid, ymid, px, py;
    Eigen::Matrix2d ramp2map;

    bool kys = false;
    ros::Publisher fuck;
    ros::Publisher markfucker;
    ros::Publisher caffPoseFck;
    ros::Publisher laotzu;

    void rampFrontCallback(const geometry_msgs::PoseArrayConstPtr& ramp_seg) {
        if (kys) {
            return;
        }
        if (ramps_to_cross <= 0) {
            return;
        }
        tf::StampedTransform transform;
        tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);
        const auto& caff = transform.getOrigin();

        // here's some dumb shit
        // tf::StampedTransform transform;
        // tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);
        // geometry_msgs::Pose fuckyou;
        // const auto& caff = transform.getOrigin();
        // fuckyou.position.x = caff.x();
        // fuckyou.position.y = caff.y();
        // fuckyou.position.z = caff.z();
        // caffPoseFck.publish(fuckyou);

        // rudimentary filter - by length of incline found
        // const auto& front = ramp_seg->poses.front().position;
        // const auto& back = ramp_seg->poses.back().position;
        // const float dx = front.x - back.x;
        // const float dy = front.y - back.y;
        // const float dz = front.z - back.z;
        // const float incline_len2 = dx*dx + dy*dy + dz*dz;
        // const float min_len = 2.2;
        // if (incline_len2 < min_len*min_len) {
        //     return;
        // }
        if (!pass_length(ramp_seg->poses)) {
            pre_ramp_detections = 0;
            return;
        }
        pre_ramp_detections += 1; // WORK HERE
        // change your dumb shit algorithm for finding the point in front of ramp
        const auto& front = ramp_seg->poses.front().position;
        const auto& back = ramp_seg->poses.back().position;
        const float x_len = back.x - front.x;
        const float y_len = back.y - front.y;
        const float len = sqrt(x_len*x_len + y_len*y_len);
        // const float slope_ = y_len / x_len;

        Eigen::Matrix2d ramp2map_;
        ramp2map_ << y_len, x_len,
                    -x_len, y_len;
        ramp2map_ = ramp2map_ / len;
        Eigen::Vector2d mid(-1, 0.5 * len);
        Eigen::Vector2d midmap = ramp2map * mid + Eigen::Vector2d(front.x, front.y);

        // const float xmid_ = x_len * 0.5 + front.x;
        // const float ymid_ = x_len * 0.5 * slope + front.y;

        // NOTE: change this to be some distance in front of ramp, not based on x distance
        // const float px_ = xmid - 0.5;
        // const float py_ = ymid + 0.5 / slope; // NOTE: watch for division by ~0, like when line is vertical on x-y plane
        const float mvavg = 0.4;
        const float mvavg_st = 1 - mvavg;

        // slope = slope * mvavg_st + mvavg * slope_;
        // slope = slope_;
        ramp2map = ramp2map * mvavg_st + mvavg * ramp2map_;
        xmid = xmid * mvavg_st + mvavg * midmap[0];
        ymid = ymid * mvavg_st + mvavg * midmap[1];
        // xmid = xmid * mvavg_st + mvavg * (x_len * 0.5 + front.x);
        // ymid = ymid * mvavg_st + mvavg * (x_len * 0.5 * slope + front.y);
        // NOTE: change this to be some distance in front of ramp, not based on x distance
        px = xmid;
        py = ymid;
        // px = px * mvavg_st + mvavg * (xmid + 0.5);
        // py = py * mvavg_st + mvavg * (ymid - 0.5) / slope;
        // px = (xmid - 0.5);
        // py = (xmid + 0.5) / slope;
        // px = px_;
        // py = py_;

        nav_msgs::Path paff;
        paff.header.stamp = ros::Time::now();
        paff.header.frame_id = "map";
        geometry_msgs::PoseStamped penis;
        // for (const auto& p : ramp_seg->poses) {
        //     penis.pose.position.x = p.position.x;
        //     penis.pose.position.y = p.position.y;
        //     penis.pose.position.z = p.position.z;
        //     paff.poses.push_back(penis);
        // }
        auto& anal = ramp_seg->poses.front().position;
        penis.pose.position.x = anal.x;
        penis.pose.position.y = anal.y;
        penis.pose.position.z = anal.z;
        paff.poses.push_back(penis);
        auto& painal = ramp_seg->poses.back().position;
        penis.pose.position.x = painal.x;
        penis.pose.position.y = painal.y;
        penis.pose.position.z = painal.z;
        paff.poses.push_back(penis);
        fuck.publish(paff);

        std_msgs::String fart;
        // fart.data = "beginngin section";
        // laotzu.publish(fart);
        if (state == no_ramp) { // OWRK FROM AROND HERE
            if (pre_ramp_detections < 10) {
                fart.data = "gape";
                laotzu.publish(fart);
                return;
            } else {
                state = to_ramp;
                fart.data = "nogape";
                laotzu.publish(fart);

                std_msgs::Bool naving_msg;
                naving_msg.data = true;
                ramp_naving_pub.publish(naving_msg);
            }
        }

        // for diagnostic, find topic in rviz to see it
        // visualization_msgs::Marker mom;
        // mom.header.stamp = ros::Time::now();
        // mom.header.frame_id = "map";
        // mom.action = 0;
        // mom.id = 69;
        // mom.type = visualization_msgs::Marker::CUBE;
        // mom.pose.position.x = px;
        // mom.pose.position.y = py;
        // mom.pose.position.z = 1;
        // mom.scale.x = 0.2;
        // mom.scale.y = 0.2;
        // mom.scale.z = 0.2;
        // mom.color.r = 1;
        // mom.color.g = 0;
        // mom.color.b = 0;
        // mom.color.a = 1;
        // mom.lifetime = ros::Duration(0);
        // mom.frame_locked = true;
        // markfucker.publish(mom);

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
        // ac.waitForResult(); // change this to be more up to date with incoming ramp data
        
        const float goalerror2 = (px - caff.x()) * (px - caff.x()) + (py - caff.y()) * (py - caff.y());
        if (goalerror2 < 0.5) {
            state = on_ramp;
            fart.data = "BULLSHIT";
            laotzu.publish(fart);

            cross(goal, xmid, ymid, ramp2map);
        }

        // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { // assume to be in front of ramp at this point, should navigate across
        //     fart.data = "ANAL";
        //     laotzu.publish(fart);
        // }
        
        // // poo and pee
        // if (ac.getState().isDone()) { 
        //     fart.data = "PAINAL";
        //     laotzu.publish(fart);
        // }
        // cross(goal, xmid, ymid, slope);
        // // }
        // naving_msg.data = false;
        // ramp_naving_pub.publish(naving_msg);
        // ramps_to_cross -= 1;
        // kys = true;
    }
    
    void cross(move_base_msgs::MoveBaseGoal goal, const float xmid, const float ymid, const Eigen::Matrix2d ramp2map) {
        // goal.target_pose.header.frame_id = "map";
        std_msgs::Bool pee;
        pee.data = true;
        ramp_routine_pub.publish(pee);

        goal.target_pose.pose.orientation.w = 1;
        float px = xmid;
        float py = ymid;
        const Eigen::Vector2d incr = ramp2map * Eigen::Vector2d(1,0);
        // NOTE: change this to be some distance in front of ramp, not based on x distance
        // const float xwise_incre = 0.5;
        for (int i = 0; i < 10; i += 1) {
        // for (int i = 0; i < 2; i += 1) {
            goal.target_pose.header.stamp = ros::Time::now();
            // px += xwise_incre;
            // py += - xwise_incre / slope; // NOTE: watch for division by ~0, like when line is vertical on x-y plane
            px += incr[0];
            py += incr[1];
            goal.target_pose.pose.position.x = px;
            goal.target_pose.pose.position.y = py;
            if (i == 6) { // NOTE: fuck me, i guess we disbale lidar til rover does crossing
                pee.data = false;
                ramp_routine_pub.publish(pee);
            }
            ac.sendGoal(goal);
            ac.waitForResult();
        }
    }

    bool pass_length(const std::vector<geometry_msgs::Pose>& seg) { // if ramp segment ends pass minimum length
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