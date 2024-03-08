#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

class RampNavigateNode {
public:
    RampNavigateNode() : ac("/move_base", true) {}
    void begin(ros::NodeHandle _nh) {
        nh = _nh;

        ramp_seg_sub = nh.subscribe("/ramp_seg", 10, &RampNavigateNode::rampFrontCallback, this);
        ramp_routine_pub = nh.advertise<std_msgs::Bool>("/ramp_routine", 1);

        fuck = nh.advertise<nav_msgs::Path>("/fuck", 1);
        markfucker = nh.advertise<visualization_msgs::Marker>("/markfucker", 1);
        caffPoseFck = nh.advertise<geometry_msgs::Pose>("/fuckmypose", 1);
    }

private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    ros::Subscriber ramp_seg_sub;
    tf::TransformListener tfListener;
    ros::Publisher ramp_routine_pub;

    ros::Publisher fuck;
    ros::Publisher markfucker;
    ros::Publisher caffPoseFck;

    void rampFrontCallback(const geometry_msgs::PoseArrayConstPtr& ramp_seg) {
        // here's some dumb shit
        tf::StampedTransform transform;
        tfListener.lookupTransform("map", "base_link", ros::Time(0), transform);
        geometry_msgs::Pose fuckyou;
        const auto& fuckpoint = transform.getOrigin();
        fuckyou.position.x = fuckpoint.x();
        fuckyou.position.y = fuckpoint.y();
        fuckyou.position.z = fuckpoint.z();
        caffPoseFck.publish(fuckyou);        

        const auto& front = ramp_seg->poses.front().position;
        const auto& back = ramp_seg->poses.back().position;
        const float x_len = back.x - front.x;
        const float ylen = back.y - front.y;
        const float slope = ylen / x_len;
        const float xmid = x_len * 0.5 + front.x;
        const float ymid = x_len * 0.5 * slope + front.y;

        // NOTE: change this to be some distance in front of ramp, not based on x distance
        const float px = xmid - 1;
        const float py = ymid + 1 / slope; // NOTE: watch for division by ~0, like when line is vertical on x-y plane

        // for diagnostic, find topic in rviz to see it
        visualization_msgs::Marker mom;
        mom.header.stamp = ros::Time::now();
        mom.header.frame_id = "map";
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

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = px;
        goal.target_pose.pose.position.y = py;
        goal.target_pose.pose.orientation.w = 1;
        while(!ac.waitForServer(ros::Duration(0.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ac.sendGoal(goal);
        ac.waitForResult(); // change this to be more up to date with incoming ramp data
        // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) { // assume to be in front of ramp at this point, should navigate across
        // if (ac.getState().isDone()) {
        cross(goal, xmid, ymid, slope);
        // }
    }
    
    void cross(move_base_msgs::MoveBaseGoal goal, const float xmid, const float ymid, const float slope) {
        // goal.target_pose.header.frame_id = "map";
        std_msgs::Bool pee;
        pee.data = true;
        ramp_routine_pub.publish(pee);

        goal.target_pose.pose.orientation.w = 1;
        float px = xmid;
        float py = ymid;
        // NOTE: change this to be some distance in front of ramp, not based on x distance
        const float xwise_incre = 0.5;
        for (int i = 0; i < 10; i += 1) {
            goal.target_pose.header.stamp = ros::Time::now();
            px += xwise_incre;
            py += - xwise_incre / slope; // NOTE: watch for division by ~0, like when line is vertical on x-y plane
            goal.target_pose.pose.position.x = px;
            goal.target_pose.pose.position.y = py;
            if (i == 6) { // NOTE: fuck me, i guess we disbale lidar til the very end
                pee.data = false;
                ramp_routine_pub.publish(pee);
            }
            ac.sendGoal(goal);
            ac.waitForResult();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ramp_navigate");
    ros::NodeHandle nh("~");

    RampNavigateNode node;
    node.begin(nh);

    ros::spin();
}