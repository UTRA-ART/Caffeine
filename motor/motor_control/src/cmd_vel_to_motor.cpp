#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

double vx = 0.0;
double wz = 0.0;
double wheelbase = 0.91; // meters

void targetcb(const geometry_msgs::Twist& msg)
{
    vx = msg.linear.x;
    wz = msg.angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_to_motor");

    ros::NodeHandle n;
    ros::Time current_time, prev_time;

    current_time = ros::Time::now();

    // Subscribe to cmd_vel topics
    ros::Subscriber cmd_vel_sub = n.subscribe("key_vel", 1, targetcb);

    // Publish to right and left wheels
    ros::Publisher rvel_pub = n.advertise<std_msgs::Float64>("/right_wheel/command", 1);
    ros::Publisher lvel_pub = n.advertise<std_msgs::Float64>("/left_wheel/command", 1);

    ros::Rate loop_rate(100);

    std_msgs::Float64 rvelmsg;
    std_msgs::Float64 lvelmsg;

    double v, vl, vr, rcommand, lcommand;

    while (n.ok())
    {
        // Check for msgs, updates xi, yi, zi, x, y, z
        ros::spinOnce();

        // NOTE: The following expressions are derived from unicycle kinematics
        vr = vx + (wheelbase * wz) / 2.0;
        vl = vx - (wheelbase * wz) / 2.0;

        rvelmsg.data = vr;
        lvelmsg.data = vl;

        rvel_pub.publish(rvelmsg);
        lvel_pub.publish(lvelmsg);

        loop_rate.sleep();
    }

    return 0;
}
