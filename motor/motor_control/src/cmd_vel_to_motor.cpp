#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

double g_vx = 0.0;
double g_wz = 0.0;
const double WHEELBASE = 1; //0.89; // meters

// For more info see https://www.ato.com/Content/doc/ATO-BLD750-BLDC-motor-controller-specs.pdf 
const double FACTOR = 1 / .447 * 5280 * 12 / 10 / 3.1415 / 60 * 16; // M/s to rpm (before gearbox) 
const double B = -19.8473; //-10.1124; // y-intecept of control profile 
const double M = 684.4425; //640.449; // slope of control profile 

void targetcb(const geometry_msgs::Twist& msg)
{
    g_vx = msg.linear.x;
    g_wz = msg.angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_to_motor");

    ros::NodeHandle n;
    ros::Time current_time, prev_time;

    current_time = ros::Time::now();

    // Subscribe to cmd_vel topics
    ros::Subscriber cmd_vel_sub = n.subscribe("/twist_mux/cmd_vel", 1, targetcb);

    // Publish to right and left wheels
    ros::Publisher rvel_pub = n.advertise<std_msgs::Float64>("/right_wheel/command", 1);
    ros::Publisher lvel_pub = n.advertise<std_msgs::Float64>("/left_wheel/command", 1);
    // ros::Publisher debug_pub = n.advertise<std_msgs::String>("debug", 1);
    ros::Publisher debug_pub = n.advertise<std_msgs::Float64>("/debug2", 1);

    ros::Rate loop_rate(100);

    std_msgs::Float64 rvelmsg;
    std_msgs::Float64 lvelmsg;
    std_msgs::Float64 debugmsg;

    double v, vl, vr, rcommand, lcommand;

    while (n.ok())
    {
        // Check for msgs, updates xi, yi, zi, x, y, z
        ros::spinOnce();

        // NOTE: The following expressions are derived from unicycle kinematics
        vr = g_vx + (WHEELBASE * g_wz) / 2.0;
        vl = g_vx - (WHEELBASE * g_wz) / 2.0;

        rvelmsg.data = (vr * FACTOR - B) / M;
        lvelmsg.data = (vl * FACTOR - B) / M;

        // if(vr == 0) rvelmsg.data = 0;
        // if(vl == 0) lvelmsg.data = 0;
        if((g_vx == 0.0) && (g_wz == 0.0)){
            rvelmsg.data = 0.0;
            lvelmsg.data = 0.0;
        }

        rvel_pub.publish(rvelmsg);
        lvel_pub.publish(lvelmsg);

        // debugmsg.data = std::to_string(g_vx) + std::to_string(g_wz);
        debugmsg.data = g_vx;
        debug_pub.publish(debugmsg);


        loop_rate.sleep();
    }

    return 0;
}
