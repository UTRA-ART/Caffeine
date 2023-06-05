/*
 * 2023-06-04 
 * convert wheel command topics to odom
 * 
 * subscribed to:
 *   twist_mux/cmd_vel
 * 
 * publishes to:
 *   odom_wheel_encoder_euler
 *   odom_wheel_encoder_quat
 * 
 * resources
 *   tcp_nodelay: https://answers.ros.org/question/360038/what-are-the-differences-between-the-different-transporthints/
 *   covariance matrix: https://answers.ros.org/question/64759/covariance-matrix-for-vo-and-odom/
 *   motion model: https://www.roboticsbook.org/S52_diffdrive_actions.html
 */


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

// publish to
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odom_new;
nav_msgs::Odometry odom_old;

// initial pose
const double initial_x = 0.0;
const double initial_y = 0.0;
const double initial_theta = 0.00000000001;
const double PI = 3.1415926;

// robot physical constants (CHANGE THESE)
const double TICKS_PER_REVOLUTION = 512;
const double WHEEL_RADIUS = 0.125; // (in metres)
const double WHEEL_BASE = 1.0; // (centre of left tire to centre of right tire)
const double TICKS_PER_METRE = 3000; 

// more constants from cmd_vel_to_motor.cpp
const double FACTOR = 1 / .447 * 5280 * 12 / 10 / 3.1415 / 60 * 16; // M/s to rpm (before gearbox) 
const double B = -19.8473; //-10.1124; // y-intecept of control profile 
const double M = 684.4425; //640.449; // slope of control profile 

//constants to convert command speed to approximate actual speed
const double m = 0.7282;
const double b = 0.0736;

// distance both wheels have travelled
double distance_left = 0;
double distance_right = 0;

// ticks per second for each wheel
float ticks_ps_left = 0;   
float ticks_ps_right = 0;

// wheel velocities
double vel_right = 0;
double vel_left = 0;

// cmd velocities
double cmd_vx, cmd_wz, cmd_vl, cmd_vr;

// approximated velocities
double vl, vr;

// has initial pose been received?
bool initial_pose_received = false;

using namespace std;

// get cmd_vel message
void cmd_vel_cb(const geometry_msgs::Twist& msg)
{
    cmd_vx = msg.linear.x;
    cmd_wz = msg.angular.z;
}

// publish odom_new as nav_msgs/odom message in quaternion form
void publish_quat(){
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_new.pose.pose.orientation.z);

    nav_msgs::Odometry quat_odom;
    quat_odom.header.stamp = odom_new.header.stamp;
    quat_odom.header.frame_id = "odom";
    quat_odom.child_frame_id = "base_link";
    quat_odom.pose.pose.position.x = odom_new.pose.pose.position.x;
    quat_odom.pose.pose.position.y = odom_new.pose.pose.position.y;
    quat_odom.pose.pose.position.z = odom_new.pose.pose.position.z;
    quat_odom.pose.pose.orientation.x = q.x();
    quat_odom.pose.pose.orientation.y = q.y();
    quat_odom.pose.pose.orientation.z = q.z();
    quat_odom.pose.pose.orientation.w = q.w();
    quat_odom.twist.twist.linear.x = odom_new.twist.twist.linear.x;
    quat_odom.twist.twist.linear.y = odom_new.twist.twist.linear.y;
    quat_odom.twist.twist.linear.z = odom_new.twist.twist.linear.z;
    quat_odom.twist.twist.angular.x = odom_new.twist.twist.angular.x;
    quat_odom.twist.twist.angular.y = odom_new.twist.twist.angular.y;
    quat_odom.twist.twist.angular.z = odom_new.twist.twist.angular.z;

    // build covariance matrix (use big number if unsure of uncertainty)
    for(int i = 0; i < 36; i++){    
        if(i == 0 || i == 7 || i == 14){
            quat_odom.pose.covariance[i] = 0.01;    // translation accuracy +/- 0.01 m
        }else if(i == 21 || i == 28 || i == 35){
            quat_odom.pose.covariance[i] += 0.1;    // rotation accuracy +/- 0.1 radian
        }else{
            quat_odom.pose.covariance[i] = 0;
        }
    }

    odom_data_pub_quat.publish(quat_odom);

}

// update odometry information
void update_odom(){
    // from cmd_vel_to_motor.cpp
    cmd_vr = ((cmd_vx + (WHEEL_BASE * cmd_wz) / 2.0) * FACTOR - B) / M;
    cmd_vl = ((cmd_vx - (WHEEL_BASE * cmd_wz) / 2.0) * FACTOR - B) /M;

    vr = cmd_vr * m + b;
    vl = cmd_vl * m + b;

    if((cmd_vx == 0.0) && (cmd_wz == 0.0)){
        vr = 0;
        vl = 0;
    }
    

    odom_new.header.stamp = ros::Time::now();
    odom_new.twist.twist.linear.x = WHEEL_RADIUS/2 * (vl + vr);
    odom_new.twist.twist.angular.z = WHEEL_RADIUS/WHEEL_BASE * (vr - vl);

    // save pose data for next cycle
    odom_old.pose.pose.position.x = odom_new.pose.pose.position.x;
    odom_old.pose.pose.position.y = odom_new.pose.pose.position.y;
    odom_old.pose.pose.orientation.z = odom_new.pose.pose.orientation.z;
    odom_old.header.stamp = odom_new.header.stamp;

    // publish odometry message
    odom_data_pub.publish(odom_new);
}

// main
int main(int argc, char **argv){
    // set data fields of odometry message
    odom_new.header.frame_id = "odom";
    odom_new.pose.pose.position.z = 0;
    odom_new.pose.pose.orientation.x = 0;
    odom_new.pose.pose.orientation.y = 0;
    odom_new.twist.twist.linear.x = 0;
    odom_new.twist.twist.linear.y = 0;
    odom_new.twist.twist.linear.z = 0;
    odom_new.twist.twist.angular.x = 0;
    odom_new.twist.twist.angular.y = 0;
    odom_new.twist.twist.angular.z = 0;
    odom_old.pose.pose.position.x = initial_x;
    odom_old.pose.pose.position.y = initial_y;
    odom_old.pose.pose.orientation.z = initial_theta;

    // initialise ros node
    ros::init(argc, argv, "wheel_odom_pub");
    ros::NodeHandle nh;

    // subscribers
    // ros::Subscriber right_vel_sub = nh.subscribe("right_wheel/ticks_ps", 100, right_encoder_cb, ros::TransportHints().tcpNoDelay());    // reduce latency for large messages
    // ros::Subscriber left_vel_sub = nh.subscribe("left_wheel/ticks_ps", 100, left_encoder_cb, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber init_pose_sub = nh.subscribe("initial_2d", 1, set_initial_2d);
    ros::Subscriber cmd_vel_sub = nh.subscribe("twist_mux/cmd_vel", 100, cmd_vel_cb, ros::TransportHints().tcpNoDelay());

    // publishers
    odom_data_pub = nh.advertise<nav_msgs::Odometry>("odom_wheel_encoder_euler", 100);   // simple odom message, orientation.z is an euler angle
    odom_data_pub_quat = nh.advertise<nav_msgs::Odometry>("odom_wheel_encoder_quat", 100);   // full odom message, orientation.z is quaternion

    ros::Rate loop_rate(30);

    while(ros::ok()){
        update_odom();
        publish_quat();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}