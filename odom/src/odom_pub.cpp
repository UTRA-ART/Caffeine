/*
 * 2023-04-29
 * gets ticks per second from each wheel and publishes linear and angular velocity
 * from https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
 * note: tutorial is in Melodic
 * 
 * to do:
 *   fix hardcoded robot constants
 *   modify covariance matrix as necessary
 * 
 * subscribed to:
 *   right_wheel/ticks_ps  (ticks per second, Float32)
 *   left_wheel/ticks_ps
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
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
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

// distance both wheels have travelled
double distance_left = 0;
double distance_right = 0;

// ticks per second for each wheel
float ticks_ps_left = 0;   
float ticks_ps_right = 0;

// wheel velocities
double vel_right = 0;
double vel_left = 0;

// has initial pose been received?
bool initial_pose_received = false;

using namespace std;

// // get initial 2d message from either rviz clicks or a manual pose publisher
// void set_initial_2d(const geometry_msgs::PoseStamped &rviz_click){
//     odom_old.pose.pose.position.x = rviz_click.pose.position.x;
//     odom_old.pose.pose.position.y = rviz_click.pose.position.y;
//     odom_old.pose.pose.orientation.z = rviz_click.pose.orientation.z;
//     initial_pose_received = true;
// }

// // calculate how far the wheels have travelled since last cycle
// void calc_left(const std_msgs::Int16& left_count){
//     static int last_count_left = 0;
//     if(left_count.data != 0 && last_count_left != 0){
//         int left_ticks = (left_count.data - last_count_left);
        
//         if(left_ticks > 10000){
//             left_ticks = 0 - (65535 - left_ticks);
//         }else if(left_ticks < -10000){
//             left_ticks = 65535 - left_ticks;
//         }

//         distance_left = left_ticks/TICKS_PER_METRE;
//     }

//     last_count_left = left_count.data;
// }

// void calc_right(const std_msgs::Int16& right_count){
//     static int last_count_right = 0;
//     if(right_count.data != 0 && last_count_right != 0){
//         int right_ticks = right_count.data - last_count_right;

//         if(right_ticks > 10000){
//             right_ticks = 0 - (65535 - right_ticks);
//         }else if(right_ticks < -10000){
//             right_ticks = 65535 - right_ticks;
//         }

//         distance_right = right_ticks/TICKS_PER_METRE;
//     }

//     last_count_right = right_count.data;
// }

// get current ticks/second from each wheel
void right_encoder_cb(const std_msgs::Float32& right_ticks){
    ticks_ps_right = right_ticks.data;
    vel_right = ticks_ps_right / TICKS_PER_METRE;
}

void left_encoder_cb(const std_msgs::Float32& left_ticks){
    ticks_ps_left = left_ticks.data;
    vel_left = ticks_ps_left / TICKS_PER_METRE;
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
    // average distance since last cycle
    double cycle_distance = (distance_right + distance_left) / 2;
    // number of radians the robot has turned since the last cycle
    double cycle_angle = asin((distance_right - distance_left)/WHEEL_BASE);
    // average angle during the last cycle
    double avg_angle = cycle_angle/2 + odom_old.pose.pose.orientation.z;


    if(avg_angle > PI){
        avg_angle -= 2*PI;
    }else if(avg_angle < -PI){
        avg_angle += 2*PI;
    }

    // calculate new pose
    odom_new.pose.pose.position.x = odom_old.pose.pose.position.x + cos(avg_angle)*cycle_distance;
    odom_new.pose.pose.position.y = odom_old.pose.pose.position.y + sin(avg_angle)*cycle_distance;
    odom_new.pose.pose.orientation.z = cycle_angle + odom_old.pose.pose.orientation.z;

    // prevent lockup from a single bad cycle
    if(isnan(odom_new.pose.pose.position.x) || isnan(odom_new.pose.pose.position.y) 
        || isnan(odom_new.pose.pose.position.z)){
            odom_new.pose.pose.position.x = odom_old.pose.pose.position.x;
            odom_new.pose.pose.position.y = odom_old.pose.pose.position.y;
            odom_new.pose.pose.orientation.z = odom_old.pose.pose.orientation.z;
    }

    // ensure theta stays in the correct range
    if(odom_new.pose.pose.orientation.z > PI){
        odom_new.pose.pose.orientation.z -= 2*PI;
    }else if(odom_new.pose.pose.orientation.z < -PI){
        odom_new.pose.pose.orientation.z += 2*PI;
    }

    // compute velocity
    // odom_new.header.stamp = ros::Time::now();
    // odom_new.twist.twist.linear.x = cycle_distance/(odom_new.header.stamp.toSec() - odom_old.header.stamp.toSec());
    // odom_new.twist.twist.angular.z = cycle_angle/(odom_new.header.stamp.toSec() - odom_old.header.stamp.toSec());
    odom_new.header.stamp = ros::Time::now();
    odom_new.twist.twist.linear.x = WHEEL_RADIUS/2 * (vel_left + vel_right);
    odom_new.twist.twist.angular.z = WHEEL_RADIUS/WHEEL_BASE * (vel_right - vel_left);

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
    ros::Subscriber right_vel_sub = nh.subscribe("right_wheel/ticks_ps", 100, right_encoder_cb, ros::TransportHints().tcpNoDelay());    // reduce latency for large messages
    ros::Subscriber left_vel_sub = nh.subscribe("left_wheel/ticks_ps", 100, left_encoder_cb, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber init_pose_sub = nh.subscribe("initial_2d", 1, set_initial_2d);

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