/*
 * 2023-04-29
 * gets ticks per second from each wheel and publishes linear and angular velocity
 * from https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
 * note: tutorial is in Melodic
 * 
 * subscribed to:
 *   right_wheel/ticks_ps  (ticks per second, Float64)
 *   left_wheel/ticks_ps
 *   right_wheel/command
 *   left_wheel/command
 * 
 * publishes to:
 *   odom_wheel_encoder_euler
 *   odom_wheel_encoder_quat
 * 
 * resources
 *   tcp_nodelay: https://answers.ros.org/question/360038/what-are-the-differences-between-the-different-transporthints/
 *   covariance matrix: https://answers.ros.org/question/64759/covariance-matrix-for-vo-and-odom/
 *   motion model: https://www.roboticsbook.org/S52_diffdrive_actions.html
 * 
 * 2024-04-10
 * fix direction issue - direction retrieved from command messages and
 * and combined with directionless ticks/s from hall effect sensors
 * 
 * 2024-05-20
 * subscribe to wheel/ticks instead, not necessarily ticks_ps
 */


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <time.h>

#include <std_msgs/String.h>
#include <sstream>

// publish to
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odom_new;
// where to get old odometry?
nav_msgs::Odometry odom_old;

ros::Publisher debug_pub;
std_msgs::String debug_msg;
std::stringstream ss;

// timestamps
time_t last;
time_t current;
// duration to calculate distance
long int duration = 0;

// initial pose
const double initial_x = 0.0;
const double initial_y = 0.0;
const double initial_theta = 0.00000000001;
const double PI = 3.1415926;

// Approximately 9.8inches = 24.892cm diameter (estimate)
const double WHEEL_RADIUS = 0.125; // (in metres)
const double CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
const double WHEEL_BASE = 1.0; // (centre of left tire to centre of right tire)

// defines slope of rpm/(ticks per second) reading, with intercept set to 0
const double A = 0.3114;
const double TICKS_PER_METRE = 1 / A / CIRCUMFERENCE * 60;

// distance both wheels have travelled
double distance_left = 0;
double distance_right = 0;

// ticks per second for each wheel
float ticks_left = 0;   
float ticks_right = 0;

// direction for each wheel
int l_direction = 0;
int r_direction = 0;

// wheel velocities
double rpm_right = 0;
double rpm_left = 0;
double vel_right = 0;
double vel_left = 0;

// has initial pose been received?
bool initial_pose_received = false;

using namespace std;



// get current ticks/second from each wheel
void right_ticks_cb(const std_msgs::Int32& right_ticks){
    // ticks from sensor since last message for right wheel
    ticks_right = right_ticks.data;
    // convert ticks to metres per second
    distance_right = ticks_right / TICKS_PER_METRE;
}

void left_ticks_cb(const std_msgs::Int32& left_ticks){
    // ticks from sensor since last message for left wheel
    ticks_left = left_ticks.data;
    distance_left = ticks_left / TICKS_PER_METRE;
}

// get the direction (positive or negative) from each wheel
void left_direction_cb(const std_msgs::Bool& left_dir_msg){
    if(left_dir_msg.data){
        l_direction = 1;
    }else{
        l_direction = -1;
    }
}

void right_direction_cb(const std_msgs::Bool& right_dir_msg){
    if(right_dir_msg.data){
        r_direction = 1;
    }else{
        r_direction = -1;
    }
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
    double cycle_distance = ((r_direction * distance_right) + (l_direction * distance_left)) / 2;
    // number of radians the robot has turned since the last cycle
    double cycle_angle = asin(((r_direction * distance_right) - (l_direction * distance_left))/WHEEL_BASE);
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
    odom_new.header.stamp = ros::Time::now();
    odom_new.twist.twist.linear.x = cycle_distance/(odom_new.header.stamp.toSec() - odom_old.header.stamp.toSec());
    odom_new.twist.twist.angular.z = cycle_angle/(odom_new.header.stamp.toSec() - odom_old.header.stamp.toSec());

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
    // ticks per second from both wheels
    ros::Subscriber right_vel_sub = nh.subscribe("/right_wheel/ticks", 100, right_ticks_cb, ros::TransportHints().tcpNoDelay());    // reduce latency for large messages
    ros::Subscriber left_vel_sub = nh.subscribe("/left_wheel/ticks", 100, left_ticks_cb, ros::TransportHints().tcpNoDelay());       // 100 is queue size
    // wheel commands to get direction for both wheels
    ros::Subscriber r_vel = nh.subscribe("/right_wheel/direction", 100, right_direction_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber l_vel = nh.subscribe("/left_wheel/direction", 100, left_direction_cb, ros::TransportHints().tcpNoDelay());
    
    // initial timestamp? not sure how to initialize the time, time taken mostly in loop?
    last = time(NULL);
    current = last;

    // publishers
    odom_data_pub = nh.advertise<nav_msgs::Odometry>("wheel_odom/euler", 100);   // simple odom message, orientation.z is an euler angle
    odom_data_pub_quat = nh.advertise<nav_msgs::Odometry>("wheel_odom/quat", 100);   // full odom message, orientation.z is quaternion

    debug_pub = nh.advertise<std_msgs::String>("debug_wheel_odom", 100);

    ros::Rate loop_rate(30);

    while(ros::ok()){
        // distance is updated in update_odom
        // skip the update if either time function fail and return -1 
        if(last == -1 || current == -1){

        }
        else{
            update_odom();
        }

        publish_quat();

        ros::spinOnce();
        loop_rate.sleep();
        
        // initial time becomes last time
        last = current;
        // last time is updated
        current = time(NULL);
        // duration is the difference, used in next loop for update_odom
        // typecast to integer in order to use in calculations
        duration = static_cast<long int> (current - last);

        ROS_DEBUG("left_dir: %d\tright_dir: %d", l_direction, r_direction);

    }

    return 0;

}