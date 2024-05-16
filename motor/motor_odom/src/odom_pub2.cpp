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
 */


#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
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

// robot physical constants (CHANGE THESE)
// Assuming 200 pulses per rotation (estimate)
const double TICKS_PER_REVOLUTION = 200;
// Approximately 9.8inches = 24.892cm diameter (estimate)
const double WHEEL_RADIUS = 0.125; // (in metres)
const double WHEEL_BASE = 1.0; // (centre of left tire to centre of right tire)

const double ANGULAR_VEL_PER_PULSE = 360 / TICKS_PER_REVOLUTION;
const double METRES_PER_TICK = ((ANGULAR_VEL_PER_PULSE) / 360) * (2 * PI * WHEEL_RADIUS);
// TICKS_PER_METRE = ticks per 
const double TICKS_PER_METRE = 1 / METRES_PER_TICK; 

// distance both wheels have travelled
double distance_left = 0;
double distance_right = 0;

// ticks per second for each wheel
float ticks_ps_left = 0;   
float ticks_ps_right = 0;

// direction for each wheel
int l_direction = 0;
int r_direction = 0;

// wheel velocities
double vel_right = 0;
double vel_left = 0;

// has initial pose been received?
bool initial_pose_received = false;

using namespace std;



// get current ticks/second from each wheel
void right_encoder_cb(const std_msgs::Float64& right_ticks){
    // ticks per second from hall effect sensor for right wheel
    ticks_ps_right = right_ticks.data;
    // velocity = ticks per second / ticks per metre = metres per second
    vel_right = ticks_ps_right / TICKS_PER_METRE;
}

void left_encoder_cb(const std_msgs::Float64& left_ticks){
    // ticks per second from hall effect sensor for left wheel
    ticks_ps_left = left_ticks.data;
    vel_left = ticks_ps_left / TICKS_PER_METRE;
}

// get the direction (positive or negative) from each wheel
void right_direction_cb(const std_msgs::Float64& left_wheel_cmd){
    if(left_wheel_cmd.data > 0){
        l_direction = 1;
    }
    else{
        l_direction = -1;
    }
}

void left_direction_cb(const std_msgs::Float64& right_wheel_cmd){
    std::stringstream ss;
    if(right_wheel_cmd.data > 0){
        r_direction = 1;
    }
    else{
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
    // distance = direction * velocity * time
    distance_left = l_direction * vel_left * duration;
    distance_right = r_direction * vel_right * duration;

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
    odom_new.twist.twist.linear.x = WHEEL_RADIUS/2 * ((l_direction * vel_left) + (r_direction * vel_right));
    odom_new.twist.twist.angular.z = WHEEL_RADIUS/WHEEL_BASE * ((r_direction * vel_right) - (l_direction * vel_left));

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
    ros::Subscriber right_vel_sub = nh.subscribe("right_wheel/ticks_ps", 100, right_encoder_cb, ros::TransportHints().tcpNoDelay());    // reduce latency for large messages
    ros::Subscriber left_vel_sub = nh.subscribe("left_wheel/ticks_ps", 100, left_encoder_cb, ros::TransportHints().tcpNoDelay());       // 100 is queue size
    // wheel commands to get direction for both wheels
    ros::Subscriber r_vel = nh.subscribe("/right_wheel/command", 100, right_direction_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber l_vel = nh.subscribe("/left_wheel/command", 100, left_direction_cb, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber init_pose_sub = nh.subscribe("initial_2d", 1, set_initial_2d);
    
    // initial timestamp? not sure how to initialize the time, time taken mostly in loop?
    last = time(NULL);
    current = last;

    // publishers
    odom_data_pub = nh.advertise<nav_msgs::Odometry>("odom_wheel_encoder_euler", 100);   // simple odom message, orientation.z is an euler angle
    odom_data_pub_quat = nh.advertise<nav_msgs::Odometry>("odom_wheel_encoder_quat", 100);   // full odom message, orientation.z is quaternion

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

    }

    return 0;

}