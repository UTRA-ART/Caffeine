/*
 * 2024-05-11
 * gets total number of ticks travelled from each wheel and publishes linear and angular velocity
 * 
 * to do:
 *   fix hardcoded robot constants
 *   modify covariance matrix as necessary
 * 
 * subscribed to:
 *   right_wheel/ticks  (int64)
 *   left_wheel/ticks   (int64)
 * 
 * publishes to:
 *   odom_wheel_encoder_euler   (nav_msgs/Odometry)
 *   odom_wheel_encoder_quat    (nav_msgs/Odometry)
 * 
 * resources
 *   tcp_nodelay: https://answers.ros.org/question/360038/what-are-the-differences-between-the-different-transporthints/
 *   covariance matrix: https://answers.ros.org/question/64759/covariance-matrix-for-vo-and-odom/
 *   motion model: https://www.roboticsbook.org/S52_diffdrive_actions.html
 */

#include <ros/ros.h>
#include <std_msgs/Int64.h>
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

// distance both wheels have travelled
double distance_left = 0;
double distance_right = 0;

using namespace std;

// calculate distance travelled by left wheel since last cycle
void calc_left(const std_msgs::Int64& left_count){
    static int last_count_l = 0;

    int left_ticks = left_count.data - last_count_l;

    distance_left = left_ticks/TICKS_PER_METRE;

    last_count_l = left_count.data;
}

// calculate distance travelled by right wheel since last cycle
void calc_right(const std_msgs::Int64& right_count){
    static int last_count_r = 0;

    int right_ticks = right_count.data - last_count_r;

    distance_right = right_ticks/TICKS_PER_METRE;

    last_count_r = right_count.data;
}

// publish odometry message in quaternion
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

    for(int i = 0; i<36; i++) {
        if(i == 0 || i == 7 || i == 14) {
        quat_odom.pose.covariance[i] = .01;
        }
        else if (i == 21 || i == 28 || i== 35) {
        quat_odom.pose.covariance[i] += 0.1;
        }
        else {
        quat_odom.pose.covariance[i] = 0;
        }
    }

    odom_data_pub_quat.publish(quat_odom);
}

// update odometry information
void update_odom() {
    
    // Calculate the average distance
    double cycle_distance = (distance_right + distance_left) / 2;
    
    // Calculate the number of radians the robot has turned since the last cycle
    double cycle_angle = asin((distance_right-distance_left)/WHEEL_BASE);
    
    // Average angle during the last cycle
    double avg_angle = cycle_angle/2 + odom_old.pose.pose.orientation.z;
        
    if (avg_angle > PI) {
        avg_angle -= 2*PI;
    }
    else if (avg_angle < -PI) {
        avg_angle += 2*PI;
    }
    else{}
    
    // Calculate the new pose (x, y, and theta)
    odom_new.pose.pose.position.x = odom_old.pose.pose.position.x + cos(avg_angle)*cycle_distance;
    odom_new.pose.pose.position.y = odom_old.pose.pose.position.y + sin(avg_angle)*cycle_distance;
    odom_new.pose.pose.orientation.z = cycle_angle + odom_old.pose.pose.orientation.z;
    
    // Prevent lockup from a single bad cycle
    if (isnan(odom_new.pose.pose.position.x) || isnan(odom_new.pose.pose.position.y)
        || isnan(odom_new.pose.pose.position.z)) {
        odom_new.pose.pose.position.x = odom_old.pose.pose.position.x;
        odom_new.pose.pose.position.y = odom_old.pose.pose.position.y;
        odom_new.pose.pose.orientation.z = odom_old.pose.pose.orientation.z;
    }
    
    // Make sure theta stays in the correct range
    if (odom_new.pose.pose.orientation.z > PI) {
        odom_new.pose.pose.orientation.z -= 2 * PI;
    }
    else if (odom_new.pose.pose.orientation.z < -PI) {
        odom_new.pose.pose.orientation.z += 2 * PI;
    }
    else{}
    
    // Compute the velocity
    odom_new.header.stamp = ros::Time::now();
    odom_new.twist.twist.linear.x = cycle_distance/(odom_new.header.stamp.toSec() - odom_old.header.stamp.toSec());
    odom_new.twist.twist.angular.z = cycle_angle/(odom_new.header.stamp.toSec() - odom_old.header.stamp.toSec());
    
    // Save the pose data for the next cycle
    odom_old.pose.pose.position.x = odom_new.pose.pose.position.x;
    odom_old.pose.pose.position.y = odom_new.pose.pose.position.y;
    odom_old.pose.pose.orientation.z = odom_new.pose.pose.orientation.z;
    odom_old.header.stamp = odom_new.header.stamp;
    
    // Publish the odometry message
    odom_data_pub.publish(odom_new);
}

int main(int argc, char **argv){
    // set initial odometry message pose and twist
    odom_new.header.frame_id = "odom";
    odom_old.pose.pose.position.x = 0;
    odom_old.pose.pose.postion.y = 0;
    odom_new.pose.pose.position.z = 0;
    odom_new.pose.pose.orientation.x = 0;
    odom_new.pose.pose.orientation.y = 0;
    odom_old.pos.pose.orientation.z = 0;
    odom_new.twist.twist.linear.x = 0;
    odom_new.twist.twist.linear.y = 0;
    odom_new.twist.twist.linear.z = 0;
    odom_new.twist.twist.angular.x = 0;
    odom_new.twist.twist.angular.y = 0;
    odom_new.twist.twist.angular.z = 0;
    
    // launch ros and create node
    ros::init(argc, argv, "wheel_odom_pub");
    ros::NodeHandle nh;

    // subscribe to ros topics
    ros::Subscriber right_tick_sub = nh.subscribe("right_wheel/ticks", 100, calc_right, ros::TransportHints().tcpNoDelay());
    ros::Subscriber left_tick_sub = nh.subscribe("left_wheel/ticks", 100, calc_left, ros::TransportHints().tcpNoDelay());
    
    // publishers
    odom_data_pub = nh.advertise<nav_msgs::Odometry>("wheel_odom/euler", 100);   // simple odom message, orientation.z is an euler angle
    odom_data_pub_quat = nh.advertise<nav_msgs::Odometry>("wheel_odom/quat", 100);   // full odom message, orientation.z is quaternion

    ros::Rate loop_rate(30);

    while(ros::ok()){
        update_odom();
        publish_quat();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}