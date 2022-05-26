#include <ros.h>
#include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>





static unsigned long ros_count = 0;

// ports (arbitrary numbers as placeholders)
// right
const int RSPEED = A0;    // outputs rotating speed as pulse frequency
const int RFR =  4;      // digital; motor direction control; F/R == COM == 0 => anticlockwise; clockwise defined as forwards
const int RSV = 5;       // controls speed; 5V = max speed
// left
const int LSPEED = A1;    // outputs rotating speed as pulse frequency
const int LFR = 7;       // digital; motor direction control; F/R == COM == 0 => anticlockwise; clockwise defined as forwards
const int LSV = 6;       // controls speed; 5V = max speed
// commands
const int RF = 'e';      // right motor forwards
const int LF = 'q';      // left motor forwards
const int RO = 'd';      // right motor off
const int LO = 'a';      // left motor off
const int RB = 'c';      // right motor backwards
const int LB = 'z';      // left motor backwards
const int BF = 'w';      // both motors forwards
const int BO = 's';      // both motors off
const int BB = 'x';      // both motors backwards
const int light_pin = 13;  
int speed = 0;      // arbitrary placeholder

int loops_until_cmd = 0;

unsigned long current_ros = 0;

//   TO DOOOOOOOOOOO
//
//
////measure millis from the last ros command (see if functions are called) if bigger than 200 ms then shut the robot off
//
//

// control motors using keyboard input
void keyboard_control(){
    if(Serial.available() > 0){  // if serial value available
        int val = Serial.read(); // read serial value
        switch(val){    // make motor move at received speed
            case RF:
                digitalWrite(RFR, HIGH);
                analogWrite(RSV, speed);
                break;
            case LF:
                digitalWrite(LFR, HIGH);
                analogWrite(LSV, speed);
                break;
            case RO: 
                analogWrite(RSV, 0);
                break; 
            case LO:
                analogWrite(LSV, 0);
                break; 
            case RB:
                digitalWrite(RFR, LOW);
                analogWrite(RSV, speed);
                break;
            case LB:
                digitalWrite(LFR, LOW);
                analogWrite(LSV, speed);
                break;
            case BF:
                digitalWrite(RFR, HIGH);
                digitalWrite(LFR, HIGH);
                analogWrite(RSV, speed);
                analogWrite(LSV, speed);
                break;
            case BO:
                analogWrite(RSV, 0);
                analogWrite(LSV, 0);
                break;
            case BB:
                digitalWrite(RFR, LOW);
                digitalWrite(LFR, LOW);
                analogWrite(RSV, speed);
                analogWrite(LSV, speed);
                break;
        }
    }
}

// code for light (flashes when rover in autonomous mode, solid when manual)
// light (code based on https://www.arduino.cc/en/Tutorial/BuiltInExamples/BlinkWithoutDelay)
int mode = 1;     // 1 -> autonomous mode (light flashes), 0 -> manual mode (light solid)
            // digital; placeholder pin number
int light_state = LOW;                // used to set light
unsigned long previousMillis = 0;   // stores last time light was updated
const long interval = 1000;         // interval at which to blink (milliseconds)

// motor speeds and commands (connect with ros)
ros::NodeHandle nh;
float right_speed = 0;
float left_speed = 0;
int right_dir = 0;
int left_dir = 0;

// get signed float speed command from ros, convert to magnitude and direction for right motor
void rmotorCb(const std_msgs::Float64& control_msg){
    ros_count = current_ros - ros_count;
    
    float right_input = control_msg.data;
    if(right_input >= 0){
        right_speed = (80.5*right_input);
        right_dir = 0;
    }else{
        right_speed = (-80.5*right_input);
        right_dir = 1;
    }
}

// get signed float speed command from ros, convert to magnitude and direction for left motor
void lmotorCb(const std_msgs::Float64& control_msg){
    float left_input = control_msg.data;
    if(left_input < 0){
        left_speed = (-80.5*left_input);
        left_dir = 0;
    }else{
        left_speed = (80.5*left_input);
        left_dir = 1;
    }
}

//std_msgs::Float64MultiArray speed_feedback;

ros::Subscriber<std_msgs::Float64> sub("/right_wheel/command", &rmotorCb );
ros::Subscriber<std_msgs::Float64> sub2("/left_wheel/command", &lmotorCb );
//ros::Publisher speed_pub("/arduino_speed_feedback", &speed_feedback);
 ros::NodeHandle n;
 ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
 tf::TransformBroadcaster odom_broadcaster;



double x = 0.0;
double y = 0.0;
double th = 0.0;
   
double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

ros::Time current_time, last_time;
current_time = ros::Time::now();
last_time = ros::Time::now();
  
 ros::Rate r(1.0);

void setup(){
    Serial.begin(9600);     // Serial comm begin at 9600bps

    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(sub2);
//    nh.advertise(speed_pub);
    
    pinMode(RFR, OUTPUT);
    pinMode(LFR, OUTPUT);
    pinMode(RSV, OUTPUT);
    pinMode(LSV, OUTPUT);
    pinMode(light_pin, OUTPUT);
    pinMode(LSPEED, INPUT);
    pinMode(RSPEED, INPUT);
    
}

void loop(){
    // control motors
//      reads motor speed feedback and publishes it to ros
      float Lspeedread = analogRead(LSPEED);
      float Rspeedread = analogRead(RSPEED);
      speed_feedback.data_length = 2;

      

      
    if (n.ok()){
      nh.spinOnce();
      if (ros_count >= interval){
        digitalWrite(RFR, 0);
        digitalWrite(LFR, 0);
        analogWrite(RSV, 0);
        analogWrite(LSV, 0);
      }
      
      digitalWrite(RFR, right_dir);
      digitalWrite(LFR, left_dir);
      analogWrite(RSV, right_speed);
      analogWrite(LSV, left_speed);
       current_time = ros::Time::now();
  //start
       //compute odometry in a typical way given the velocities of the robot
       
       double dt = (current_time - last_time).toSec();

       float feedback_speed = (Lspeedread + Rspeedread)/2;
       float feedback_angular_vel = (Rspeedread - Lspeedread)/0.889;
       float th = feedback_angular_vel * dt;
       
       double delta_x = (feedback_speed * cos(th) - feedback_speed * sin(th)) * dt;
       double delta_y = (feedback_speed * sin(th) + feedback_speed * cos(th)) * dt;
       double delta_th = feedback_angular_vel * dt;
   
       x += delta_x;
       y += delta_y;
       th += delta_th;
   
       //since all odometry is 6DOF we'll need a quaternion created from yaw
       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
   
       //first, we'll publish the transform over tf
       geometry_msgs::TransformStamped odom_trans;
       odom_trans.header.stamp = current_time;
       odom_trans.header.frame_id = "odom";
       odom_trans.child_frame_id = "base_link";
   
       odom_trans.transform.translation.x = x;
       odom_trans.transform.translation.y = y;
       odom_trans.transform.translation.z = 0.0;
       odom_trans.transform.rotation = odom_quat;
   
       //send the transform
       odom_broadcaster.sendTransform(odom_trans);
   
       //next, we'll publish the odometry message over ROS
       nav_msgs::Odometry odom;
       odom.header.stamp = current_time;
       odom.header.frame_id = "odom";
   
       //set the position
       odom.pose.pose.position.x = x;
       odom.pose.pose.position.y = y;
       odom.pose.pose.position.z = 0.0;
       odom.pose.pose.orientation = odom_quat;
   
       //set the velocity
       odom.child_frame_id = "base_link";
       odom.twist.twist.linear.x = feedback_speed;
       odom.twist.twist.linear.y = feedback_speed;
       odom.twist.twist.angular.z = vth;
  
      //publish the message
       odom_pub.publish(odom);
   
       last_time = current_time;
       r.sleep();
     }

  //    speed_feedback.data[0] = Lspeedread;
  //    speed_feedback.data[1] = Rspeedread;
  
      
      // control light
      if(mode){
          unsigned long currentMillis = millis();
          current_ros = currentMillis;
          if(currentMillis - previousMillis >= interval){
              previousMillis = currentMillis;
              if(light_state == LOW){
                  light_state = HIGH;
              }
              else{
                  light_state = LOW;
              }
          }
      }else{
          light_state = HIGH;
      }
      digitalWrite(light_pin, light_state);
      delay(1);
}
