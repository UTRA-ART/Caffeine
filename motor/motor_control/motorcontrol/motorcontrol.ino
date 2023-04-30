#include <ros.h>
#include <ArduinoHardware.h>
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>


const int BAUD_RATE = 57600;
// ports (arbitrary numbers as placeholders)
// right
const int RFR =  4;      // digital; motor direction control; F/R == COM == 0 => anticlockwise; clockwise defined as forwards
const int RSV = 5;       // controls speed; 5V = max speed
// left
const int LFR = 7;       // digital; motor direction control; F/R == COM == 0 => anticlockwise; clockwise defined as forwards
const int LSV = 6;       // controls speed; 5V = max speed
// commands
const int LIGHT_PIN = 13; 

const long INTERVAL = 1000;         // INTERVAL at which to blink (milliseconds)
static unsigned long g_last_ros_count = 0;
unsigned long g_current_ros = 0;
unsigned long g_previousMillis = 0;   // stores last time light was updated

// code for light (flashes when rover in autonomous g_mode, solid when manual)
// light (code based on https://www.arduino.cc/en/Tutorial/BuiltInExamples/BlinkWithoutDelay)
boolean g_mode = 1;     // 1 -> autonomous g_mode (light flashes), 0 -> manual g_mode (light solid)
            // digital; placeholder pin number
int g_light_state = LOW;                // used to set light

float g_right_speed = 0;
float g_left_speed = 0;
int g_right_dir = 0;
int g_left_dir = 0;

// motor speeds and commands (connect with ros)
ros::NodeHandle nh;

// get signed float speed command from ros, convert to magnitude and direction for right motor
void rmotorCb(const std_msgs::Float64& control_msg){
    g_last_ros_count = g_current_ros;
    g_current_ros = millis();    
    float right_input = control_msg.data;
    if(right_input >= 0){
        g_right_speed = 51*right_input;
        g_right_dir = 0;
    }else{
        g_right_speed = -51*right_input;
        g_right_dir = 1;
    }
}

// get signed float speed command from ros, convert to magnitude and direction for left motor
void lmotorCb(const std_msgs::Float64& control_msg){
    float left_input = control_msg.data;
    if(left_input < 0){
        g_left_speed = -51*left_input;
        g_left_dir = 0;
    }else{
        g_left_speed = 51*left_input;
        g_left_dir = 1;
    }
}

void setmode(const std_msgs::Bool& mode_msg){
    g_mode = mode_msg.data; 
}

//ros subs
ros::Subscriber<std_msgs::Float64> right_motor_vel_sub("/right_wheel/command", &rmotorCb );
ros::Subscriber<std_msgs::Float64> left_motor_vel_sub("/left_wheel/command", &lmotorCb );
ros::Subscriber<std_msgs::Bool> mode_sub("/pause_navigation", &setmode );


void setup() {
    Serial.begin(BAUD_RATE);     // Serial comm begin at 9600bps
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();
    nh.subscribe(right_motor_vel_sub);
    nh.subscribe(left_motor_vel_sub);
    nh.subscribe(mode_sub); 
    pinMode(RFR, OUTPUT);
    pinMode(LFR, OUTPUT);
    pinMode(RSV, OUTPUT);
    pinMode(LSV, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);
    
}

void loop(){
    // control motors
    nh.spinOnce();

    if(nh.connected()){
        // If the ros node is still running, output speed 
        nh.spinOnce(); // Spin multiple times because rosserial has very limited packet sizes, can't afford an overflow
        if (g_current_ros - g_last_ros_count >= INTERVAL){
            // If a command has not been recieved in 1s, something is probably wrong. Stop the motors. 
            digitalWrite(RFR, 0);
            digitalWrite(LFR, 0);
            analogWrite(RSV, 0);
            analogWrite(LSV, 0);
        } else {
            // Write speed and direction pins 
            digitalWrite(RFR, g_right_dir);
            digitalWrite(LFR, g_left_dir);
            analogWrite(RSV, g_right_speed);
            analogWrite(LSV, g_left_speed);
        }

        // control light
        nh.spinOnce();
        if (!g_mode) {
                // Autonomous mode (flashing light) 
                unsigned long currentMillis = millis();
                if (currentMillis - g_previousMillis >= INTERVAL) {
                    g_previousMillis = currentMillis;
                    if (g_light_state == LOW) {
                        g_light_state = HIGH;
                    }
                    else {
                        g_light_state = LOW;
                    }
                }
            } else {
                // Manual control (solid light) 
                g_light_state = HIGH;
            }
        
            digitalWrite(LIGHT_PIN, g_light_state);
        } else {
            // If ros node has crashed, stop the motors
            digitalWrite(RFR, 0);
            digitalWrite(LFR, 0);
            analogWrite(RSV, 0);
            analogWrite(LSV, 0);
        }
    nh.spinOnce();
}
