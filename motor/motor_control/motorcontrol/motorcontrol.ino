
#include <ros.h>
#include <ArduinoHardware.h>


#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ros/time.h>

static unsigned long last_ros_count = 0;

// ports (arbitrary numbers as placeholders)
// right
const int RFR =  4;      // digital; motor direction control; F/R == COM == 0 => anticlockwise; clockwise defined as forwards
const int RSV = 5;       // controls speed; 5V = max speed
// left
const int LFR = 7;       // digital; motor direction control; F/R == COM == 0 => anticlockwise; clockwise defined as forwards
const int LSV = 6;       // controls speed; 5V = max speed
// commands
const int light_pin = 13;  

unsigned long current_ros = 0;


// code for light (flashes when rover in autonomous mode, solid when manual)
// light (code based on https://www.arduino.cc/en/Tutorial/BuiltInExamples/BlinkWithoutDelay)
boolean mode = 1;     // 1 -> autonomous mode (light flashes), 0 -> manual mode (light solid)
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
    last_ros_count = current_ros;
    current_ros = millis();    
    float right_input = control_msg.data;
    if(right_input >= 0){
        right_speed = 51*right_input;
        right_dir = 0;
    }else{
        right_speed = -51*right_input;
        right_dir = 1;
    }
}

// get signed float speed command from ros, convert to magnitude and direction for left motor
void lmotorCb(const std_msgs::Float64& control_msg){
    float left_input = control_msg.data;
    if(left_input < 0){
        left_speed = -51*left_input;
        left_dir = 0;
    }else{
        left_speed = 51*left_input;
        left_dir = 1;
    }
}

void setmode(const std_msgs::Bool& mode_msg){
    mode = mode_msg.data; 
}

//ros subs
ros::Subscriber<std_msgs::Float64> right_motor_vel_sub("/right_wheel/command", &rmotorCb );
ros::Subscriber<std_msgs::Float64> left_motor_vel_sub("/left_wheel/command", &lmotorCb );
ros::Subscriber<std_msgs::Bool> mode_sub("/pause_navigation", &setmode );


void setup() {

    Serial.begin(57600);     // Serial comm begin at 9600bps
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.subscribe(right_motor_vel_sub);
    nh.subscribe(left_motor_vel_sub);
    nh.subscribe(mode_sub); 
    pinMode(RFR, OUTPUT);
    pinMode(LFR, OUTPUT);
    pinMode(RSV, OUTPUT);
    pinMode(LSV, OUTPUT);
    pinMode(light_pin, OUTPUT);
    
}

void loop(){
    // control motors
    nh.spinOnce();

    if(nh.connected()){
        nh.spinOnce();
        if (current_ros - last_ros_count >= interval){
            digitalWrite(RFR, 0);
            digitalWrite(LFR, 0);
            analogWrite(RSV, 0);
            analogWrite(LSV, 0);
        } else {
            digitalWrite(RFR, right_dir);
            digitalWrite(LFR, left_dir);
            analogWrite(RSV, right_speed);
            analogWrite(LSV, left_speed);
        }

        // control light
        nh.spinOnce();
        if (!mode) {
                unsigned long currentMillis = millis();
                if (currentMillis - previousMillis >= interval) {
                    previousMillis = currentMillis;
                    if (light_state == LOW) {
                        light_state = HIGH;
                    }
                    else {
                        light_state = LOW;
                    }
                }
            } else {
                light_state = HIGH;
            }

        
            digitalWrite(light_pin, light_state);
        } else {
            digitalWrite(RFR, 0);
            digitalWrite(LFR, 0);
            analogWrite(RSV, 0);
            analogWrite(LSV, 0);
        }
    nh.spinOnce();
}
