#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ros/time.h>

static unsigned long last_ros_count = 0;

// ports (arbitrary numbers as placeholders)
// right
const int RSPEED = A0;    // outputs rotating speed as pulse frequency
const int RFR =  4;      // digital; motor direction control; F/R == COM == 0 => anticlockwise; clockwise defined as forwards
const int RSV = 5;       // controls speed; 5V = max speed
// left
const int LSPEED = A5;    // outputs rotating speed as pulse frequency
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
int mode = 0;     // 1 -> autonomous mode (light flashes), 0 -> manual mode (light solid)
            // digital; placeholder pin number
int light_state = LOW;                // used to set light
unsigned long previousMillis = 0;   // stores last time light was updated
const long interval = 1000;         // interval at which to blink (milliseconds)

// motor speeds and commands (connect with ros)
ros::NodeHandle nh;
float right_speed = 0;
float left_speed = 0;
int Lspeedread;
int Rspeedread;
int right_dir = 0;
int left_dir = 0;

// get signed float speed command from ros, convert to magnitude and direction for right motor
void rmotorCb(const std_msgs::Float64& control_msg){
    last_ros_count = current_ros;
    current_ros = millis();    
    float right_input = control_msg.data;
    if(right_input >= 0){
        right_speed = right_input;
        right_dir = 0;
    }else{
        right_speed = -right_input;
        right_dir = 1;
    }
}

// get signed float speed command from ros, convert to magnitude and direction for left motor
void lmotorCb(const std_msgs::Float64& control_msg){
    float left_input = control_msg.data;
    if(left_input < 0){
        left_speed = -left_input;
        left_dir = 0;
    }else{
        left_speed = left_input;
        left_dir = 1;
    }
}

/*void setmode(const std_msgs::Float64& mode_msg){
    mode = mode_msg.data; 
}*/

//ros subs
ros::Subscriber<std_msgs::Float64> right_motor_vel_sub("/right_wheel/command", &rmotorCb );
ros::Subscriber<std_msgs::Float64> left_motor_vel_sub("/left_wheel/command", &lmotorCb );
//ros::Subscriber<std_msgs::Float64> mode_sub("/system/operating_mode", &setmode );

std_msgs::Float64 str_msg;
//std_msgs::Float64MultiArray speed_feedback;
std_msgs::Int16MultiArray speed_feedback;

//ros pubs
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher speed_pub("/arduino_speed_feedback", &speed_feedback);


char hello[13] = "hello world!";

void setup() {

    Serial.begin(115200);     // Serial comm begin at 9600bps
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(right_motor_vel_sub);
    nh.subscribe(left_motor_vel_sub);
    //nh.subscribe(mode_sub); 
    nh.advertise(chatter);
    nh.advertise(speed_pub);
    //nh.negotiateTopics();

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
    nh.spinOnce();

    
    if(nh.connected()){
      nh.spinOnce();
      //str_msg.data = left_speed; 
      //chatter.publish( &str_msg );
      //nh.spinOnce();
      if (current_ros - last_ros_count >= interval){
        digitalWrite(RFR, 0);
        digitalWrite(LFR, 0);
        analogWrite(RSV, 0);
        analogWrite(LSV, 0);
      }else{
        digitalWrite(RFR, right_dir);
        digitalWrite(LFR, left_dir);
        analogWrite(RSV, right_speed);
        analogWrite(LSV, left_speed);
      }
      nh.spinOnce();

      //reads motor speed feedback and publishes it to ros
      Lspeedread = analogRead(LSPEED);
      Rspeedread = analogRead(RSPEED);
      speed_feedback.data_length = 2;
      nh.spinOnce();
      
      int speeds[] = { Lspeedread, Rspeedread };
      
      speed_feedback.data = speeds;
      //speed_feedback.data[1] = Rspeedread;
      speed_pub.publish(&speed_feedback); 
      nh.spinOnce();
      
      // control light
      nh.spinOnce();
      if(mode){
          unsigned long currentMillis = millis();
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
    }else{
        digitalWrite(RFR, 0);
        digitalWrite(LFR, 0);
        analogWrite(RSV, 0);
        analogWrite(LSV, 0);
      }
      nh.spinOnce();
}
