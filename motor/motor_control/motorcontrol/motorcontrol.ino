#include <ros.h>
#include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>

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
    ros_count = currentMillis - ros_count;
    
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

    //reads motor speed feedback and publishes it to ros
//    float Lspeedread = analogRead(LSPEED);
//    float Rspeedread = analogRead(RSPEED);
//    speed_feedback.data_length = 2;
//    
//    speed_feedback.data[0] = Lspeedread;
//    speed_feedback.data[1] = Rspeedread;

    
    // control light
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
    delay(1);
}
