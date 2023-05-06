#include <ros.h> 

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h> 
#include <std_msgs/Bool.h>

/* notes
 * 
 * motorcontrol from last year
 * fixed timeout ability
 * removed "nh.getHardware()->setBaud(BAUD_RATE);" from setup, which may have been the cause of issues
 * if(nh.connected()) may also be an issue
 * 
 * nh.spinOnce() occurs multiple times to prevent overflow
 * can prevent overflow further by using Float64 arrays, to reduce overhead
 * 
 * subscribed to: 
 *   /right_wheel/command
 *   /left_wheel/command
 *   /pause_navigation
 * publishes to:
 *   /right_wheel/ticks_ps (Float32)
 *   /left_wheel/ticks_ps  (Float32)
 * 
*/

const int BAUD_RATE = 57600;

const int RFR = 4;
const int RSV = 5;
const int LFR = 7;
const int LSV = 6;

const int LIGHT_PIN = 13;

// interrupt pins for encoders
const int REA = 2;
const int REB = 3;
const int LEA = 18;
const int LEB = 19;

// motor variables
float g_right_speed = 0;
int g_right_dir = 0;      // 1: CW, 0: CCW
float g_left_speed = 0;
int g_left_dir = 0;

unsigned long current_millis;

const long TIMEOUT = 10000;  // stop motors if TIMEOUT ms have passed
unsigned long g_last_ros = 0;
unsigned long g_current_ros = 0;

// light variables
const long INTERVAL = 1000;   // blink interval
unsigned long g_previous_millis = 0;  // last time light was updated
boolean g_mode = 1; // 1: autonomous mode (light flashes)
                // 0: manual mode (light solid)
int g_light_state = 1;

// encoder variables
// const int pulses_per_rot = 190;   // placeholder value - CHANGE THIS
// const int denoise_period = 4;     // can change?
const int PRINT_RATE = 500;   // calculate and publish every 500 ms
const int TICK_WINDOW = 1;   // average period over this many ticks
volatile int r_pulse_count = 0;
volatile int l_pulse_count = 0;
unsigned long last_millis = 0;
unsigned long time_diff = 0;
float r_vel = 0;    // speed of each wheel in m/s
float l_vel = 0;


// waveform A leads waveform B for clockwise rotation
int phase_ra = 0;
int phase_rb = 0;
int phase_la = 0;
int phase_lb = 0;
int r_direction = 1;  // 1: CW, -1: CCW
int l_direction = 1;

// test encoder
const int EN_TEST = 8;
unsigned long test_start = 0;
unsigned long test_end = 0;

ros::NodeHandle nh;

// get signed float speed command from ros, 
// convert to magnitude and direction for right motor
void rmotorCb(const std_msgs::Float64& control_msg){
  // input received, ros node ok
  g_last_ros = g_current_ros;
  g_current_ros = millis();

  float input = control_msg.data;
  if(input >= 0){
    g_right_speed = 51.0 * input;
    g_right_dir = 0;
  }else{
    g_right_speed = -51.0 * input;
    g_right_dir = 1;
  }
}

void lmotorCb(const std_msgs::Float64& control_msg){
  // input received, ros node ok
  g_last_ros = g_current_ros;
  g_current_ros = millis();

  float input = control_msg.data;
  if(input >= 0){
    g_left_speed = 51.0 * input;
    g_left_dir = 0;
  }else{
    g_left_speed = -51.0 * input;
    g_left_dir = 1;
  }
}

// get mode (autonomous or manual)
void setmode(const std_msgs::Bool& mode_msg){
  g_mode = mode_msg.data;
}

void ra_state_change(){
  r_pulse_count++;
  phase_ra = !phase_ra;  
}

void rb_state_change(){
  // a leads b for clockwise
  // if b goes up and a is already up -> clockwise
  phase_rb = !phase_rb;
  if(phase_ra == phase_rb) r_direction = 1;
  else r_direction = -1;
}

void la_state_change(){
  l_pulse_count++;
  phase_la = !phase_la;
}

void lb_state_change(){
  phase_lb = !phase_lb;
  if(phase_la == phase_lb) l_direction = 1;
  else l_direction = -1;
}

// subscribe to
ros::Subscriber<std_msgs::Float64> right_motor_vel_sub("/right_wheel/command", &rmotorCb);
ros::Subscriber<std_msgs::Float64> left_motor_vel_sub("/left_wheel/command", &lmotorCb);
ros::Subscriber<std_msgs::Bool> mode_sub("/pause_navigation", &setmode);

// publish to
// std_msgs::Int16 r_raw_msg;
// std_msgs::Int16 l_raw_msg;
// ros::Publisher right_encoder_raw_pub("/right_wheel/odom_raw", &r_raw_msg);
// ros::Publisher left_encoder_raw_pub("/left_wheel/odom_raw", &l_raw_msg);
std_msgs::Float32 r_vel_msg;
std_msgs::Float32 l_vel_msg;
ros::Publisher right_encoder_vel_pub("/right_wheel/ticks_ps", &r_vel_msg);
ros::Publisher left_encoder_vel_pub("/left_wheel/ticks_ps", &l_vel_msg);

void setup(){
  Serial.begin(BAUD_RATE);

  // ros node
  nh.initNode();

  nh.subscribe(right_motor_vel_sub);
  nh.subscribe(left_motor_vel_sub);
  nh.subscribe(mode_sub);

  nh.advertise(right_encoder_raw_pub);  
  nh.advertise(left_encoder_raw_pub);

  // set pins as output
  pinMode(RFR, OUTPUT);
  pinMode(RSV, OUTPUT);

  pinMode(LFR, OUTPUT);
  pinMode(LSV, OUTPUT);

  pinMode(LIGHT_PIN, OUTPUT);

  // attach interrupts
  pinMode(REA, INPUT);
  pinMode(REB, INPUT);
  attachInterrupt(digitalPinToInterrupt(REA), ra_state_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REB), rb_state_change, CHANGE);

  pinMode(LEA, INPUT);
  pinMode(LEB, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEA), la_state_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEB), lb_state_change, CHANGE);

  // test encoder code
  
}

void loop(){

  if(nh.connected()){
    current_millis = millis();
    
    // control motors
    if(current_millis - g_current_ros >= TIMEOUT){
      // command has not been received in some time, something may be wrong
      // - stop motors
      digitalWrite(RFR, 0);
      analogWrite(RSV, 0);
      digitalWrite(LFR, 0);
      analogWrite(LSV, 0);
    }else{
      // write speed and direction pins
      digitalWrite(RFR, g_right_dir);
      analogWrite(RSV, g_right_speed);
      digitalWrite(LFR, g_left_dir);
      analogWrite(LSV, g_left_speed);
    }

    nh.spinOnce();

    // control light
    if(g_mode){
      // autonomous mode, flashing 
      if(current_millis - g_previous_millis >= INTERVAL){
        g_previous_millis = current_millis;

        g_light_state = !g_light_state;
      }

      digitalWrite(LIGHT_PIN, g_light_state);
    }else{
      // manual mode, solid
      digitalWrite(LIGHT_PIN, 1);
    }
    nh.spinOnce();

    // publish encoder message
    noInterrupts();
    time_diff = millis() - last_millis;
    if(time_diff > PRINT_RATE){
        r_vel_msg.data = r_direction * r_pulse_count * 1000 / time_diff;  // ticks per second
        l_vel_msg.data = l_direction * l_pulse_count * 1000 / time_diff;

        right_encoder_vel_pub.publish(&r_vel_msg);
        left_encoder_vel_pub.publish(&l_vel_msg);

        r_pulse_count = 0;
        l_pulse_count = 0;
        last_millis = millis();
    }
    interrupts();
    // if(millis() - last_millis > PRINT_RATE){  
    //   // don't use current_millis because it may be out of date
    //   // divide by 2 because it counts both rising and falling edges
    //   // multiply by direction
    //   r_raw_msg.data = r_direction * r_pulse_count/2; 
    //   l_raw_msg.data = l_direction * l_pulse_count/2;
    //   right_encoder_raw_pub.publish(&r_raw_msg);
    //   left_encoder_raw_pub.publish(&l_raw_msg);

    //   last_millis = millis();
    //   r_pulse_count = 0;   
    //   l_pulse_count = 0;   
    // }

    nh.spinOnce();




  }else{
    // nh.connected == False: ros node has crashed - stop motors
    digitalWrite(RFR, 0);
    analogWrite(RSV, 0);
    digitalWrite(LFR, 0);
    analogWrite(LSV, 0);

    digitalWrite(LIGHT_PIN, 0);
  }

  nh.spinOnce();
}
