#!/usr/bin/env python3
'''
Subscribes to velocity command topics for each wheel and converts to appropriate raspberry pi output
2023-05-07

To do:
* adjust RATE
* adjust TIMEOUT

Subscribes to:
* right_wheel/command
* left_wheel/command
* pause_navigation

Publishes to:
* right_wheel/direction
* left_wheel/direction
'''

import rospy 
from std_msgs.msg import Float64
from std_msgs.msg import Bool 
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import RPi.GPIO as gpio
import time 
from math import pi

# controls
ROS_RATE = 15
PWM_FREQ = 512 
TIMEOUT = 3000         # stop motors if TIMEOUT seconds have passed
rostime_last = 0
current_time = 0

# constants
WHEEL_RADIUS = 0.125    # update this with urdf
CIRCUMFERENCE = 2 * pi * WHEEL_RADIUS
WHEEL_BASE = 1          # update this with urdf


VEL_MAX = 2.2352        # 5 mph = 2.2352 m/s

# conversion: duty cycle = A * rpm + B
A_left = 0.448
B_left = -6.3637

A_left_small = 0.2016

def convert_speed_left(target):
    # convert to RPM
    rpm = target * 60 / CIRCUMFERENCE
    # convert to duty cycle
    dc = A_left * rpm + B_left
    return dc if dc > 0 else A_left_small * rpm

A_right = 0.4385
B_right = -5.9086

A_right_small = 0.2016

def convert_speed_right(target):
    # convert to RPM
    rpm = target * 60 / CIRCUMFERENCE
    # convert to duty cycle
    dc = A_right * rpm + B_right
    return dc if dc > 0 else A_right_small * rpm

# pins
R_DIR_PIN = 5       # (29)
R_SPEED_PIN = 12    # (32)
L_DIR_PIN = 6       # (31)
L_SPEED_PIN = 13    # (33)
LIGHT_PIN = 16      # (36)

# speed variables
g_vx = 0
g_wz = 0

# motor variables
right_speed = 0
right_dir = True      # True: CW, False: CCW
left_speed = 0
left_dir = True

right_dir_last = True 
left_dir_last = False

# light variables
BLINK_INTERVAL = 0.5
lighttime_last = 0
mode = False            # True: autonomous mode (light flashes)
                        # False: manual mode (light solid)
light_state = True

def target_cb(target_msg):
    global rostime_last, g_vx, g_wz
    
    rostime_last = time.time()

    g_vx = min(target_msg.linear.x, VEL_MAX)
    g_wz = target_msg.angular.z

def mode_cb(mode_msg):
    global mode

    mode = mode_msg.data

def node_cleanup():
    '''set all pins to 0, then call cleanup function'''
    gpio.output(R_DIR_PIN, gpio.LOW)
    gpio.output(L_DIR_PIN, gpio.LOW)
    r_speed_pin.ChangeDutyCycle(0)
    l_speed_pin.ChangeDutyCycle(0)
    
    gpio.output(LIGHT_PIN, gpio.LOW)

    gpio.cleanup()

if __name__ == '__main__':
    # pins
    print("Setting up pins...")
    gpio.setmode(gpio.BCM)
    # digital 1/0
    gpio.setup(R_DIR_PIN, gpio.OUT, initial=gpio.LOW)
    gpio.setup(L_DIR_PIN, gpio.OUT, initial=gpio.LOW)
    gpio.setup(LIGHT_PIN, gpio.OUT, initial=gpio.LOW)
    # digital PWM
    gpio.setup(R_SPEED_PIN, gpio.OUT)
    r_speed_pin = gpio.PWM(R_SPEED_PIN, PWM_FREQ)
    r_speed_pin.start(0)
    gpio.setup(L_SPEED_PIN, gpio.OUT)
    l_speed_pin = gpio.PWM(L_SPEED_PIN, PWM_FREQ)
    l_speed_pin.start(0)

    rospy.on_shutdown(node_cleanup)

    # subscribers
    print("Subscribing to topics...")
    rospy.init_node("motor_control_node", anonymous=True)

    rospy.Subscriber("/twist_mux/cmd_vel", Twist, target_cb)
    rospy.Subscriber("pause_navigation", Bool, mode_cb)

    # publishers
    right_dir_pub = rospy.Publisher("right_wheel/direction", Bool, queue_size=1)
    left_dir_pub = rospy.Publisher("left_wheel/direction", Bool, queue_size=1)
    
    debug_pub = rospy.Publisher("debug", String, queue_size=10)
    # debug_lights = rospy.Publisher("debug_lights", Bool, queue_size=1)

    rate = rospy.Rate(ROS_RATE)

    # run
    print("Running")
    while not rospy.is_shutdown():
        current_time = time.time() 

        # control motors
        if current_time - rostime_last >= TIMEOUT and not mode:
            # command has not been received in some time
            # something may be wrong
            # -> stop the motors
            gpio.output(R_DIR_PIN, gpio.LOW)
            gpio.output(L_DIR_PIN, gpio.LOW)
            r_speed_pin.ChangeDutyCycle(0)
            l_speed_pin.ChangeDutyCycle(0)
        else:
            # calculate speeds for each wheel
            # right speed
            vr = g_vx - (WHEEL_BASE * g_wz) / 2
            if vr == 0:
                right_duty_cycle = 0
            elif vr > 0:
                right_duty_cycle = convert_speed_right(vr)
                right_dir = True 
            else:
                right_duty_cycle = convert_speed_right(-vr)
                right_dir = False
            right_speed = min(max(right_duty_cycle, 0), VEL_MAX)

            # left speed
            vl = g_vx + (WHEEL_BASE * g_wz) / 2
            if vl == 0:
                left_duty_cycle = 0
            elif vl > 0:
                left_duty_cycle = convert_speed_left(vl)
                left_dir = False 
            else:
                left_duty_cycle = convert_speed_left(-vl)
                left_dir = True 
            left_speed = min(max(left_duty_cycle, 0), VEL_MAX)

            # write speed and direction pins
            gpio.output(R_DIR_PIN, right_dir)
            gpio.output(L_DIR_PIN, left_dir)

            test_msg = f'Right: {right_speed}, {right_dir}, Left: {left_speed}, {left_dir}'
            rospy.loginfo(test_msg)
            debug_pub.publish(test_msg)

            r_speed_pin.ChangeDutyCycle(right_speed)
            l_speed_pin.ChangeDutyCycle(left_speed)

            # publish direction
            # if right_dir != right_dir_last:
            #     right_dir_pub.publish(right_dir)
            #     right_dir_last = right_dir 
            #     rospy.loginfo(f"changing right_dir to  {right_dir}")
            # if left_dir != left_dir_last:
            #     left_dir_pub.publish(not left_dir)
            #     left_dir_last = left_dir 
            #     rospy.loginfo(f"changing left_dir to {left_dir}")
            right_dir_pub.publish(right_dir)
            left_dir_pub.publish(not left_dir)

        # control light
        if not mode:  
            # autonomous mode; flashing
            if current_time - lighttime_last >= BLINK_INTERVAL:
                lighttime_last = current_time
                light_state = not light_state

                gpio.output(LIGHT_PIN, light_state)
                
        else:
            # manual mode; solid
            gpio.output(LIGHT_PIN, True)

        # debug_lights.publish(light_mode)
        rate.sleep()
    

