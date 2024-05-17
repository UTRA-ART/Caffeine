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
'''

import rospy 
from std_msgs.msg import Float64
from std_msgs.msg import Bool 
from std_msgs.msg import String

import RPi.GPIO as gpio
import time 

# constants
def vel_convert(x):
    # x must be positive
    return max(48.5469 * 100 * x / 255 - 0.2489, 0)

# VEL_CONVERT = 51.0 * 100 / 255  # factor by which to multiply cmd_vel message before sending to motor controllers
#                                 # 51 because ??, 100 because rpi pwm messages are from 0-100, 255 because arduino pwm messages are from 0-255
VEL_MAX = 100       # maximum pwm duty cycle of raspberry pi is 100
RATE = 10          # rospy.Rate
PWM_FREQ = 512

# pins
R_DIR_PIN = 5       # (29)
R_SPEED_PIN = 12    # (32)
L_DIR_PIN = 6       # (31)
L_SPEED_PIN = 13    # (33)
LIGHT_PIN = 16      # (36)

# program control
TIMEOUT = 10    # stop motors if TIMEOUT s have passed
rostime_last = 0
current_time = 0

# motor variables
right_speed = 0
right_dir = True      # True: CW, False: CCW
left_speed = 0
left_dir = True

# light variables
BLINK_INTERVAL = 0.5
lighttime_last = 0
light_mode = True       # True: autonomous mode (light flashes)
                        # False: manual mode (light solid)
light_state = True


def rmotor_cb(control_msg):
    global rostime_last, right_speed, right_dir

    rostime_last = time.time() 

    input = control_msg.data 
    if input >= 0:
        right_speed = min(vel_convert(input), VEL_MAX)     # clip speed at maximum duty cycle
        right_dir = True
    else:
        right_speed = min(vel_convert(-input), VEL_MAX)
        right_dir = False

def lmotor_cb(control_msg):
    global rostime_last, left_speed, left_dir

    rostime_last = time.time() 

    input = control_msg.data 
    if input >= 0:
        left_speed = min(vel_convert(input), VEL_MAX)
        left_dir = False
    else:
        left_speed = min(vel_convert(-input), VEL_MAX)
        left_dir = True

def mode_cb(mode_msg):
    global light_mode

    light_mode = mode_msg.data

def node_cleanup():
    '''set all pins to 0, then call cleanup function'''
    gpio.output(R_DIR_PIN, gpio.LOW)
    gpio.output(L_DIR_PIN, gpio.LOW)
    r_speed_pin.ChangeDutyCycle(0)
    l_speed_pin.ChangeDutyCycle(0)
    
    gpio.output(LIGHT_PIN, gpio.LOW)

    gpio.cleanup()

if __name__ == '__main__':
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


    print("Subscribing to topics...")
    rospy.init_node("motor_control_node", anonymous=True)

    rospy.Subscriber("right_wheel/command", Float64, rmotor_cb)
    rospy.Subscriber("left_wheel/command", Float64, lmotor_cb)
    rospy.Subscriber("pause_navigation", Bool, mode_cb)
    
    debug_pub = rospy.Publisher("debug", String, queue_size=10)
    debug_lights = rospy.Publisher("debug_lights", Bool, queue_size=1)

    rate = rospy.Rate(RATE)

    print("Running")
    while not rospy.is_shutdown():
        current_time = time.time() 

        # control motors
        if current_time - rostime_last >= TIMEOUT:
            # command has not been received in some time
            # something may be wrong
            # -> stop the motors
            gpio.output(R_DIR_PIN, gpio.LOW)
            gpio.output(L_DIR_PIN, gpio.LOW)
            r_speed_pin.ChangeDutyCycle(0)
            l_speed_pin.ChangeDutyCycle(0)
        else:
            # write speed and direction poins as normal
            gpio.output(R_DIR_PIN, right_dir)
            gpio.output(L_DIR_PIN, left_dir)

            test_msg = f'Right: {right_speed}, {right_dir}, Left: {left_speed}, {left_dir}'
            rospy.loginfo(test_msg)
            debug_pub.publish(test_msg)

            r_speed_pin.ChangeDutyCycle(right_speed)
            l_speed_pin.ChangeDutyCycle(left_speed)

        # control light
        if not light_mode:  
            # autonomous mode; flashing
            if current_time - lighttime_last >= BLINK_INTERVAL:
                lighttime_last = current_time
                light_state = not light_state

                gpio.output(LIGHT_PIN, light_state)
                
        else:
            # manual mode; solid
            gpio.output(LIGHT_PIN, True)

        debug_lights.publish(light_mode)
        rate.sleep()
    

