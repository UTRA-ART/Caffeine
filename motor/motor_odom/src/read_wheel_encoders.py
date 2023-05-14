#!/usr/bin/env python3
'''
Code for reading from optical encoders using raspberry pi and publishing to ROS
2023-05-12

Subscribes to:
* none
Publishes to:
* right_wheel/ticks_ps
* left_wheel/ticks_ps

Interrupts tutorial: https://roboticsbackend.com/raspberry-pi-gpio-interrupts-tutorial/#Interrupts_with_add_event_detect_and_threaded_callback
'''
import RPi.GPIO as gpio
import time
import signal   # to handle CTRL+C
import sys 

import rospy 
from std_msgs.msg import Float64 

PRINT_RATE = 0.5
ROS_RATE = 60       # ros node cycles at 60 Hz

encoder_ra = 27     # (13)
encoder_rb = 17     # (11)
encoder_la = 24     # (18)
encoder_lb = 23     # (16)

phase_ra = 0
phase_rb = 0
phase_la = 0
phase_lb = 0 

direction_r = 1     # 1: CW, -1: CCW
direction_l = 1
pulse_count_r = 0
pulse_count_l = 0

def signal_handler(sig, frame):
    '''handles cleanup and exit when user presses CTRL+C'''
    gpio.cleanup()
    sys.exit(0)

def node_cleanup():
    rospy.loginfo("Shutting down wheel_encoder_node")
    gpio.cleanup()

def ra_state_change(channel):
    global pulse_count_r, phase_ra 

    pulse_count_r += 1
    phase_ra = int(not phase_ra)

def rb_state_change(channel):
    global phase_ra, phase_ra, direction_r 

    phase_rb = int(not phase_rb)
    if phase_ra == phase_rb:
        direction_r = 1
    else:
        direction_r = -1

def la_state_change(channel):
    global pulse_count_l, phase_la 

    pulse_count_l += 1
    phase_la = int(not phase_la)

def lb_state_change(channel):
    global phase_la, phase_la, direction_l
    
    phase_lb = int(not phase_lb)
    if phase_la == phase_lb:
        direction_l = 1
    else:
        direction_l = -1

if __name__ == '__main__':
    gpio.setmode(gpio.BCM)
    gpio.setup(encoder_ra, gpio.IN, pull_up_down=gpio.PUD_DOWN)    # set encoder pin as input with pull down resistor
    gpio.setup(encoder_rb, gpio.IN, pull_up_down=gpio.PUD_DOWN)
    gpio.setup(encoder_la, gpio.IN, pull_up_down=gpio.PUD_DOWN)
    gpio.setup(encoder_lb, gpio.IN, pull_up_down=gpio.PUD_DOWN)

    # attach interrupts
    gpio.add_event_detect(encoder_ra, gpio.BOTH, callback=ra_state_change)
    gpio.add_event_detect(encoder_rb, gpio.BOTH, callback=rb_state_change)
    gpio.add_event_detect(encoder_la, gpio.BOTH, callback=la_state_change) 
    gpio.add_event_detect(encoder_lb, gpio.BOTH, callback=lb_state_change)

    # handle CTRL+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # ros node setup
    rospy.init_node("wheel_encoder_node", anonymous=True)
    left_pub = rospy.Publisher("left_wheel/ticks_ps", Float64)
    right_pub = rospy.Publisher("right_wheel/ticks_ps", Float64)

    last_time = time.time() 

    rate = rospy.Rate(ROS_RATE)

    while not rospy.is_shutdown():
        current_time = time.time()
        time_diff = time.time() - last_time 
        if current_time - last_time > PRINT_RATE:
            print(f"ticks since last count: {pulse_count_r/2}\twheel direction: {direction_r}")

            l_ticks_ps = direction_l * pulse_count_l / (time.time() - last_time)
            r_ticks_ps = direction_r * pulse_count_r / (time.time() - last_time)

            left_pub.publish(l_ticks_ps)
            right_pub.publish(r_ticks_ps)
            
            last_time = time.time()
            pulse_count_r = 0
            pulse_count_l = 0 

        rate.sleep()

