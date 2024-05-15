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

<<<<<<< HEAD
PRINT_RATE = 1
=======
PRINT_RATE = 0.5
>>>>>>> 8ba7006e7dd2a2d30d21b1cfa5612980676f5600
ROS_RATE = 60       # ros node cycles at 60 Hz

encoder_r = 22     # (15)
encoder_l = 23     # (16)

pulse_count_r = 0
pulse_count_l = 0

def signal_handler(sig, frame):
    '''handles cleanup and exit when user presses CTRL+C'''
    gpio.cleanup()
    sys.exit(0)

def node_cleanup():
    rospy.loginfo("Shutting down wheel_encoder_node")
    gpio.cleanup()

def r_state_change(channel):
    global pulse_count_r 

    pulse_count_r += 1

def l_state_change(channel):
    global pulse_count_l 

    pulse_count_l += 1

if __name__ == '__main__':
    gpio.setmode(gpio.BCM)
    gpio.setup(encoder_r, gpio.IN, pull_up_down=gpio.PUD_DOWN)    # set encoder pin as input with pull down resistor
    gpio.setup(encoder_l, gpio.IN, pull_up_down=gpio.PUD_DOWN)

    # attach interrupts
    gpio.add_event_detect(encoder_r, gpio.RISING, callback=r_state_change)
    gpio.add_event_detect(encoder_l, gpio.RISING, callback=l_state_change) 

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
<<<<<<< HEAD
            print(f"ticks since last count: {pulse_count_r} ")
=======
            print(f"ticks since last count: {pulse_count_r}")
>>>>>>> 8ba7006e7dd2a2d30d21b1cfa5612980676f5600

            l_ticks_ps = pulse_count_l / (time.time() - last_time)
            r_ticks_ps = pulse_count_r / (time.time() - last_time)

            left_pub.publish(l_ticks_ps)
            right_pub.publish(r_ticks_ps)
            
            last_time = time.time()
            pulse_count_r = 0
            pulse_count_l = 0 

<<<<<<< HEAD
        rate.sleep()
=======
        rate.sleep()
>>>>>>> 8ba7006e7dd2a2d30d21b1cfa5612980676f5600
