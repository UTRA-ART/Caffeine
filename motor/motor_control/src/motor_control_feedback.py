#!/usr/bin/env python2

# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# EXAMPLE SETUP
# Connect a button to pin 18 and GND, a pull-up resistor connecting the button
# to 3V3 and an LED connected to pin 12. The application performs the same
# function as the button_led.py but performs a blocking wait for the button
# press event instead of continuously checking the value of the pin in order to
# reduce CPU usage.

import RPi.GPIO as GPIO
import time
import numpy as np
import rospy 
from std_msgs.msg import Float64
from math import exp

# Pin Definitons:
LEFT_FEEDBACK_PIN = 16
RIGHT_FEEDBACK_PIN = 12  # NOTE: overlaps with audio GPIO

LOGIC_WINDOW_SIZE = 90
TIME_WINDOW_SIZE = 30
METERS_PER_TICK = 0.00443
ZERO_DELAY = 0.1

class MotorSpeedFeedbackNode:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
        GPIO.setup(LEFT_FEEDBACK_PIN, GPIO.IN)
        GPIO.setup(RIGHT_FEEDBACK_PIN, GPIO.IN)

        self.tic_left = time.time()
        self.tic_right = time.time()
        self.is_low_left = True
        self.is_low_right = True

        self.left_sign = -1
        self.right_sign = -1

        self.logic_end_left = LOGIC_WINDOW_SIZE
        self.logic_end_right = LOGIC_WINDOW_SIZE

        self.time_index_left = 0
        self.logic_index_left = 0
        self.time_index_right = 0
        self.logic_index_right = 0
        self.time_window_left = [0 for _ in range(TIME_WINDOW_SIZE)]
        self.logic_window_left = [0 for _ in range(LOGIC_WINDOW_SIZE)]
        self.time_window_right = [0 for _ in range(TIME_WINDOW_SIZE)]
        self.logic_window_right = [0 for _ in range(LOGIC_WINDOW_SIZE)]

        rospy.init_node("motor_feedback_node")
        rospy.Subscriber("/right_wheel/command", Float64, self.update_right)
        rospy.Subscriber("/left_wheel/command", Float64, self.update_left)

        self.pub_left = rospy.Publisher("/left_wheel/feedback", Float64, queue_size=1)
        self.pub_right = rospy.Publisher("/right_wheel/feedback", Float64, queue_size=1)

    def update_left(self, msg):
        self.left_sign = -1 if msg.data < 1 else 1
        new_window_size = self.map_signal_to_size(msg.data)
        self.logic_end_left = new_window_size
        if self.logic_index_left > new_window_size:
            self.logic_index_left = 0

    def update_right(self, msg):
        self.right_sign = -1 if msg.data < 1 else 1
        new_window_size = self.map_signal_to_size(msg.data)
        self.logic_end_right = new_window_size
        if self.logic_index_right > new_window_size:
            self.logic_index_right = 0

    def map_signal_to_size(self, x):
        # To reduce transitory noise effects on velocity measurement. Given a command velocity, returns the number 
        # of feedback readings 
        # ret = int(max(1, 20 - 4*x))
        # ret = int(max(1, (x / 0.00308)**-1.0593 / 0.0001426 / 3)) # x-axis period in s, y-axis is m/s
        # ret = int(max(1, 0.00992 * exp(-1.23*x) * 2337.541 * 3 + (x > 2.5) - 5*(x < 1))) # x-axis period in s, y-axis is m/s
        ret = int(max(1, (-0.00312*x + 0.00683) / 6 / 0.0001426)) # x-axis period in s, y-axis is m/s

        return ret

    def run(self):
        # Estimated time per loop: 0.0001426s / reading 
        # Read and publish left motor feedback 
        reading_left = GPIO.input(LEFT_FEEDBACK_PIN)
        self.logic_window_left[self.logic_index_left % self.logic_end_left] = reading_left 
        self.logic_index_left += 1

        if self.is_low_left and np.all(self.logic_window_left[:self.logic_end_left]):
            self.is_low_left = False
            self.time_window_left[self.time_index_left % TIME_WINDOW_SIZE] = time.time() - self.tic_left
            self.tic_left = time.time()
            self.time_index_left += 1
            self.pub_left.publish(METERS_PER_TICK * self.left_sign / np.mean(self.time_window_left))

        elif not reading_left:
            self.is_low_left = True
        if time.time() - self.tic_left > ZERO_DELAY:
            self.pub_left.publish(0)
            self.tic_left = time.time()

        # Read and publish right motor feedback 
        reading_right = GPIO.input(RIGHT_FEEDBACK_PIN)
        self.logic_window_right[self.logic_index_right % self.logic_end_right] = reading_right 
        self.logic_index_right += 1

        if self.is_low_right and np.all(self.logic_window_right[:self.logic_end_right]):
            self.is_low_right = False
            self.time_window_right[self.time_index_right % TIME_WINDOW_SIZE] = time.time() - self.tic_right
            self.tic_right = time.time()
            self.time_index_right += 1
            self.pub_right.publish(METERS_PER_TICK * self.right_sign / np.mean(self.time_window_right))

        elif not reading_right:
            self.is_low_right = True
        if time.time() - self.tic_right > ZERO_DELAY:
            self.pub_right.publish(0)
            self.tic_right = time.time()

    def spin(self):
        while (not rospy.is_shutdown()):
            self.run()
        GPIO.cleanup()

if __name__ == '__main__':
    node = MotorSpeedFeedbackNode()
    node.spin()
