#!/usr/bin/env python3

'''
2024-05-11

read from arduino

subscribes to:
- left_wheel/command    (Float64)
- right_wheel/command

publishes to:
- left_wheel/ticks      (Int32)
- right_wheel/ticks

params
- baud_rate
- ros_rate
- arduino_port
'''

import serial

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool

arduino_port = rospy.get_param('/ticks_publisher/arduino_port')

BAUD_RATE = rospy.get_param('/ticks_publisher/baud_rate')
ROS_RATE = rospy.get_param('/ticks_publisher/ros_rate', 31)

direction_r = 1 
direction_l = 1 

def r_command_cb(control_msg):
    global direction_r 
    if control_msg.data >= 0:
        direction_r = True 
    else:
        direction_r = False 

def l_command_cb(control_msg):
    global direction_l 
    if control_msg.data >= 0:
        direction_l = False 
    else:
        direction_l = True 

if __name__ == '__main__':
    # serial connection setup
    conn = serial.Serial(arduino_port, BAUD_RATE, timeout=1)
    conn.reset_input_buffer()

    # ros setup
    rospy.init_node("ticks_publisher", anonymous=True)

    ticks_pub_r = rospy.Publisher("/right_wheel/ticks", Int32, queue_size=10)
    ticks_pub_l = rospy.Publisher("/left_wheel/ticks", Int32, queue_size=10)

    # rospy.Subscriber("right_wheel/command", Float64, r_command_cb)
    # rospy.Subscriber("left_wheel/command", Float64, l_command_cb)

    rate = rospy.Rate(ROS_RATE)

    l_val = 0
    r_val = 0

    # run
    while not rospy.is_shutdown():
        if conn.in_waiting > 0:          
            try:
                line = conn.readline().decode('utf-8').rstrip()
                l_val, r_val = line[1:-1].split(',')    # data in format <{left_count},{right_count}>

                ticks_pub_l.publish(direction_l * int(l_val))
                ticks_pub_r.publish(direction_r * int(r_val))
            except:
                pass