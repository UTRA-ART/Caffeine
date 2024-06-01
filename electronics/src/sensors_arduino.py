#!/usr/bin/env python3
'''
2024-05-23

read from arduino and control brk and en of motors

subscribes to:
- delay_restart
- enable_motors

publishes to:
- motors_stopped
- electronics_status
- electronics_status/details

params
- baud_rate
- ros_rate
- arduino_port
'''

import serial 

import rospy 
from std_msgs.msg import String 
from std_msgs.msg import Bool

arduino_port = rospy.get_param('/fault_monitor/arduino_port')

BAUD_RATE = rospy.get_param('/fault_monitor/baud_rate')
ROS_RATE = rospy.get_param('/fault_monitor/ros_rate', 10)

# messages
OK_MSG = "O"
TEMPERATURE_MSG = "T"
CURRENT_MSG = "C"
ALARM_MSG = "A"
USER_MSG = "U"

ERROR_MSGS = [TEMPERATURE_MSG, CURRENT_MSG, ALARM_MSG, USER_MSG]

# settings
restart_when_ok = True  # motors enabled
last_restart_msg = False 

# def restart_cb(restart_msg):
#     global restart_when_ok
#     if restart_msg.data == 'DELAY':
#         restart_when_ok = False 
#     elif restart_msg.data == 'OK':
#         restart_when_ok = True

def enable_cb(enable_msg):
    global restart_when_ok
    if enable_msg.data:
        restart_when_ok = True 
    else:
        restart_when_ok = False 

if __name__ == '__main__':
    # serial connection setup
    conn = serial.Serial(arduino_port, BAUD_RATE, timeout=1)
    conn.reset_input_buffer()

    # ros setup
    rospy.init_node("fault_monitor", anonymous=True)

    motors_pub = rospy.Publisher("/motors_stopped", Bool, queue_size=10)
    status_pub = rospy.Publisher("/electronics_status/msg", String, queue_size=10)
    details_pub = rospy.Publisher("/electronics_status/details", String, queue_size=10)

    rospy.Subscriber("/enable_motors", Bool, enable_cb)
    # rospy.Subscriber("delay_restart", String, restart_cb)

    rate = rospy.Rate(ROS_RATE)

    # run
    while not rospy.is_shutdown():
        if conn.in_waiting > 0:
            try:
                line = conn.readline().decode('utf-8').rstrip()
        
                if line == OK_MSG:
                    if restart_when_ok:
                        motors_pub.publish(False)
                        status_pub.publish(f"{OK_MSG} ON")
                    else:
                        motors_pub.publish(True)
                        status_pub.publish(f"{OK_MSG} OFF")
                elif line in ERROR_MSGS:
                    motors_pub.publish(True)
                    status_pub.publish(line)
                else:
                    details_pub.publish(line)

            except:
                pass
            
        if restart_when_ok != last_restart_msg:
            if restart_when_ok:
                conn.write(b'r')
            else:
                conn.write(b's')

            last_restart_msg = restart_when_ok