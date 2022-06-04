#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
w : increase forward speed by 0.1
a : increase leftward turn by 0.1rad/s 
s : increase backward speed by 0.1
d : increase rightward turn by 0.1rad/s 
p : give motor control to nav_stack (set to autonomous mode)

anything else : take control from nav_stack, stops rover 

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,1),
    }

delta = 0.1
speedBindings={
        'w':(delta,0),
        's':(-delta,0),
        'd':(0,-delta),
        'a':(0,delta),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('key_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.0)
    turn = rospy.get_param("~turn", 0.0)
    repeat = rospy.get_param("~repeat_rate", 0.001)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)
    autonomous_mode = False
    mode_pub = rospy.Publisher('/pause_navigation', Bool, queue_size = 1)

    x = 1
    y = 0
    z = 0
    th = 1
    status = 0
    top_vel = 2.2352 # m/s

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(key_timeout)
            if key != 'p' and key != '' and autonomous_mode:
                rospy.loginfo("Autonomous mode set to false. Teleop control is active.")
                autonomous_mode = False
            elif key == 'p':
                autonomous_mode = True
                rospy.wait_for_service('/move_base/clear_costmaps')
                clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                clear_costmaps()

                rospy.wait_for_service('/move_base/clear_unknown_space')
                clear_unknown_space = rospy.ServiceProxy('/move_base/clear_unknown_space', Empty)
                clear_unknown_space()

                rospy.loginfo("Autonomous mode set to true.")
                rospy.loginfo("Costmap cleared.")
                
            mode_pub.publish(not autonomous_mode)

            if key in speedBindings.keys(): 
                if speedBindings[key][1] != 0: # case: changing angular vel
                    turn = turn + speedBindings[key][1] # TODO: Write angular limits
                else: # case: changing linear vel 
                    speed = speed + speedBindings[key][0]
                    # if turn < 0:
                    #     speed = min(max(speed + speedBindings[key][0], (-2*top_vel - turn*0.89)/2), (2*top_vel + turn*0.89)/2) # mph 
                    # else:
                    #     speed = min(max(speed + speedBindings[key][0], (-2*top_vel + turn*0.89)/2), (2*top_vel - turn*0.89)/2) # mph 

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and autonomous_mode:
                    continue
                elif key != '':
                    x = 1
                    y = 0
                    z = 0
                    th = 1
                    speed = 0
                    turn = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
