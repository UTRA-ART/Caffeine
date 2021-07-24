#!/usr/bin/env python

#this script tests the distance between GPS fix and local/global odometry

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
import utm
import matplotlib.pyplot as plt
import time
import math

#publish new topic
pub = rospy.Publisher('deltaGPS', String, queue_size=10)

odometryToComp = 0
odometryToComp2 = 0

rospy.init_node('listener', anonymous=True)
tfListener = TransformListener()

start = time.time()

localPt = []
globalPt = []


def callback(data):
    global localPt
    global globalPt
    if(odometryToComp ==0 or odometryToComp2==0):
        print("return")
        return
    if(time.time()-start > 60*3):
        plot1 = plt.figure(1)
        plt.plot(globalPt)
        plt.ylabel('global')
        plot2 = plt.figure(2)
        plt.plot(localPt)
        plt.ylabel('local')
        plt.show()
    try:
        #compute the delta and give it to publisher
        p_in_frame = get_pose_from_gps(data.longitude, data.latitude, "odom").pose.position
        delta = "Local: deltaX= "+ str(p_in_frame.x-odometryToComp.x) + ", deltaY = " +str(p_in_frame.y-odometryToComp.y)+ ", deltaZ = " +str(p_in_frame.z-odometryToComp.z)
        localPt.append(math.sqrt((p_in_frame.x-odometryToComp.x) ** 2+(p_in_frame.y-odometryToComp.y) ** 2))
        print(delta)
        delta = "Global: deltaX= "+ str(p_in_frame.x-odometryToComp2.x) + ", deltaY = " +str(p_in_frame.y-odometryToComp2.y)+ ", deltaZ = " +str(p_in_frame.z-odometryToComp2.z)
        globalPt.append(math.sqrt((p_in_frame.x-odometryToComp2.x) ** 2+(p_in_frame.y-odometryToComp2.y) ** 2))
        print(delta)
    except rospy.ROSInterruptException:
        pass

def callback2(data):
    global odometryToComp
    odometryToComp = data.pose.pose.position

def callback3(data):
    global odometryToComp2
    odometryToComp2 = data.pose.pose.position

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.Subscriber("odometry/local", Odometry, callback2)
    rospy.Subscriber("odometry/global", Odometry, callback3)
    rospy.Subscriber("gps/fix", NavSatFix, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#converts gps coordinated to frame (odom,map,etc)
def get_pose_from_gps(longitude, latitude, frame, pose_test_var = None):
    utm_coords = utm.from_latlon(latitude, longitude)
    
    # create PoseStamped message to set up for do_transform_pose 
    utm_pose = PoseStamped()
    utm_pose.header.frame_id = 'utm'
    utm_pose.pose.position.x = utm_coords[0]
    utm_pose.pose.position.y = utm_coords[1]
    utm_pose.pose.orientation.w = 1.0 #make sure its right side up

    p_in_frame = tfListener.transformPose("/"+frame, utm_pose)

    return p_in_frame


# license removed for brevity
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
