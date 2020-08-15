#! /usr/bin/env python

from movebase_client import movebase_client, init_node
import rospy
import sys

if __name__ == '__main__':
    response = init_node(sys.argv)

    if response != 'Failure':
        try:
            x, y, z, roll, pitch, yaw, frame = response
	    if frame[0:9] == "caffeine/":
                frame = frame[9:]
            result = movebase_client(x, y, z, roll, pitch, yaw, f"caffeine/{frame}")
            rospy.loginfo(result)
        except:
            rospy.logerr("Error - Navigation failed for unknown reasons. Exiting...")
            rospy.signal_shutdown("Error - Navigation failed for unknown reasons. Exiting...")
