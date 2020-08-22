#! /usr/bin/env python
 
import rospy
from sensor_msgs.msg import NavSatFix
import json
import std_msgs.msg
import rospkg, tf

def callback(msg):
    global latitude 
    global longitude 
    latitude = msg.latitude
    longitude = msg.longitude


if __name__ == "__main__":
    rospy.init_node('get_gps_coords')
    
    start_time = rospy.get_time()

    # Initialize transform listener 
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    rospy.sleep(1) # This prevents the waitfortransform to error out 
    
    # Wait for transform from /caffeine/map to /utm
    listener.waitForTransform("/caffeine/map", "/utm", rospy.Time(), rospy.Duration(50.0))

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(50.0))
            print(rospy.get_time() - start_time)
            break

        except (tf.LookupException, tf.ConnectivityException):
            continue

        rate.sleep()

    # Subscribe to /gps/fix topic to get the gps coords. 
    # This could be replaced with waitformessage ? 
    sub = rospy.Subscriber('/caffeine/gps/fix', NavSatFix, callback)
    
    rospy.sleep(1)

    data = {"waypoints" : [
        {
            "id" : 0,
            "longitude" : longitude,
            "latitude" : latitude,
            "description"  : "Rover start point"
        }
    ]}

    base_dir = rospkg.RosPack().get_path('load_waypoints')
    # Write the coords to a json 
    with open(base_dir + '/scripts/first_gps_coords.json', 'w') as outfile:
        print(data)
        json.dump(data, outfile) 
        print("Success")
