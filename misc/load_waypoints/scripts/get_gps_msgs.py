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
    rospy.init_node('gps_values')
    # now = rospy.Time.now()
    start_time = rospy.get_time()

    # start_time = rospy.get_time()

    listener = tf.TransformListener()

    # gps_ready = rospy.Publisher('gps_ready', std_msgs.msg.Bool, queue_size=1)

    rate = rospy.Rate(10.0)

    available = False 
    rospy.sleep(1)
    listener.waitForTransform("/caffeine/map", "/utm", rospy.Time(), rospy.Duration(50.0))

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/caffeine/map", "/utm", now, rospy.Duration(50.0))
            available = True 
            print(rospy.get_time() - start_time)
            break

        except (tf.LookupException, tf.ConnectivityException):
            continue

        # gps_ready.publish(available)

        rate.sleep()

    # gps_info = rospy.wait_for_message('gps_ready', std_msgs.msg.Bool)

    # while True:
    #     # rospy.loginfo(gps_info, 'gps')
    #     if gps_info.data == False:
    #         continue
    #     elif gps_info.data == True:
    #         break
    #     gps_info = rospy.wait_for_message('gps_ready', std_msgs.msg.Bool)

    # print(rospy.get_time() - start_time)
    # print(rospy.Time() - start_time)

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

    # base_dir = '/home/trudie/caffeine-ws/src/Caffeine/misc/load_waypoints'
    base_dir = rospkg.RosPack().get_path('load_waypoints')

    with open(base_dir + '/scripts/first_gps_coords.json', 'w') as outfile:
        print(data)
        json.dump(data, outfile) 
        print("Success")
    # return [latitude, longitude]