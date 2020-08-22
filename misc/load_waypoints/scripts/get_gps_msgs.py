#! /usr/bin/env python
 
import rospy
from sensor_msgs.msg import NavSatFix
import json
import std_msgs.msg

def callback(msg):
    global latitude 
    global longitude 
    latitude = msg.latitude
    longitude = msg.longitude


if __name__ == "__main__":
    rospy.init_node('gps_values')
    start_time = rospy.Time.now()

    gps_info = rospy.wait_for_message('gps_ready', std_msgs.msg.Bool)
    print(gps_info)
    while gps_info.data == False:
        continue

    print(rospy.Time.now() - start_time)

    sub = rospy.Subscriber('/caffeine/gps/fix', NavSatFix, callback)
    rospy.sleep(1)

    data = {"waypoints" : [
        {
            "id" : 0,
            "longitude" : longitude,
            "latitude" : latitude,
            "description"  : "Start point"
        }
    ]}

    base_dir = '/home/trudie/caffeine-ws/src/Caffeine/misc/load_waypoints'
    
    with open(base_dir + '/scripts/first_gps_coords.json', 'w') as outfile:
    # with open("sample_waypoints.json", "w") as outfile: 
        # outfile.write(json_waypoint) 
        print(data)
        json.dump(data, outfile) 
        print("success")
    # return [latitude, longitude]