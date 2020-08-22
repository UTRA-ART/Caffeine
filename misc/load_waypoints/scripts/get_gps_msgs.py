#! /usr/bin/env python
 
import rospy
from sensor_msgs.msg import NavSatFix
import json

def callback(msg):
    # print((msg.latitude))
    # print((msg.longitude))
    global latitude 
    global longitude 
    latitude = msg.latitude
    longitude = msg.longitude
    # print(msg['latitude'])

if __name__ == "__main__":
    rospy.init_node('gps_values')
    sub = rospy.Subscriber('/caffeine/gps/fix', NavSatFix, callback)
    rospy.sleep(1)

    data = {"waypoints" : [
        {
            "id" : 0,
            "coordinate 1" : longitude,
            "coordinate 2" : latitude,
            "description"  : "Start point"
        }
    ]}
    # json_waypoint = json.dumps(data, indent=4)
    # base_dir = rospy.get_param('~arg_name') # ~ added to arg_name because private param 
    base_dir = '/home/trudie/caffeine-ws/src/Caffeine/misc/load_waypoints'
    
    with open(base_dir + '/scripts/sample_waypoints.json', 'w') as outfile:
    # with open("sample_waypoints.json", "w") as outfile: 
        # outfile.write(json_waypoint) 
        print(data)
        json.dump(data, outfile) 
        print("success")
    # return [latitude, longitude]