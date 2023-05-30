#!/usr/bin/env python3

import rospy
from cv.msg import FloatArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
# from pcl_msgs.msg import PointXYZ
from std_msgs.msg import Header


class FloatArrayToPointCloud2Node:
    def __init__(self):
        rospy.init_node("float_array_to_pc2_node")
        self.points2_pub = rospy.Publisher(
            "/cv/lane_detections_cloud", PointCloud2, queue_size=1
        )

    def run(self):
        self.float_array_sub = rospy.Subscriber(
            "/cv/lane_detections", FloatArray, self.float_array_callback, queue_size=10
        )

        rospy.spin()
    def float_array_callback(self, msg):
        points = []
        for float_list in msg.lists:
            for element in float_list.elements:
                point = [element.x, element.y, element.z]
                points.append(point)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          ]

        header = Header(frame_id='left_camera_link_optical')

        output_msg = point_cloud2.create_cloud(header, fields, points)
        self.points2_pub.publish( output_msg)


if __name__ == "__main__":
    float_array_to_pc2_node = FloatArrayToPointCloud2Node()
    float_array_to_pc2_node.run()
