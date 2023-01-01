#!/usr/bin/env python3
import json
import struct

import cv2
from cv2 import intersectConvexConvex
import numpy as np
import onnx
import onnxruntime as ort
import rospkg
from runonnx import run_inference
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from cv.msg import FloatArray, FloatList
from geometry_msgs.msg import Point
from cv_utils import camera_projection

class CVModelInferencer:
    def __init__(self):
        rospy.init_node('pothole_detection_model_inference')
        self.pub = rospy.Publisher('cv/pothole_detections', FloatArray, queue_size=10)

        self.bridge = CvBridge()
        self.projection = camera_projection.CameraProjection()
        
        rospack = rospkg.RosPack()
        model_path = f"{rospack.get_path('pothole_detection')}/models/best.onnx"

        self.model = onnx.load(model_path)
        onnx.checker.check_model(self.model)

        self.ort_session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

    def run(self):
        rospy.Subscriber("image", Image, self.process_image)
        rospy.spin()

    def process_image(self, data):
        if data == []:
            return

        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        if raw is not None:
            potholes = run_inference(raw, self.ort_session)
            # rospy.loginfo("potholes: %s", potholes)
            if potholes is None:
                return

            potholes = potholes[0]

            potholes = [potholes[i][:4] for i in range(len(potholes))]
            potholes = [[[int(p[0] + ((p[2] - p[0]) / 2)), int(p[1] + ((p[3] - p[1]) / 2))] for p in potholes]]
            # json_dumpable = {"pothole": potholes}

            # raw = cv2.resize(raw, (330, 180))
            # for circle in potholes[0]:
            #     raw = cv2.circle(raw, circle, 5, (0, 0, 255), -1)
            # cv2.imshow("im", raw)
            # cv2.waitKey(1)

            # Publish to /cv/pothole_detections
            lanes_msgs = []
            for lane in potholes:
                project_points = self.projection(np.array(lane, dtype=np.int))

                lane_msg = FloatList()
                pts_msg = []
                for pt in project_points:
                    pt_msg = Point()
                    pt_msg.x = pt[0]
                    pt_msg.y = pt[1]
                    pt_msg.z = pt[2]
                    pts_msg += [pt_msg]
                lane_msg.elements = pts_msg
                lanes_msgs += [lane_msg]
            rospy.loginfo("Publishes Something!")
            msg = FloatArray()
            msg.lists = lanes_msgs
            self.pub.publish(msg)

def get_copied_resized_input(frame: np.ndarray) -> np.ndarray:
    frame_copy = np.copy(frame)
    frame_copy = cv2.resize(frame_copy,(448,448))
    frame_copy = (frame_copy / 255.).transpose(2,0,1).reshape(1,3,448,448)
    return frame_copy


if __name__ == '__main__':
    wrapper = CVModelInferencer()
    wrapper.run()
