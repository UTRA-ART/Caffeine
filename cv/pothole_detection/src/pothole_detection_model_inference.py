#!/usr/bin/env python3
import json
import struct

import cv2
from cv2 import intersectConvexConvex
import numpy as np
import onnx
import onnxruntime as ort
import redis
import rospkg
from runonnx import run_inference


class CVModelInferencer:
    def __init__(self):
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=3)
        
        rospack = rospkg.RosPack()
        model_path = f"{rospack.get_path('pothole_detection')}/models/best.onnx"
        self.model = onnx.load(model_path)
        onnx.checker.check_model(self.model)
        self.ort_session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])

    def run(self) -> None:
        raw = self._fromRedis("zed/preprocessed")

        if raw is not None:
            potholes = run_inference(raw, self.ort_session)
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

            self._toRedis(potholes, "pothole_detection")

    def _toRedis(self, lanes, name):
        """Store given Numpy array 'img' in Redis under key 'name'"""
        encoded = json.dumps(lanes)
        # Store encoded data in Redis
        self.redis.set(name,encoded)

    def _fromRedis(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        h, w = struct.unpack('>II',encoded[:8])
        a = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8, offset=8), 1).reshape(h,w,3)
        return a


def get_copied_resized_input(frame: np.ndarray) -> np.ndarray:
    frame_copy = np.copy(frame)
    frame_copy = cv2.resize(frame_copy,(448,448))
    frame_copy = (frame_copy / 255.).transpose(2,0,1).reshape(1,3,448,448)
    return frame_copy


if __name__ == '__main__':
    wrapper = CVModelInferencer()

    while True:
        wrapper.run()
