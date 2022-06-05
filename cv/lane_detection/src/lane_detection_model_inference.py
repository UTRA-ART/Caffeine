#!/usr/bin/env python3
import struct
import sys
import time
import json
import os

import cv2
import numpy as np
import redis
import onnx
import onnxruntime as ort

# import rospkg

from line_fitting import fit_lanes
from datetime import datetime


class CVModelInferencer:
    def __init__(self):
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=3)

        # rospack = rospkg.RosPack()
        # model_path = rospack.get_path('lane_detection') + '/models/unet_with_sigmoid.onnx'
        model_path = "/home/art-jetson/Desktop/caffeine_ws/cv_models/lanes.onnx"

        self.model = onnx.load(model_path)
        onnx.checker.check_model(self.model)

        self.ort_session = ort.InferenceSession(
            model_path, providers=['CUDAExecutionProvider'])

        self.last_timestamp = str(datetime.utcnow())

    def run(self):
        ts = self._fromRedisImgTimeStamp("zed/preprocessed_timestamp")

        if self.last_timestamp == ts:
            return
        else:
            self.last_timestamp = ts

        raw = self._fromRedisImg("zed/preprocessed")
        if raw is not None:
            img = get_input(raw.copy())

            # Do model inference
            output = self.ort_session.run(
                None, {'Inputs': img.astype(np.float32)})[0][0][0]
            mask = np.where(output > 0.5, 1., 0.)
            self._toRedisImg(mask, "cv/model/output")

            lanes = fit_lanes(mask)
            if lanes is not None:
                self._toRedisLanes(lanes, "lane_detection")
                self._toRedisLanesTimeStamp(
                    str(datetime.utcnow()), "lane_detection_timestamp")

            # toshow = np.concatenate([cv2.resize(raw, (330, 180)), np.tile(mask[...,np.newaxis]*255, (1, 1, 3))], axis=1).astype(np.uint8)
            # cv2.imshow("image", toshow)
            # cv2.waitKey(1)

    def _toRedisLanes(self, lanes, name):
        """Store given Numpy array 'img' in Redis under key 'name'"""

        encoded = json.dumps(lanes)
        # Store encoded data in Redis
        self.redis.set(name, encoded)

    def _toRedisLanesTimeStamp(self, ts, name):
        self.redis.set(name, ts)

    def _fromRedisImg(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        if encoded is None:
            return None
        h, w = struct.unpack('>II', encoded[:8])
        a = cv2.imdecode(np.frombuffer(
            encoded, dtype=np.uint8, offset=8), 1).reshape(h, w, 3)
        return a

    def _fromRedisImgTimeStamp(self, name):
        time = self.redis.get(name)
        if time is None:
            return None
        return time

    def _toRedisImg(self, img, name):
        """Store given Numpy array 'img' in Redis under key 'name'"""
        h, w = img.shape[:2]
        shape = struct.pack('>II', h, w)

        retval, buffer = cv2.imencode('.png', img)
        img_bytes = np.array(buffer).tostring()

        encoded = shape + img_bytes

        # Store encoded data in Redis
        self.redis.set(name, encoded)

        return


def find_edge_channel(img):
    edges_mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
    width = img.shape[1]
    height = img.shape[0]

    gray_im = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray_im = cv2.GaussianBlur(gray_im,(3,3),0)
    # Separate into quadrants
    med1 = np.median(gray_im[:height//2, :width//2])
    med2 = np.median(gray_im[:height//2, width//2:])
    med3 = np.median(gray_im[height//2:, width//2:])
    med4 = np.median(gray_im[height//2:, :width//2])

    l1 = int(max(0, (1-0.205)*med1))
    u1 = int(min(255, (1+0.205)*med1))
    e1 = cv2.Canny(gray_im[:height//2, :width//2], l1, u1)

    l2 = int(max(0, (1-0.205)*med2))
    u2 = int(min(255, (1+0.205)*med2))
    e2 = cv2.Canny(gray_im[:height//2, width//2:], l2, u2)

    l3 = int(max(0, (1-0.205)*med3))
    u3 = int(min(255, (1+0.205)*med3))
    e3 = cv2.Canny(gray_im[height//2:, width//2:], l3, u3)

    l4 = int(max(0, (1-0.205)*med4))
    u4 = int(min(255, (1+0.205)*med4))
    e4 = cv2.Canny(gray_im[height//2:, :width//2], l4, u4)

    # Stitch the edges together
    edges_mask[:height//2, :width//2] = e1
    edges_mask[:height//2, width//2:] = e2
    edges_mask[height//2:, width//2:] = e3
    edges_mask[height//2:, :width//2] = e4

    edges_mask_inv = cv2.bitwise_not(edges_mask)

    return edges_mask, edges_mask_inv


def get_input(frame):
    #frame = cv2.resize(frame,(1280,720),interpolation=cv2.INTER_AREA)
    frame_copy = np.copy(frame)

    test_edges, test_edges_inv = find_edge_channel(frame_copy)
    frame_copy = np.append(frame_copy, test_edges.reshape(
        test_edges.shape[0], test_edges.shape[1], 1), axis=2)
    frame_copy = np.append(frame_copy, test_edges_inv.reshape(
        test_edges_inv.shape[0], test_edges_inv.shape[1], 1), axis=2)
    frame_copy = cv2.resize(frame_copy, (330, 180))

    input = (frame_copy/255.).transpose(2, 0, 1).reshape(1, 5, 180, 330)
    return input


if __name__ == '__main__':
    wrapper = CVModelInferencer()

    while True:
        wrapper.run()
