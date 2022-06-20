#!/usr/bin/env python3
import json
import struct

import cv2
import numpy as np
import onnx
import onnxruntime as ort
import redis
import rospkg

from line_fitting import fit_lanes


WIDTH: int = 330
HEIGHT: int = 180
DEBUG_MODE: bool = True


class CVModelInferencer:
    def __init__(self):
        self.redis = redis.Redis(host="127.0.0.1", port=6379, db=3)

        rospack = rospkg.RosPack()
        model_path = (
            rospack.get_path("lane_detection") + "/models/unet_with_sigmoid.onnx"
        )

        self.model = onnx.load(model_path)
        onnx.checker.check_model(self.model)

        self.ort_session = ort.InferenceSession(
            model_path, providers=["CUDAExecutionProvider"]
        )

    def run(self):
        raw = self._fromRedisImg("zed/preprocessed")
        if raw is None:
            return
        img = get_input(raw)  # Does not return None

        # Do model inference
        outputs = self.ort_session.run(None, {"Inputs": img.astype(np.float32)})
        mask = np.where(outputs[0][0][0] > 0.5, 1.0, 0.0)
        self._toRedisImg(mask, "cv/model/output")

        lanes = fit_lanes(mask)
        if lanes is not None:
            self._toRedisLanes(lanes, "lane_detection")

        if DEBUG_MODE:
            toshow = np.concatenate([cv2.resize(raw, (330, 180)), np.tile(mask[...,np.newaxis]*255, (1, 1, 3))], axis=1).astype(np.uint8)
            cv2.imshow("image", toshow)
            cv2.waitKey(1)

    def _toRedisLanes(self, lanes, name):
        """Store given Numpy array 'img' in Redis under key 'name'"""

        encoded = json.dumps(lanes)
        # Store encoded data in Redis
        self.redis.set(name, encoded)

    def _fromRedisImg(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        if encoded is None:
            return None
        h, w = struct.unpack(">II", encoded[:8])
        a = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8, offset=8), 1).reshape(
            h, w, 3
        )
        return a

    def _toRedisImg(self, img, name):
        """Store given Numpy array 'img' in Redis under key 'name'"""
        h, w = img.shape[:2]
        shape = struct.pack(">II", h, w)

        retval, buffer = cv2.imencode(".png", img)
        img_bytes = np.array(buffer).tostring()

        encoded = shape + img_bytes

        # Store encoded data in Redis
        self.redis.set(name, encoded)

        return


def find_edge_channel(img):
    edges_mask = np.zeros(img.shape[0:2], dtype=np.uint8)
    width = img.shape[1]
    height = img.shape[0]

    gray_im = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # gray_im = cv2.GaussianBlur(gray_im,(3,3),0)
    # Separate into quadrants
    med1 = np.median(gray_im[: height // 2, : width // 2])
    med2 = np.median(gray_im[: height // 2, width // 2 :])
    med3 = np.median(gray_im[height // 2 :, width // 2 :])
    med4 = np.median(gray_im[height // 2 :, : width // 2])

    l1 = int(max(0, (1 - 0.205) * med1))
    u1 = int(min(255, (1 + 0.205) * med1))
    e1 = cv2.Canny(gray_im[: height // 2, : width // 2], l1, u1)

    l2 = int(max(0, (1 - 0.205) * med2))
    u2 = int(min(255, (1 + 0.205) * med2))
    e2 = cv2.Canny(gray_im[: height // 2, width // 2 :], l2, u2)

    l3 = int(max(0, (1 - 0.205) * med3))
    u3 = int(min(255, (1 + 0.205) * med3))
    e3 = cv2.Canny(gray_im[height // 2 :, width // 2 :], l3, u3)

    l4 = int(max(0, (1 - 0.205) * med4))
    u4 = int(min(255, (1 + 0.205) * med4))
    e4 = cv2.Canny(gray_im[height // 2 :, : width // 2], l4, u4)

    # Stitch the edges together
    edges_mask[: height // 2, : width // 2] = e1
    edges_mask[: height // 2, width // 2 :] = e2
    edges_mask[height // 2 :, width // 2 :] = e3
    edges_mask[height // 2 :, : width // 2] = e4

    edges_mask_inv = cv2.bitwise_not(edges_mask)

    return edges_mask, edges_mask_inv


def get_input(frame: np.ndarray) -> np.ndarray:
    frame_copy = np.copy(frame)
    frame_copy = cv2.resize(frame_copy, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)

    test_edges, test_edges_inv = find_edge_channel(frame_copy)
    frame_copy = np.block(
        [
            frame_copy,  # shape: (180, 330, 3)
            test_edges.reshape(*test_edges.shape, 1),  # shape: (180, 330, 1)
            test_edges_inv.reshape(*test_edges_inv.shape, 1),  # shape: (180, 330, 1)
        ]
    )  # (330, 180, 5)

    input = (frame_copy / 255.0).transpose(2, 0, 1).reshape(1, 5, *test_edges.shape)
    return input


if __name__ == "__main__":
    wrapper = CVModelInferencer()
    if DEBUG_MODE:
        while True:
            wrapper.run()
    else:
        while True:
            try:
                wrapper.run()
            except:
                pass
