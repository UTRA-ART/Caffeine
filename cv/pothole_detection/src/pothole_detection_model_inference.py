#!/usr/bin/env python3
import json
import struct

import cv2
import numpy as np
import onnx
import onnxruntime as ort
import redis
import rospkg


class CVModelInferencer:
    def __init__(self):
        self.redis = redis.Redis(host='127.0.0.1', port=6379)
        
        rospack = rospkg.RosPack()
        model_path = f"{rospack.get_path('pothole_detection')}/models/best.onnx"
        self.model = onnx.load(model_path)
        onnx.checker.check_model(self.model)
        self.ort_session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])

    def run(self) -> None:
        raw = self._fromRedis("zed/preprocessed")
        if raw is not None:
            # PREPROCESS IMAGE
            img = get_copied_resized_input(raw)
            # RUN ONNX SESSION
            # Get inputs names with `[(i.name, i.shape, i.type) for i in self.ort_session.get_inputs()]`
            # Get inputs names with `[(o.name, o.shape, o.type) for o in self.ort_session.get_outputs()]`
            output = self.ort_session.run(None, {'images': img.astype(np.float32)})[0][0]
            if output is None:
                return
            # TODO(@Ammar-V) if you want to add non-potholes (e.g. ramps), do so here!
            # 0.5 is from CV-Pipeline's onnx/runonnx-numpy.py
            potholes = [[x, y] for x, y, _, _, confs, class_ in output if confs > 0.5 and class_ == 0]
            non_potholes = [[x, y] for x, y, _, _, confs, class_ in output if confs > 0.5 and class_ != 0]
            json_dumpable = {"pothole": potholes, "non_potholes": non_potholes}
            self._toRedis(json_dumpable, "pothole_detection")

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


def find_edge_channel(img):
    edges_mask = np.zeros((img.shape[0],img.shape[1]),dtype=np.uint8)
    width = img.shape[1]
    height = img.shape[0]

    gray_im = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # gray_im = cv2.GaussianBlur(gray_im,(3,3),0)
    # Separate into quadrants
    med1 = np.median(gray_im[:height//2,:width//2])
    med2 = np.median(gray_im[:height//2,width//2:])
    med3 = np.median(gray_im[height//2:,width//2:])
    med4 = np.median(gray_im[height//2:,:width//2])

    l1 = int(max(0,(1-0.205)*med1))
    u1 = int(min(255,(1+0.205)*med1))
    e1 = cv2.Canny(gray_im[:height//2,:width//2],l1,u1)

    l2 = int(max(0,(1-0.205)*med2))
    u2 = int(min(255,(1+0.205)*med2))
    e2 = cv2.Canny(gray_im[:height//2,width//2:],l2,u2)

    l3 = int(max(0,(1-0.205)*med3))
    u3 = int(min(255,(1+0.205)*med3))
    e3 = cv2.Canny(gray_im[height//2:,width//2:],l3,u3)

    l4 = int(max(0,(1-0.205)*med4))
    u4 = int(min(255,(1+0.205)*med4))
    e4 = cv2.Canny(gray_im[height//2:,:width//2],l4,u4)

    # Stitch the edges together
    edges_mask[:height//2,:width//2] = e1
    edges_mask[:height//2,width//2:] = e2
    edges_mask[height//2:,width//2:] = e3
    edges_mask[height//2:,:width//2] = e4

    edges_mask_inv = cv2.bitwise_not(edges_mask)

    return edges_mask, edges_mask_inv


def get_copied_resized_input(frame: np.ndarray) -> np.ndarray:
    frame_copy = np.copy(frame)
    frame_copy = cv2.resize(frame_copy,(448,448))
    frame_copy = (frame_copy / 255.).transpose(2,0,1).reshape(1,3,448,448)
    return frame_copy


if __name__ == '__main__':
    wrapper = CVModelInferencer()

    while True:
        wrapper.run()
