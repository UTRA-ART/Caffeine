import json
import cv2 
import numpy as np
import rospkg

rospack = rospkg.RosPack()
save_path = rospack.get_path('lane_detection') + '/config/depth_vals.json'

vals = json.load(open(save_path, 'r'))

img = np.zeros((180, 330, 1))

for y in range(img.shape[0]):
    for x in range(img.shape[1]):
        img[y, x] = vals[str((x, y))]

max = np.max(img)
print(max)
scale = 255 / max

img *= scale

cv2.imshow("image", img.astype(np.uint8))
cv2.waitKey(0)