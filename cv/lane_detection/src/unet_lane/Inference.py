from .UNet import UNet
import torch
import cv2
import numpy as np
import math
import os
import matplotlib.pyplot as plt
import time
import onnx
import onnxruntime as ort
# from UNetDataset import LaneDataset
import tqdm

# weights_path = r"C:\Users\ammar\Documents\CodingProjects\ART\CV-Pipeline\src\lane_detection\unet-lane\production\bgr+edge_unet_batch1_lr0.1_ep79.pt"

# weights_path = r""

class Inference():
    def __init__(self, weights_path = None, debug=False):
        self.model = UNet()
        self.model.load_state_dict(
                torch.load(weights_path,
                   map_location=torch.device("cuda"),
                )
        )
        self.model.eval()
        self.model.to(device="cuda")

        self.debug = debug

    def inference(self, frame, size=(640, 360)):
        frame = self.pre_process(frame)
        annotated, pred = self.predict_lanes(frame)

        if self.debug:
            annotated = cv2.resize(annotated, size)
            pred = cv2.resize(pred, size)
            return annotated, pred
        else:
            pred = cv2.resize(pred, size)
            return pred


    def predict_lanes(self, frame):
        # cv2.imwrite('gradient.png', frame[:, :, 3])
        frame = cv2.resize(frame, (256, 160))

        input = torch.Tensor((frame/255.0).transpose(2, 0, 1)).reshape(
            1, 4, 160, 256
        )
        input = input.to(device="cuda")
        
        output = self.model(input)[0][0]
        # output = torch.tensor(output)
        # output = output
        output = torch.sigmoid(output)
        output = output.detach().cpu().numpy()
        pred_mask = np.where(output > 0.5, 1, 0).astype("float32")
        
        if self.debug:
            bgr_frame = cv2.cvtColor(frame[:,:,:3], cv2.COLOR_HSV2BGR)
            overlayed_mask = np.copy(cv2.resize(bgr_frame, (256, 160)))
            overlayed_mask[np.where(pred_mask == 1)[0],
                        np.where(pred_mask == 1)[1], 2] = 255
            overlayed_mask[np.where(pred_mask == 1)[0],
                        np.where(pred_mask == 1)[1], 1] = 0
            overlayed_mask[np.where(pred_mask == 1)[0],
                        np.where(pred_mask == 1)[1], 0] = 0


            return overlayed_mask, pred_mask
                
        return None, pred_mask
    
    def pre_process(self, image=None):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges, edges_inv = self.find_edge_channel(image)
        gradient_map = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=-1)  # Gradient map along x
        #         gradient_map = cv2.Laplacian(gray, cv2.CV_64F)
        gradient_map = np.uint8(np.absolute(gradient_map))
        output_image = np.zeros((gray.shape[0], gray.shape[1], 4), dtype=np.uint8)
        output_image[:,:,0] = gray
        output_image[:,:,1] = edges
        output_image[:,:,2] = edges_inv
        output_image[:,:,3] = gradient_map

        return output_image
    
    def find_edge_channel(self, img):
        edges_mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
        width = img.shape[1]
        height = img.shape[0]

        gray_im = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Separate into quadrants
        med1 = np.median(gray_im[:height // 2, :width // 2])
        med2 = np.median(gray_im[:height // 2, width // 2:])
        med3 = np.median(gray_im[height // 2:, width // 2:])
        med4 = np.median(gray_im[height // 2:, :width // 2])

        l1 = int(max(0, (1 - 0.205) * med1))
        u1 = int(min(255, (1 + 0.205) * med1))
        e1 = cv2.Canny(gray_im[:height // 2, :width // 2], l1, u1)

        l2 = int(max(0, (1 - 0.205) * med2))
        u2 = int(min(255, (1 + 0.205) * med2))
        e2 = cv2.Canny(gray_im[:height // 2, width // 2:], l2, u2)

        l3 = int(max(0, (1 - 0.205) * med3))
        u3 = int(min(255, (1 + 0.205) * med3))
        e3 = cv2.Canny(gray_im[height // 2:, width // 2:], l3, u3)

        l4 = int(max(0, (1 - 0.205) * med4))
        u4 = int(min(255, (1 + 0.205) * med4))
        e4 = cv2.Canny(gray_im[height // 2:, :width // 2], l4, u4)

        # Stitch the edges together
        edges_mask[:height // 2, :width // 2] = e1
        edges_mask[:height // 2, width // 2:] = e2
        edges_mask[height // 2:, width // 2:] = e3
        edges_mask[height // 2:, :width // 2] = e4

        edges_mask_inv = cv2.bitwise_not(edges_mask)

        return edges_mask, edges_mask_inv



# Testing 
# image_path = r"C:\Users\ammar\Documents\CodingProjects\ART\CV-Pipeline\src\lane_detection\unet-lane\UNet-LaneDetection\input\additional-data\inputs"
# image_path = r"C:\Users\ammar\Documents\CodingProjects\ART\CV-Pipeline\src\lane_detection\unet-lane\UNet-LaneDetection\input\tusimple\inputs"

# image_path = r"C:\Users\ammar\Documents\CodingProjects\ART\CV-Pipeline\src\lane_detection\unet-lane\UNet-LaneDetection\input\comp\inputs"

# imagePaths = []
# fileNames = []
# fileSizes=[]
# for img in os.listdir(image_path):
#     img_path = os.path.join(image_path, img)

#     x = cv2.imread(img_path)
#     h, w, _ = x.shape
#     fileSizes.append((w, h))

#     imagePaths.append(img_path)
#     fileNames.append(img)
# test_dataset = LaneDataset(imagePaths, None)

# Output = Inference(None, False)
# for img_idx in tqdm.tqdm(range(test_dataset.__len__())):
#     frame, _, path = test_dataset.__getitem__(img_idx)
#     _, mask = Output.inference(frame, fileSizes[img_idx])

#     bgr_frame = cv2.imread(imagePaths[img_idx])
#     # print(bgr_frame.shape)
#     overlayed_mask = np.copy(cv2.resize(bgr_frame, fileSizes[img_idx]))
#     # print(mask.shape)
#     overlayed_mask[np.where(mask == 1)[0],
#                 np.where(mask == 1)[1], 2] = 255
#     overlayed_mask[np.where(mask == 1)[0],
#                 np.where(mask == 1)[1], 1] = 0
#     overlayed_mask[np.where(mask == 1)[0],
#                 np.where(mask == 1)[1], 0] = 0

    
#     cv2.imwrite(f'output/{fileNames[img_idx]}', mask * 255)
#     cv2.imshow("Predict", overlayed_mask)
#     if cv2.waitKey(0) == 'q':
#         cv2.destroyWindow("Predict")
