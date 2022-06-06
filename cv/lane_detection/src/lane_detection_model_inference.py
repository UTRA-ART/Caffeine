#!/usr/bin/env python3
import struct
import sys 
import time
import json
import os

import cv2
import numpy as np
import redis
# import onnx
# import onnxruntime as ort 
from datetime import datetime

import torch
import torch.nn as nn
import torch
from torch.nn import functional as F
import math
import cv2
import numpy as np
import random

class UNet(nn.Module):
    def __init__(self):
        super(UNet,self).__init__()

        self.max_pool = nn.MaxPool2d(kernel_size=2,stride=2)
        self.conv_1x1 = nn.Conv2d(in_channels=32,out_channels=1,kernel_size=1,stride=1)

        # Encoder side
        # First layer two convolution layers
        self.conv_down1 = self.double_conv(5,32,3)  # Change dependng on if it's colour or grayscale
        # Second layer two convolution layers
        self.conv_down2 = self.double_conv(32,64,3)
        # Third layer two convolution layers
        self.conv_down3 = self.double_conv(64,128,3)
        # Fourth layer two convolution layers
        self.conv_down4 = self.double_conv(128,256,3)
        # Fifth layer two convlution layers
        self.conv_down5 = self.double_conv(256,512,3)
        # Sixth layer two convolution layers
        self.conv_down6 = self.double_conv(512,1024,3)

        # Decoder Side
        # First decoder layer two convolution layers
        self.upsample1 = nn.ConvTranspose2d(in_channels=1024,out_channels=512,kernel_size=2,stride=2)
        self.conv_up1 = self.double_conv(1024,512,3)
        # Second decoder layer two convolution layers
        self.upsample2 = nn.ConvTranspose2d(in_channels=512,out_channels=256,kernel_size=2,stride=2)
        self.conv_up2 = self.double_conv(512,256,3)
        # Third decoder layer two convolution layers
        self.upsample3 = nn.ConvTranspose2d(in_channels=256,out_channels=128,kernel_size=2,stride=2)
        self.conv_up3 = self.double_conv(256,128,3)
        # Fourth decoder layer two convolution layers
        self.upsample4 = nn.ConvTranspose2d(in_channels=128,out_channels=64,kernel_size=2,stride=2)
        self.conv_up4 = self.double_conv(128,64,3)
        # Fifth decoder layer two convolution layers
        self.upsample5 = nn.ConvTranspose2d(in_channels=64,out_channels=32,kernel_size=2,stride=2)
        self.conv_up5 = self.double_conv(64,32,3)
        # Sixth decoder layer two convolution layers
        self.conv_up6 = self.double_conv(32,3,3)


    def forward(self,x): # Use F.interpolate on the final output to resize back to the original size
        # Encoder
        x1 = self.conv_down1(x)     # Output from first double conv
        x1_pool = self.max_pool(x1)
        x2 = self.conv_down2(x1_pool)     # Output from second double conv
        x2_pool = self.max_pool(x2)
        x3 = self.conv_down3(x2_pool)     # Output from third double conv
        x3_pool = self.max_pool(x3)
        x4 = self.conv_down4(x3_pool)     # Output from fourth double conv
        x4_pool = self.max_pool(x4)
        x5 = self.conv_down5(x4_pool)     # Output from fifth double conv
        x5_pool = self.max_pool(x5)
        x6 = self.conv_down6(x5_pool)     # Output from sixth double conv

        # print(x6.shape)
        # Decoder
        # Decoder first layer
        x7 = self.upsample1(x6)
        (_,_,H,W) = x7.shape
        x5_cropped = self.crop(x5,(H,W))
        x8 = self.conv_up1(torch.cat((x5_cropped,x7),dim=1))
        # Decoder second layer
        x9 = self.upsample2(x8)
        (_,_,H,W) = x9.shape
        x4_cropped = self.crop(x4,(H,W))
        x10 = self.conv_up2(torch.cat((x4_cropped,x9),dim=1))
        # Decoder third layer
        x11 = self.upsample3(x10)
        (_,_,H,W) = x11.shape
        x3_cropped = self.crop(x3,(H,W))
        x12 = self.conv_up3(torch.cat((x3_cropped,x11),dim=1))
        # Decoder fourth layer
        x13 = self.upsample4(x12)
        (_,_,H,W) = x13.shape
        x2_cropped = self.crop(x2,(H,W))
        x14 = self.conv_up4(torch.cat((x2_cropped,x13),dim=1))
        # Decoder fifth layer
        x15 = self.upsample5(x14)
        (_,_,H,W) = x15.shape
        x1_cropped = self.crop(x1,(H,W))
        x16 = self.conv_up5(torch.cat((x1_cropped,x15),dim=1))

        # x15 = self.conv_1x1(x14)
        x17 = self.conv_1x1(x16)

        # print(x17.shape)

        # x16 = F.interpolate(x15,(720,1280))
        x18 = F.interpolate(x17,(180,330))
        # return x16
        # return torch.sigmoid(x18)
        return x18

    def double_conv(self,in_c,out_c,k_size=3):
        conv_double = nn.Sequential(
            nn.Conv2d(in_c,out_c,k_size,1,1,bias=True),
            nn.BatchNorm2d(out_c),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_c,out_c,k_size,1,1,bias=True),
            nn.BatchNorm2d(out_c),
            nn.ReLU(inplace=True)
        )
        return conv_double

    def crop(self,image,target_size):
        width = target_size[1]
        height = target_size[0]
        (_,_,height_img,width_img) = image.shape

        delta_width = torch.div(width_img - width,2,rounding_mode="floor")
        delta_height = height_img - height

        if (width_img - 2*delta_width) > width:
            cropped_image = image[:,:,delta_height:height_img,delta_width:width_img-delta_width-1]
        elif (width_img - 2*delta_width) < width:
            cropped_image = image[:,:,delta_height:height_img,delta_width-1:width_img-delta_width]
        else:
            cropped_image = image[:,:,delta_height:height_img,delta_width:width_img-delta_width]

        return cropped_image

class UNet2(nn.Module):
    def __init__(self):
        super(UNet2,self).__init__()

        self.max_pool = nn.MaxPool2d(kernel_size=2,stride=2)
        self.conv_1x1 = nn.Conv2d(in_channels=32,out_channels=1,kernel_size=1,stride=1)

        # Encoder side
        # First layer two convolution layers
        self.conv_down1 = self.double_conv(1,32,3)
        # Second layer two convolution layers
        self.conv_down2 = self.double_conv(32,64,3)
        # Third layer two convolution layers
        self.conv_down3 = self.double_conv(64,128,3)

        # Decoder Side
        # First decoder layer two convolution layers
        self.upsample4 = nn.ConvTranspose2d(in_channels=128,out_channels=64,kernel_size=2,stride=2)
        self.conv_up4 = self.double_conv(128,64,3)
        # Secondd decoder layer two convolution layers
        self.upsample5 = nn.ConvTranspose2d(in_channels=64,out_channels=32,kernel_size=2,stride=2)
        self.conv_up5 = self.double_conv(64,32,3)

    def forward(self,x): # Use F.interpolate on the final output to resize back to the original size
        # Encoder
        x1 = self.conv_down1(x)     # Output from first double conv
        x1_pool = self.max_pool(x1)
        x2 = self.conv_down2(x1_pool)     # Output from second double conv
        x2_pool = self.max_pool(x2)
        x3 = self.conv_down3(x2_pool)     # Output from third double conv

        # Decoder
        # Decoder first layer
        x4 = self.upsample4(x3)
        (_,_,H,W) = x4.shape
        x2_cropped = self.crop(x2,(H,W))
        x5 = self.conv_up4(torch.cat((x2_cropped,x4),dim=1))
        # Decoder second layer
        x6 = self.upsample5(x5)
        (_,_,H,W) = x6.shape
        x1_cropped = self.crop(x1,(H,W))
        x7 = self.conv_up5(torch.cat((x1_cropped,x6),dim=1))

        x8 = self.conv_1x1(x7)
        x9 = F.interpolate(x8,(160,240))

        return x9

    def double_conv(self,in_c,out_c,k_size=3):
        conv_double = nn.Sequential(
            nn.Conv2d(in_c,out_c,k_size,1,1,bias=True),
            nn.BatchNorm2d(out_c),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_c,out_c,k_size,1,1,bias=True),
            nn.BatchNorm2d(out_c),
            nn.ReLU(inplace=True)
        )
        return conv_double

    def crop(self,image,target_size):
        width = target_size[1]
        height = target_size[0]
        (_,_,height_img,width_img) = image.shape

        delta_width = ((width_img - width)//2)
        delta_height = height_img - height

        if (width_img - 2*delta_width) > width:
            cropped_image = image[:,:,delta_height:height_img,delta_width:width_img-delta_width-1]
        elif (width_img - 2*delta_width) < width:
            cropped_image = image[:,:,delta_height:height_img,delta_width-1:width_img-delta_width]
        else:
            cropped_image = image[:,:,delta_height:height_img,delta_width:width_img-delta_width]

        return cropped_image

class UNet_Prob(nn.Module):
    def __init__(self):
        super(UNet,self).__init__()

        self.max_pool = nn.MaxPool2d(kernel_size=2,stride=2)
        self.conv_1x1 = nn.Conv2d(in_channels=32,out_channels=1,kernel_size=1,stride=1)

        # Encoder side
        # First layer two convolution layers
        self.conv_down1 = self.double_conv(5,32,3)  # Change dependng on if it's colour or grayscale
        # Second layer two convolution layers
        self.conv_down2 = self.double_conv(32,64,3)
        # Third layer two convolution layers
        self.conv_down3 = self.double_conv(64,128,3)
        # Fourth layer two convolution layers
        self.conv_down4 = self.double_conv(128,256,3)
        # Fifth layer two convlution layers
        self.conv_down5 = self.double_conv(256,512,3)
        # Sixth layer two convolution layers
        self.conv_down6 = self.double_conv(512,1024,3)

        # Decoder Side
        # First decoder layer two convolution layers
        self.upsample1 = nn.ConvTranspose2d(in_channels=1024,out_channels=512,kernel_size=2,stride=2)
        self.upsample1_p = nn.ConvTranspose2d(in_channels=1024,out_channels=1024,kernel_size=2,stride=2)
        self.conv_up1 = self.double_conv(1024,512,3)
        # Second decoder layer two convolution layers
        self.upsample2 = nn.ConvTranspose2d(in_channels=512,out_channels=256,kernel_size=2,stride=2)
        self.upsample2_p = nn.ConvTranspose2d(in_channels=512,out_channels=512,kernel_size=2,stride=2)
        self.conv_up2 = self.double_conv(512,256,3)
        # Third decoder layer two convolution layers
        self.upsample3 = nn.ConvTranspose2d(in_channels=256,out_channels=128,kernel_size=2,stride=2)
        self.upsample3_p = nn.ConvTranspose2d(in_channels=256,out_channels=256,kernel_size=2,stride=2)
        self.conv_up3 = self.double_conv(256,128,3)
        # Fourth decoder layer two convolution layers
        self.upsample4 = nn.ConvTranspose2d(in_channels=128,out_channels=64,kernel_size=2,stride=2)
        self.upsample4_p = nn.ConvTranspose2d(in_channels=128,out_channels=128,kernel_size=2,stride=2)
        self.conv_up4 = self.double_conv(128,64,3)
        # Fifth decoder layer two convolution layers
        self.upsample5 = nn.ConvTranspose2d(in_channels=64,out_channels=32,kernel_size=2,stride=2)
        self.upsample5_p = nn.ConvTranspose2d(in_channels=64,out_channels=64,kernel_size=2,stride=2)
        self.conv_up5 = self.double_conv(64,32,3)
        # Sixth decoder layer two convolution layers
        self.conv_up6 = self.double_conv(32,3,3)


    def forward(self,x,prob=0): # Use F.interpolate on the final output to resize back to the original size
        # Encoder
        x1 = self.conv_down1(x)     # Output from first double conv
        x1_pool = self.max_pool(x1)
        x2 = self.conv_down2(x1_pool)     # Output from second double conv
        x2_pool = self.max_pool(x2)
        x3 = self.conv_down3(x2_pool)     # Output from third double conv
        x3_pool = self.max_pool(x3)
        x4 = self.conv_down4(x3_pool)     # Output from fourth double conv
        x4_pool = self.max_pool(x4)
        x5 = self.conv_down5(x4_pool)     # Output from fifth double conv
        x5_pool = self.max_pool(x5)
        x6 = self.conv_down6(x5_pool)     # Output from sixth double conv

        # print(x6.shape)
        # Decoder
        # Decoder first layer
        if (random.random() < p):
            x7 = self.upsample1(x6)
            (_,_,H,W) = x7.shape
            x5_cropped = self.crop(x5,(H,W))
            x8 = self.conv_up1(torch.cat((x5_cropped,x7),dim=1))
        else:
            x7 = self.upsample1_p(x6)
            x8 = self.conv_up1(x7)
        # Decoder second layer
        if (random.random() < p):
            x9 = self.upsample2(x8)
            (_,_,H,W) = x9.shape
            x4_cropped = self.crop(x4,(H,W))
            x10 = self.conv_up2(torch.cat((x4_cropped,x9),dim=1))
        else:
            x9 = self.upsample2_p(x8)
            x10 = self.conv_up2(x9)
        # Decoder third layer
        if (random.random() < p):
            x11 = self.upsample3(x10)
            (_,_,H,W) = x11.shape
            x3_cropped = self.crop(x3,(H,W))
            x12 = self.conv_up3(torch.cat((x3_cropped,x11),dim=1))
        else:
            x11 = self.upsample3_p(x10)
            x12 = self.conv_up3(x11)
        # Decoder fourth layer
        if (random.random() < p):
            x13 = self.upsample4(x12)
            (_,_,H,W) = x13.shape
            x2_cropped = self.crop(x2,(H,W))
            x14 = self.conv_up4(torch.cat((x2_cropped,x13),dim=1))
        else:
            x13 = self.upsample4_p(x12)
            x14 = self.conv_up4(x13)
        # Decoder fifth layer
        if (random.random() < p):
            x15 = self.upsample5(x14)
            (_,_,H,W) = x15.shape
            x1_cropped = self.crop(x1,(H,W))
            x16 = self.conv_up5(torch.cat((x1_cropped,x15),dim=1))
        else:
            x15 = self.upsample5_p(x14)
            x16 = self.conv_up5(x15)

        # x15 = self.conv_1x1(x14)
        x17 = self.conv_1x1(x16)

        # x16 = F.interpolate(x15,(720,1280))
        x18 = F.interpolate(x17,(180,330))
        # return x16
        return x18

    def double_conv(self,in_c,out_c,k_size=3):
        conv_double = nn.Sequential(
            nn.Conv2d(in_c,out_c,k_size,1,1,bias=True),
            nn.BatchNorm2d(out_c),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_c,out_c,k_size,1,1,bias=True),
            nn.BatchNorm2d(out_c),
            nn.ReLU(inplace=True)
        )
        return conv_double

    def crop(self,image,target_size):
        width = target_size[1]
        height = target_size[0]
        (_,_,height_img,width_img) = image.shape

        delta_width = ((width_img - width)//2)
        delta_height = height_img - height

        if (width_img - 2*delta_width) > width:
            cropped_image = image[:,:,delta_height:height_img,delta_width:width_img-delta_width-1]
        elif (width_img - 2*delta_width) < width:
            cropped_image = image[:,:,delta_height:height_img,delta_width-1:width_img-delta_width]
        else:
            cropped_image = image[:,:,delta_height:height_img,delta_width:width_img-delta_width]

        return cropped_image

# import rospkg

from line_fitting import fit_lanes

class CVModelInferencer:
    def __init__(self):
        self.redis = redis.Redis(host='127.0.0.1', port=6379, db=3)
        
        # rospack = rospkg.RosPack()
        # model_path = rospack.get_path('lane_detection') + '/models/unet_with_sigmoid.onnx'
        model_path = "/home/art-jetson/Desktop/caffeine_ws/cv_models/lanes.onnx"

        # self.model = onnx.load(model_path)
        # onnx.checker.check_model(self.model)

        # print("Device:", ort.get_device())
        # self.ort_session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])
        
        self.unet = UNet()
        self.unet.load_state_dict(
            torch.load(
                "/home/art-jetson/Desktop/caffeine_ws/src/cv/lane_detection/src/unet_model_batch64_scheduled_lr0.5_epochs50.pt",
                map_location=torch.device("cuda"),
            )
        )
        self.unet.eval()
        self.unet.cuda()

        self.last_timestamp = str(datetime.utcnow())

    def _run(self):
        raw = self._fromRedisImg("zed/preprocessed")
        if raw is not None:
            img = get_input(raw.copy())
            
            # Do model inference 
            # output = self.ort_session.run(None, {'Inputs': img.astype(np.float32)})[0][0][0]
            
            output = torch.sigmoid(self.unet(torch.Tensor(img).cuda()))[0][0].detach().cpu().numpy()


            mask = np.where(output > 0.5, 1., 0.)
            # print(mask.shape)

            self._toRedisImg(mask, "cv/model/output")

            lanes = fit_lanes(mask)
            if lanes is not None:
                self._toRedisLanes(lanes, "lane_detection")
                self._toRedisLanesTimeStamp(str(datetime.utcnow()), 'lane_detection_timestamp')

            # toshow = np.concatenate([cv2.resize(raw, (330, 180)), np.tile(mask[...,np.newaxis]*255, (1, 1, 3))], axis=1).astype(np.uint8)
            # cv2.imshow("image", toshow)
            # cv2.waitKey(1)


    def run(self):

        ts = self._fromRedisImgTimeStamp('zed/preprocessed_timestamp')
        if self.last_timestamp == ts:
            return
        else:
            self.last_timestamp = ts
        self._run()

        
    def _toRedisLanes(self,lanes,name):
        """Store given Numpy array 'img' in Redis under key 'name'"""

        encoded = json.dumps(lanes)
        # Store encoded data in Redis
        self.redis.set(name,encoded)

    def _toRedisLanesTimeStamp(self, ts, name):
        self.redis.set(name, ts)

    def _fromRedisImg(self, name):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.redis.get(name)
        if encoded is None:
            return None
        h, w = struct.unpack('>II',encoded[:8])
        a = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8, offset=8), 1).reshape(h,w,3)
        return a

    def _toRedisImg(self,img,name):
        """Store given Numpy array 'img' in Redis under key 'name'"""
        h, w = img.shape[:2]
        shape = struct.pack('>II',h,w)

        retval, buffer = cv2.imencode('.png', img)
        img_bytes = np.array(buffer).tostring()

        encoded = shape + img_bytes

        # Store encoded data in Redis
        self.redis.set(name,encoded)

        return

    def _fromRedisImgTimeStamp(self, name):
        time = self.redis.get(name)
        
        if time is None:
            return None
        return time

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


def get_input(frame):
    #frame = cv2.resize(frame,(1280,720),interpolation=cv2.INTER_AREA)
    frame_copy = np.copy(frame)

    test_edges,test_edges_inv = find_edge_channel(frame_copy)
    frame_copy = np.append(frame_copy,test_edges.reshape(test_edges.shape[0],test_edges.shape[1],1),axis=2)
    frame_copy = np.append(frame_copy,test_edges_inv.reshape(test_edges_inv.shape[0],test_edges_inv.shape[1],1),axis=2)
    frame_copy = cv2.resize(frame_copy,(330,180))

    input = (frame_copy/255.).transpose(2,0,1).reshape(1,5,180,330)
    return input

def main():
    wrapper = CVModelInferencer()
    start_time = time.time()

    while True:
        wrapper.run()
        if time.time() - start_time > 30:
            break

if __name__ == '__main__':
    main()