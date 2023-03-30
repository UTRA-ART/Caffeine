import cv2
import numpy as np

import rospkg
import os
import time


def iou(img1, img2):
    inter = cv2.bitwise_and(img1, img2)
    union = cv2.bitwise_or(img1, img2)
    out = cv2.countNonZero(inter)/cv2.countNonZero(union)
    return out

def check_ellipse(cnt, thres = 0.75, height=297, width=396):
    img_cnt = np.zeros((height, width, 3), np.uint8)
    cv2.drawContours(image=img_cnt, contours=[cnt], contourIdx=-1, color=(255,255,255), thickness=-1)
    img_cnt = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
    
    ellp = cv2.fitEllipse(cnt)
    img_fit = np.zeros((height, width), np.uint8)
    cv2.ellipse(img_fit, ellp, 255, -1)
    
    inter = cv2.bitwise_and(img_cnt, img_fit)
    union = cv2.bitwise_or(img_cnt, img_fit)
    iou = cv2.countNonZero(inter)/(cv2.countNonZero(union)+1)
    return iou > thres

def hist2list(hist):
    cut = np.diff(hist.reshape(-1)) > 0
    thres = np.where(np.concatenate(([cut[0]], cut[:-1] != cut[1:])))[0][::2]
    if thres[-1] != 31:
        thres = np.concatenate( (thres, np.array([31,])) )
    thres = thres*8
    thres = zip( thres, np.concatenate((thres[1:], np.array([255,]))) )
    return list(thres)

def hist2thres(hist, cut_thres=5000):
    cut = hist.reshape(-1) > cut_thres
    ones = np.where(np.concatenate(([cut[0]], cut[:-1] != cut[1:])))[0]
    n = len(ones)
    if n == 0:
        if cut_thres > 500:
            return hist2thres(hist, cut_thres/2)
        else:
            return -1
    if n == 1:
        return -1
    if n == 2 or hist[ones[-2]:ones[-1]].sum()>3.2*cut_thres:
        thres = ones[-1]
        while(thres < 32 and hist[thres-1] > hist[thres]): 
            thres+=1
        return min(31, thres+1)
    if n%2 == 1:
        thres = ones[-1]
    else:
        thres = ones[-2]  
    while(thres > 16 and hist[thres-1] < hist[thres]): 
        thres-=1
    return thres

def multi_thres(image, scale=0.1, min_w=45, min_h=2):
    width = int(image.shape[1] * scale)
    height = int(image.shape[0] * scale)
    size = (width, height)

    resized = cv2.resize(image, size, interpolation = cv2.INTER_AREA)
    masked = np.zeros((height, width, 3), np.uint8)

    y, u, vv = cv2.split(cv2.cvtColor(resized, cv2.COLOR_BGR2YUV))
    
    pink = cv2.cvtColor(cv2.merge([y, u, cv2.equalizeHist(vv)]), cv2.COLOR_YUV2BGR)
    b = cv2.split(pink)[0]   
    blue_hist = cv2.calcHist([pink], [0], None, [32], [0,256])
    thres = hist2list(blue_hist)
    
    boxes = [] # boxes in x, y, w, h as ratios
    
    for th1, th2 in thres:
        ret, mask = cv2.threshold(b, th1, th2, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, 1)
        for cnt in contours:
            box = cv2.boundingRect(cnt)
            if box[2] < min_w or box[3] < min_h: continue
            if box[2] > 250 or box[3] > 100: continue
            #approx = cv2.approxPolyDP(cnt, .02 * cv2.arcLength(cnt, True), True)
            #if len(approx) < 7: continue
            if not check_ellipse(cnt): continue
            cv2.drawContours(image=masked, contours=[cnt], contourIdx=-1, color=(255,255,255), thickness=-1)
            box = [ box[0]/width, box[1]/height, box[2]/width, box[3]/height ]
            boxes.append(box)
            

    thres = hist2thres(blue_hist)*8
    ret, mask = cv2.threshold(b, thres, 255, cv2.THRESH_BINARY)
    
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, 1)
    for cnt in contours:
        box = cv2.boundingRect(cnt)
        if box[2] < min_w or box[3] < min_h: continue
        if box[2] > 250 or box[3] > 100: continue
        #approx = cv2.approxPolyDP(cnt, .02 * cv2.arcLength(cnt, True), True)
        #if len(approx) < 7: continue
        if not check_ellipse(cnt): continue
        cv2.drawContours(image=masked, contours=[cnt], contourIdx=-1, color=(255,255,255), thickness=-1)
        box = [ box[0]/width, box[1]/height, box[2]/width, box[3]/height ]
        boxes.append(box)
        
    masked = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)       
    
    return masked, boxes




