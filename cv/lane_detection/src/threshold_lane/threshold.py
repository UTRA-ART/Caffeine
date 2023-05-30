import os
import cv2
import time
# from tqdm import tqdm
import numpy as np

INPUT_DIR = "C:\\Users\\16474\\Desktop\\lanes-utra\\lanes-big-in"
OUTPUT_DIR = "C:\\Users\\16474\\Desktop\\lanes-utra\\threshold-out"


def create_mask(img_hsv):
    '''Create mask for white pixels using HSV color scheme'''
    sensitivity = 35
    lower_white = np.array([0,0,255-sensitivity])
    upper_white = np.array([255,sensitivity,255])
    mask = cv2.inRange(img_hsv, lower_white, upper_white)

    return mask

def create_orange_mask(img_hsv):
    '''Create mask for orange pixels using HSV color scheme'''
    lower_orange = np.array([10, 100, 100])  # Lower HSV values for orange color
    upper_orange = np.array([50, 255, 255])  # Upper HSV values for orange color
    mask = cv2.inRange(img_hsv, lower_orange, upper_orange)

    return mask

def get_output_img(img, mask):
    '''Convert masked image to output format'''
    output_img = np.zeros((img.shape[0], img.shape[1]), np.uint8)
    output_img[np.where(mask == 0)] = 0
    output_img[np.where(mask > 0)] = 255

    return output_img

def get_mask(img, mask_function=create_mask):

    # resize image
    scale_percent = 25
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    # create white mask
    mask = mask_function(resized)

    # generate black and white output image
    output_img = get_output_img(resized, mask)

    # resize image
    scale_percent = 400
    width = int(output_img.shape[1] * scale_percent / 100)
    height = int(output_img.shape[0] * scale_percent / 100)
    dim = (width, height)
    output_img = cv2.resize(output_img, dim, interpolation = cv2.INTER_AREA)

    return output_img

def clean_barrels (img):
    if img.max() > 0:

        # Expand img
        m_up = np.float64([
            [1, 0, 0],
            [0, 1, 30],
        ])

        m_down = np.float64([
            [1, 0, 0],
            [0, 1, -30],
        ])

        up = cv2.warpAffine(img, m_up, (img.shape[1], img.shape[0]))     
        down = cv2.warpAffine(img, m_down, (img.shape[1], img.shape[0]))

        out = img + up + down

        return out
    
    else:
        return img

def rm_barrel(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    mask = get_mask(hsv, create_orange_mask)

    if mask.max() > 0:
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            epsilon  = 0.1*cv2.arcLength(c,True)
            approx = cv2.approxPolyDP(c, epsilon , True)
            if len(approx) <4:
                continue
            p0,p1,p2,p3 = approx[0][0], approx[1][0],approx[2][0], approx[3][0]
            pts = np.array([
                    [p1, [p0[0]+int((p0[0]-p1[0])/(p1[1]-p0[1])*p0[1]),0],
                    [p3[0]+int((p3[0]-p2[0])/(p2[1]-p3[1])*p3[1]),0], p2]
                ])
            cv2.fillPoly(img, pts, color=(0,0,0))
            cv2.drawContours(img, pts, -1, color=(0, 0, 0), thickness=10)

    return img

def lane_detection(img):
    # convert from bgr to hsv
    img = cv2.resize(img, (640, 360))
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


    # no_barrels = rm_barrel(img)
    lanes = get_mask(img_hsv, create_mask)
    lanes = np.clip(lanes, 0, 1)

    barrels = get_mask(img_hsv, create_orange_mask)
    barrels = clean_barrels(barrels)

    output_img = cv2.subtract(lanes, barrels)
    output_img = np.clip(output_img, 0, 1) * 255


    # contours, hierarchy = cv2.findContours(output_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # # Plot the areas and the location of blobs
    # areas = [cv2.contourArea(cnt) for cnt in contours]

    # # Get all the contours that are greater than a certain area
    # blobs = []
    # for i in range(0, len(areas)):
    #     area = areas[i]
        
    #     if area > 0: # PARAM
    #         blobs.append(contours[i])

    # print(len(blobs))
    # out = np.zeros((img.shape[1], img.shape[0])).astype(np.uint8)
    # cv2.drawContours(out, blobs, -1, (255, 255, 255), -1)
    



    cv2.imwrite(r'/home/ammarvora/utra/caffeine-ws/src/Caffeine/cv/lane_detection/barrels.png', lanes * 255)
    return output_img


# def main():
#     ts = []

#     # iterate through input files
#     for filename in tqdm(os.listdir(INPUT_DIR)):
#         f = os.path.join(INPUT_DIR, filename)
#         img = cv2.imread(f)

#         t1 = time.time()

#         # convert from bgr to hsv
#         img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#         # resize image
#         scale_percent = 25
#         width = int(img_hsv.shape[1] * scale_percent / 100)
#         height = int(img_hsv.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         resized = cv2.resize(img_hsv, dim, interpolation = cv2.INTER_AREA)

#         # create white mask
#         mask = create_mask(resized)

#         # generate black and white output image
#         output_img = get_output_img(resized, mask)

#         # resize image
#         scale_percent = 400
#         width = int(output_img.shape[1] * scale_percent / 100)
#         height = int(output_img.shape[0] * scale_percent / 100)
#         dim = (width, height)
#         output_img = cv2.resize(output_img, dim, interpolation = cv2.INTER_AREA)

#         t2 = time.time()

#         ts.append(t2 - t1)

#         # save output image
#         cv2.imwrite(OUTPUT_DIR + "\\" + filename, output_img)
    
#     print(str(len(ts) / sum(ts)) + " fps")


# if __name__ == "__main__":
#     main()
