from math import ceil, floor
import onnxruntime
import numpy as np
import time
import cv2


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def non_max_suppression(prediction, conf_thres=0.4, iou_thres=0.6):
    """
    prediction = [[[x, y, w, h, confidence, class], ...]]

    returns an nx6 with (x1, y1, x2, y2, confidence, class)
    """

    candidates = prediction[..., 4] > conf_thres

    max_det = 300
    time_limit = 0.05

    t = time.time()
    output = [np.zeros(6)] * prediction.shape[0]

    for xi, x in enumerate(prediction):
        x = x[candidates[xi]]

        if not x.shape[0]:
            continue

        x[:, 5:] *= x[:, 4:5]

        box = xywh2xyxy(x[:, :4])

        conf = x[:, 5:]
        j = np.zeros(conf.shape)
        x = np.concatenate((box, conf, j / 1.0), 1)
        x = np.asarray([x_e for x_e in x if x_e[4] > conf_thres])

        if not x.shape[0]:
            continue

        # Sort the boxes in descending order before passing through nms
        x = np.flipud(x[x[:, 4].argsort()])
        boxes = x[:, :4]
        i = nms(boxes, iou_thres)
        b_dets = x[i]

        if b_dets.shape[0 > max_det]:
            b_dets = b_dets[:max_det]

        output[xi] = b_dets

        if (time.time() - t) > time_limit:
            break

    return output


def nms(boxes, iou_thres=0.6):

    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]

    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    indices = np.arange(len(x1))
    keep_index = np.ones(len(x1))

    for i, box in enumerate(boxes):
        if not keep_index[i]:
            continue

        # Find the area of the intersection
        X1 = np.maximum(box[0], boxes[indices, 0])
        Y1 = np.maximum(box[1], boxes[indices, 1])
        X2 = np.minimum(box[2], boxes[indices, 2])
        Y2 = np.minimum(box[3], boxes[indices, 3])

        widths = np.maximum(0, X2 - X1 + 1)
        heights = np.maximum(0, Y2 - Y1 + 1)

        overlaps = (widths * heights) / (areas[indices] + areas[i] - (widths * heights))

        # If the overlap is greater than the threshold, remove
        if np.any([iou > iou_thres if iou != 1.0 else False for iou in overlaps]):
            keep_index = np.multiply(
                keep_index,
                np.array(
                    [int(iou < iou_thres) if iou != 1.0 else 1 for iou in overlaps]
                ),
            )

    keep_index = [True if x == 1 else False for x in keep_index]
    return indices[keep_index]


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(
            img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1]
        )  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (
            img1_shape[0] - img0_shape[0] * gain
        ) / 2  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2]] -= pad[0]  # x padding
    coords[:, [1, 3]] -= pad[1]  # y padding
    coords[:, :4] /= gain
    clip_coords(coords, img0_shape)
    return coords


def clip_coords(boxes, img_shape):
    # Clip bounding xyxy bounding boxes to image shape (height, width)
    boxes[:, 1].clip(0, img_shape[0])  # y1
    boxes[:, 0].clip(0, img_shape[1])  # x1
    boxes[:, 2].clip(0, img_shape[1])  # x2
    boxes[:, 3].clip(0, img_shape[0])  # y2


def run_inference(image, ort_session):
    img = image.copy()
    # ort_session = onnxruntime.InferenceSession(onnx_weights)

    # Resize the image to [448, 448] and add padding (preserve ratio)
    # img = cv2.imread(img)
    img0x, img0y = img.shape[1], img.shape[0]

    ratio = 448 / img0x
    size = (448, int(img0y * ratio))
    img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)

    newx, newy = img.shape[1], img.shape[0]
    if newx > newy:
        padding = (448 - newy) / 2
        img = cv2.copyMakeBorder(img, floor(padding), ceil(padding), 0, 0, cv2.BORDER_CONSTANT)
    else:
        padding = (448 - newx) // 2
        img = cv2.copyMakeBorder(img, 0, 0, padding, padding, cv2.BORDER_CONSTANT)
    img = img[..., ::-1]

    # Switch up the axis to be compatible with the model
    array = np.array(img)
    array = np.moveaxis(array, 0, 2)
    array = np.moveaxis(array, 0, 2)

    input = np.expand_dims(array, axis=0)
    input = input.astype(np.float32)
    input /= 255.0

    # Run inference
    ort_inputs = {ort_session.get_inputs()[0].name: input}
    ort_outs = ort_session.run(None, ort_inputs)
    pred = ort_outs[0]

    # Non max suppression to get rid of invalid boxes
    pred = non_max_suppression(pred, conf_thres=0.5, iou_thres=0.81)

    if pred is None:
        return
    if np.all(pred[0] == 0):
        return
    # List of bboxs in format [x1, y1, x2, y2, confs, class]
    output = []

    for i, det in enumerate(pred):
        det[:, :4] = scale_coords(
            img1_shape=(448, 448), coords=det[:, :4], img0_shape=(180, 330)
        )
        output.append(det)

    return output
