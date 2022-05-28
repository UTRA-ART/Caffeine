import torchvision.transforms as transforms
from PIL import Image
import onnxruntime
import numpy as np
import time
import torch


def to_numpy(tensor):
    return tensor.detach().cpu().numpy() if tensor.requires_grad else tensor.cpu().numpy()


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def non_max_suppression(prediction, conf_thres=0.4, iou_thres=0.6):
    '''
    prediction = [[[x, y, w, h, confidence, class], ...]]

    returns an nx6 with (x1, y1, x2, y2, confidence, class)
    '''

    candidates = prediction[..., 4] > conf_thres

    max_wh = 4096
    max_det = 300
    time_limit = 5.0

    t = time.time()
    output = [torch.zeros(0, 6)] * prediction.shape[0]

    for xi, x in enumerate(prediction):
        x = torch.tensor(x)
        x = x[candidates[xi]]

        if not x.shape[0]:
            continue
        x[:, 5:] *= x[:, 4:5]

        box = xywh2xyxy(x[:, :4])

        conf, j = torch.max(x[:, 5:], 1, keepdim=True)
        x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

        if not x.shape[0]:
            continue

        c = x[:, 5:6] * max_wh
        boxes, scores = x[:, :4] + c, x[:, 4]
        i = torch.ops.torchvision.nms(boxes, scores, iou_thres)
        if i.shape[0] > max_det:
            i = i[:max_det]

        output[xi] = x[i]

        if (time.time() - t) > time_limit:
            break
    return output


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(img1_shape[0] / img0_shape[0],
                   img1_shape[1] / img0_shape[1])  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / \
            2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
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
    boxes[:, 0].clamp_(0, img_shape[1])  # x1
    boxes[:, 1].clamp_(0, img_shape[0])  # y1
    boxes[:, 2].clamp_(0, img_shape[1])  # x2
    boxes[:, 3].clamp_(0, img_shape[0])  # y2


def run_inference(img, onnx_weights):
    ort_session = onnxruntime.InferenceSession(onnx_weights)

    img0x, img0y = img.size

    # Resize the image to [448, 448] and add padding (preserve ratio)
    img.thumbnail([448, 448], Image.ANTIALIAS)

    


    back = Image.new("RGB", (448, 448), 0)
    if img.width > img.height:
        back.paste(img, (0, int((448 / 2) - img.height / 2)))
    else:
        back.paste(img, (int((448 / 2) - img.width / 2), 0))
    img = back

    # Switch up the axis to be compatible with the model
    array = np.array(img)
    array = np.moveaxis(array, 0, 2)
    array = np.moveaxis(array, 0, 2)

    input = transforms.ToTensor()(array)
    input.unsqueeze_(0)
    input = to_numpy(input)
    input = np.moveaxis(input, 1, 3)

    # Run inference
    ort_inputs = {ort_session.get_inputs()[0].name: input}
    ort_outs = ort_session.run(None, ort_inputs)
    pred = ort_outs[0]

    # Non max suppression to get rid of invalid boxes
    pred = non_max_suppression(pred, conf_thres=0.5,
                               iou_thres=0.8)

    # List of bboxs in format [x1, y1, x2, y2, confs, class]
    output = []

    for i, det in enumerate(pred):
        det[:, :4] = scale_coords(img1_shape=(
            448, 448), coords=det[:, :4], img0_shape=(img0y, img0x))
        output.append(det)

    return output
