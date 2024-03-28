import cv2
import numpy as np

def cvt_image(img, w, h):
    img = resize_image(img, w, h)

    shape = img.shape
    # transform to BGR
    if shape[-1] != 3:
        img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_I420)

    return img

def resize_image(img, w, h):
    shape = img.shape
    if shape[-1] != 3:
        h = int(h * 3 / 2)

    if shape[0] != h or shape[1] != w:
        img = cv2.resize(img, (w, h))

    return img

def get_image_size(img):
    shape = img.shape
    if shape[-1] != 3:
        height, width = shape
        height = int(height * 2 / 3)
    else:
        height, width, _ = shape
    return width, height

def undistort_image(img, cfg):
    w  = cfg['output_width']  if 'output_width'  in cfg else 640
    h  = cfg['output_height'] if 'output_height' in cfg else 480
    fisheye = cfg.get("fisheye", False)
    fx = cfg['intrinsic_parameters'][0]
    fy = cfg['intrinsic_parameters'][1]
    cx = cfg['intrinsic_parameters'][2]
    cy = cfg['intrinsic_parameters'][3]
    k1 = cfg['intrinsic_parameters'][4]
    k2 = cfg['intrinsic_parameters'][5]
    p1 = cfg['intrinsic_parameters'][6]
    p2 = cfg['intrinsic_parameters'][7]
    mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    dist = np.array([k1, k2, p1, p2])

    img = cvt_image(img, w, h)
    if fisheye:
        xmap, ymap = cv2.fisheye.initUndistortRectifyMap(mtx, dist, np.eye(3), mtx, (w, h), cv2.CV_16SC2)
        img = cv2.remap(img, xmap, ymap, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    else:
        img = cv2.undistort(img, mtx, dist, None)
    return img