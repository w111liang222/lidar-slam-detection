import numpy as np
import cv2
from sensor_inference.utils.image_ops_utils import xywh2xyxy, nms_2d_box

class PostProcesser():
    def __init__(self, model_cfg, num_class, class_names):
        self.model_cfg = model_cfg
        self.num_class = num_class
        self.class_names = class_names

        # traffic light color dict
        self.color_dict = {
            'black':  {'min': np.array([0,   0,     0]), 'max': np.array([180, 255,  46])},
            'red1':   {'min': np.array([0,   66,   66]), 'max': np.array([10,  255, 255])},
            'red2':   {'min': np.array([156, 66,   66]), 'max': np.array([180, 255, 255])},
            'green':  {'min': np.array([35,  66,   66]), 'max': np.array([110, 255, 255])},
            'yellow1':{'min': np.array([20,  66,   66]), 'max': np.array([32,  255, 255])},
            'yellow2':{'min': np.array([220, 66,   66]), 'max': np.array([255, 255, 255])},
        }

    def forward(self, image, preds):
        pred_dicts = self.post_processing(image, preds)
        return pred_dicts

    def post_processing(self, image, preds):
        preds = preds.transpose((1, 0))

        # confidence filter
        mask  = (preds[:, 4] > self.model_cfg.SCORE_THRESH)
        preds = preds[mask]
        boxes, scores, labels = preds[:, :4], preds[:, 4], preds[:, 5]

        # NMS
        if boxes.shape[0] > 0:
            boxes = xywh2xyxy(boxes) # xywh to xyxy
            keep = nms_2d_box(boxes, scores, self.model_cfg.IOU_THRESHOLD)
            boxes, scores, labels = boxes[keep], scores[keep], labels[keep]

        colors = self.detect_colors(image, boxes)

        result_dict = {
            'pred_boxes':  boxes,
            'pred_scores': scores,
            'pred_labels': labels,
            'pred_colors': colors,
        }
        return result_dict

    # recognize color in HSV space
    def detect_colors(self, image, pred_boxes):
        colors = np.zeros(pred_boxes.shape[0], dtype=np.int8)
        for i in range(pred_boxes.shape[0]):
            bbox = pred_boxes[i, :]
            crop = image[round(bbox[1]):round(bbox[3]), round(bbox[0]):round(bbox[2])]
            if crop.shape[0] == 0 or crop.shape[1] == 0:
                colors[i] = -1
                continue

            crop_HSV = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

            mask_red    = cv2.inRange(crop_HSV, self.color_dict['red1']['min'], self.color_dict['red1']['max']) + \
                          cv2.inRange(crop_HSV, self.color_dict['red2']['min'], self.color_dict['red2']['max'])
            mask_green  = cv2.inRange(crop_HSV, self.color_dict['green']['min'], self.color_dict['green']['max'])
            mask_yellow = cv2.inRange(crop_HSV, self.color_dict['yellow1']['min'], self.color_dict['yellow1']['max']) + \
                          cv2.inRange(crop_HSV, self.color_dict['yellow2']['min'], self.color_dict['yellow2']['max'])

            # morphology filter
            kernel = np.ones([3, 3], np.uint8)

            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            red      = sum(sum(mask_red.astype(int)))

            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
            green      = sum(sum(mask_green.astype(int)))

            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
            yellow      = sum(sum(mask_yellow.astype(int)))

            ratio = max([red, green, yellow]) / (255 * crop.shape[0] * crop.shape[1])
            if ratio < 0.02:
                colors[i] = 0
            elif max([red, green, yellow]) == red:
                colors[i] = 1
            elif max([red, green, yellow]) == green:
                colors[i] = 2
            elif max([red, green, yellow]) == yellow:
                colors[i] = 3

        return colors
