import numpy as np
from hardware.platform_common import is_jetson
import sensor_driver.common_lib.iou3d_nms as iou3d_nms_utils

def class_agnostic_nms_lidar(box_scores, box_preds, box_labels, nms_config, score_thresh=None):
    for cls_idx, score in score_thresh.items():
        cls_mask = (box_labels == cls_idx)
        scores_mask = (box_scores < score)
        valid_mask = np.bitwise_and(cls_mask, scores_mask)
        box_scores = box_scores[~valid_mask]
        box_preds = box_preds[~valid_mask]
        box_labels = box_labels[~valid_mask]

    selected = []
    if box_scores.shape[0] > 0:
        nms_type = nms_config.NMS_TYPE if is_jetson() else "nms_cpu"
        keep_idx, selected_scores = getattr(iou3d_nms_utils, nms_type)(
                box_preds, box_scores, nms_config.NMS_THRESH
        )
        selected = keep_idx[:nms_config.NMS_POST_MAXSIZE]
    return box_scores[selected], box_preds[selected], box_labels[selected]

def class_agnostic_nms_camera(box_scores, box_preds, nms_config):
    selected = []
    if box_scores.shape[0] > 0:
        keep_idx, selected_scores = getattr(iou3d_nms_utils, nms_config.NMS_TYPE)(
                box_preds, box_scores, nms_config.NMS_THRESH
        )
        selected = keep_idx[:nms_config.NMS_POST_MAXSIZE]
    return selected

def class_agnostic_nms_fusion(box_scores, box_preds, threshold):
    selected = []
    if box_scores.shape[0] > 0:
        keep_idx, selected_scores = getattr(iou3d_nms_utils, 'nms_gpu')(
                box_preds, box_scores, threshold
        )
        selected = keep_idx[:256]
    return selected