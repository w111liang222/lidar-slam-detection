import numpy as np
import sensor_driver.inference.iou3d_nms as iou3d_nms_utils

def class_agnostic_nms(box_scores, box_preds, box_labels, nms_config, score_thresh=None):
    for cls_idx, score in score_thresh.items():
        cls_mask = (box_labels == cls_idx)
        scores_mask = (box_scores < score)
        valid_mask = np.bitwise_and(cls_mask, scores_mask)
        box_scores = box_scores[~valid_mask]
        box_preds = box_preds[~valid_mask]
        box_labels = box_labels[~valid_mask]

    selected = []
    if box_scores.shape[0] > 0:
        keep_idx, selected_scores = getattr(iou3d_nms_utils, nms_config.NMS_TYPE)(
                box_preds, box_scores, nms_config.NMS_THRESH
        )
        selected = keep_idx[:nms_config.NMS_POST_MAXSIZE]
    return box_scores[selected], box_preds[selected], box_labels[selected]
