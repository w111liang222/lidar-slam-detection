import numpy as np
import sensor_driver.inference.iou3d_nms as iou3d_nms_utils

def class_agnostic_nms(box_scores, box_preds, box_labels, nms_config, score_thresh=None):
    mask = np.zeros((box_scores.shape[0]), dtype=np.bool)
    for cls_idx, score in score_thresh.items():
        cls_mask = np.bitwise_and((box_labels == cls_idx), (box_scores >= score))
        mask = np.bitwise_or(mask, cls_mask)

    box_scores = box_scores[mask]
    box_preds  = box_preds[mask]
    box_labels = box_labels[mask]

    selected = []
    if box_scores.shape[0] > 0:
        keep_idx = getattr(iou3d_nms_utils, nms_config.NMS_TYPE)(
                box_preds, box_scores, nms_config.NMS_THRESH
        )
        selected = keep_idx[:nms_config.NMS_POST_MAXSIZE]
    return box_scores[selected], box_preds[selected], box_labels[selected]
