import numpy as np
import math

def xywh2xyxy(x):
    """
    Convert bounding box coordinates from (x, y, width, height) format to (x1, y1, x2, y2) format where (x1, y1) is the
    top-left corner and (x2, y2) is the bottom-right corner.

    Args:
        x (np.ndarray): The input bounding box coordinates in (x, y, width, height) format.
    Returns:
        y (np.ndarray): The bounding box coordinates in (x1, y1, x2, y2) format.
    """
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2  # top left x
    y[..., 1] = x[..., 1] - x[..., 3] / 2  # top left y
    y[..., 2] = x[..., 0] + x[..., 2] / 2  # bottom right x
    y[..., 3] = x[..., 1] + x[..., 3] / 2  # bottom right y
    return y
# numpy nms
def box_area(boxes :np.ndarray):
    return (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])

def box_iou(box1 :np.ndarray, box2: np.ndarray):
    area1 = box_area(box1)  # N
    area2 = box_area(box2)  # M
    lt = np.maximum(box1[:, np.newaxis, :2], box2[:, :2])
    rb = np.minimum(box1[:, np.newaxis, 2:], box2[:, 2:])
    wh = rb - lt
    wh = np.maximum(0, wh) # [N, M, 2]
    inter = wh[:, :, 0] * wh[:, :, 1]
    iou = inter / (area1[:, np.newaxis] + area2 - inter)
    return iou  # NxM

def nms_2d_box(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float):
    idxs = scores.argsort()
    keep = []
    while idxs.size > 0:
        max_score_index = idxs[-1]
        max_score_box = boxes[max_score_index][None, :]
        keep.append(max_score_index)
        if idxs.size == 1:
            break
        idxs = idxs[:-1]
        other_boxes = boxes[idxs]
        ious = box_iou(max_score_box, other_boxes)
        idxs = idxs[ious[0] <= iou_threshold]
    keep = np.array(keep)
    return keep

def cost_matrix(lights_map, lights_det):
    cost_matrix = np.zeros([lights_map.shape[0], lights_det.shape[0]], dtype=float)
    for i in range(lights_map.shape[0]):
        for j in range(lights_det.shape[0]):
            center_det_x = (lights_det[j][0] + lights_det[j][2]) / 2
            center_det_y = (lights_det[j][1] + lights_det[j][3]) / 2
            dist = pow(center_det_x - lights_map[i, 0], 2) + pow(center_det_y - lights_map[i, 1], 2)
            area = (lights_det[j][2] - lights_det[j][0]) * (lights_det[j][3] - lights_det[j][1])
            cost_matrix[i][j] = dist/area if dist/area < 10 else 1e8
    return cost_matrix