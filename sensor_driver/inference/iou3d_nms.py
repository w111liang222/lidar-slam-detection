import numpy as np
import iou3d_nms_ext

def boxes_bev_iou_cpu(boxes_a, boxes_b):
    """
    Args:
        boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
        boxes_b: (N, 7) [x, y, z, dx, dy, dz, heading]

    Returns:

    """
    boxes_a = np.ascontiguousarray(boxes_a, dtype=np.float32)
    boxes_b = np.ascontiguousarray(boxes_b, dtype=np.float32)
    ans_iou = np.zeros((boxes_a.shape[0], boxes_b.shape[0]), dtype=np.float32)
    iou3d_nms_ext.boxes_iou_bev_cpu(boxes_a, boxes_b, ans_iou)

    return ans_iou

def boxes_giou3d_gpu(boxes_a, boxes_b):
    """
    Args:
        boxes_a: (N, 7) [x, y, z, dx, dy, dz, heading]
        boxes_b: (N, 7) [x, y, z, dx, dy, dz, heading]

    Returns:
        ans_iou: (N, M)
    """

    boxes_a = np.ascontiguousarray(boxes_a, dtype=np.float32)
    boxes_b = np.ascontiguousarray(boxes_b, dtype=np.float32)
    # height overlap
    boxes_a_height_max = (boxes_a[:, 2] + boxes_a[:, 5] / 2).reshape(-1, 1)
    boxes_a_height_min = (boxes_a[:, 2] - boxes_a[:, 5] / 2).reshape(-1, 1)
    boxes_b_height_max = (boxes_b[:, 2] + boxes_b[:, 5] / 2).reshape(1, -1)
    boxes_b_height_min = (boxes_b[:, 2] - boxes_b[:, 5] / 2).reshape(1, -1)

    # bev overlap
    overlaps_bev = np.zeros((boxes_a.shape[0], boxes_b.shape[0]), dtype=np.float32)  # (N, M)
    iou3d_nms_ext.boxes_overlap_bev_gpu(boxes_a, boxes_b, overlaps_bev)

    convexhull_bev = np.zeros((boxes_a.shape[0], boxes_b.shape[0]), dtype=np.float32)  # (N, M)
    iou3d_nms_ext.boxes_union_bev_gpu(boxes_a, boxes_b, convexhull_bev)

    max_of_min = np.maximum(boxes_a_height_min, boxes_b_height_min)
    min_of_max = np.minimum(boxes_a_height_max, boxes_b_height_max)
    overlaps_h = np.clip(min_of_max - max_of_min, 0, None)

    min_of_min = np.minimum(boxes_a_height_min, boxes_b_height_min)
    max_of_max = np.maximum(boxes_a_height_max, boxes_b_height_max)
    unions_h = np.clip(max_of_max - min_of_min, 0, None)

    # 3d iou
    overlaps_3d = overlaps_bev * overlaps_h
    convexhull_3d = np.clip(convexhull_bev * unions_h, 1e-6, None)

    vol_a = (boxes_a[:, 3] * boxes_a[:, 4] * boxes_a[:, 5]).reshape(-1, 1)
    vol_b = (boxes_b[:, 3] * boxes_b[:, 4] * boxes_b[:, 5]).reshape(1, -1)
    unions_3d = np.clip(vol_a + vol_b - overlaps_3d, 1e-6, None)

    giou3d = overlaps_3d / unions_3d - (convexhull_3d - unions_3d) / convexhull_3d
    return giou3d

def nms_gpu(boxes, scores, thresh, pre_maxsize=None, **kwargs):
    """
    :param boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
    :param scores: (N)
    :param thresh:
    :return:
    """

    order = np.argsort(-scores)
    boxes = np.ascontiguousarray(boxes[order], dtype=np.float32)
    keep = np.zeros((boxes.shape[0]), dtype=np.int_)
    num_out = iou3d_nms_ext.nms_gpu(boxes, keep, thresh)
    return order[keep[:num_out]]

def nms_cpu(boxes, scores, thresh):
    """
    :param boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
    :param scores: (N)
    :param thresh:
    :return:
    """
    sorted_inds = np.argsort(-scores)
    keep = []
    while len(sorted_inds) > 0:
        keep.append(sorted_inds[0])
        if len(sorted_inds) == 1:
            break
        keep_box = boxes[sorted_inds[0], :]
        res_inds = sorted_inds[1:]
        res_boxes = boxes[res_inds, :]
        se_inds = np.arange(len(res_inds))
        # x
        keep_box_min_x = keep_box[0] - keep_box[3] / 2
        keep_box_max_x = keep_box[0] + keep_box[3] / 2
        res_boxes_min_x = res_boxes[:, 0] - res_boxes[:, 3] / 2
        res_boxes_max_x = res_boxes[:, 0] + res_boxes[:, 3] / 2
        min_x = np.maximum(res_boxes_min_x, keep_box_min_x)
        max_x = np.minimum(res_boxes_max_x, keep_box_max_x)
        x_overlap = max_x > min_x
        # y
        keep_box_min_y = keep_box[1] - keep_box[4] / 2
        keep_box_max_y = keep_box[1] + keep_box[4] / 2
        res_boxes_min_y = res_boxes[:, 1] - res_boxes[:, 4] / 2
        res_boxes_max_y = res_boxes[:, 1] + res_boxes[:, 4] / 2
        min_y = np.maximum(res_boxes_min_y, keep_box_min_y)
        max_y = np.minimum(res_boxes_max_y, keep_box_max_y)
        y_overlap = max_y > min_y
        # z
        keep_box_min_z = keep_box[2] - keep_box[5] / 2
        keep_box_max_z = keep_box[2] + keep_box[5] / 2
        res_boxes_min_z = res_boxes[:, 2] - res_boxes[:, 5] / 2
        res_boxes_max_z = res_boxes[:, 2] + res_boxes[:, 5] / 2
        min_z = np.maximum(res_boxes_min_z, keep_box_min_z)
        max_z = np.minimum(res_boxes_max_z, keep_box_max_z)
        z_overlap = max_z > min_z

        overlap_mask = x_overlap & y_overlap & z_overlap
        care_res_boxes = res_boxes[overlap_mask, :]
        overlap_se_inds = se_inds[overlap_mask]
        ious = boxes_bev_iou_cpu(np.expand_dims(keep_box, axis = 0), care_res_boxes)[0]
        delete_mask = ious > thresh
        delete_se_inds = overlap_se_inds[delete_mask]
        sorted_inds = np.delete(res_inds, delete_se_inds)

    keep = np.array(keep, dtype=np.int_)
    return keep