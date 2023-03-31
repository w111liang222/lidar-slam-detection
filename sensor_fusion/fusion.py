import numpy as np
import copy

import sensor_driver.common_lib.cpp_utils as util
import sensor_driver.common_lib.iou3d_nms as iou3d_nms_utils
from module.detect.detect_template import DetectTemplate
from util.box_utils import boxes3d_lidar_to_camera, boxes3d_to_corners3d_camera, corners3d_to_img_boxes, lidar_to_img

def affine_transform(pt, t):
    new_pt = np.array([pt[0], pt[1], 1.], dtype=np.float32).T
    new_pt = np.dot(t, new_pt)
    return new_pt[:2]

def iou_xyxy(box1, box2):
    x1min, y1min, x1max, y1max = box1[0], box1[1], box1[2], box1[3]
    x2min, y2min, x2max, y2max = box2[0], box2[1], box2[2], box2[3]

    s1 = (y1max - y1min + 1.) * (x1max - x1min + 1.)
    s2 = (y2max - y2min + 1.) * (x2max - x2min + 1.)

    xmin = max(x1min, x2min)
    ymin = max(y1min, y2min)
    xmax = min(x1max, x2max)
    ymax = min(y1max, y2max)

    inter_h = max(ymax - ymin + 1, 0)
    inter_w = max(xmax - xmin + 1, 0)

    intersection = inter_h * inter_w
    union = s1 + s2 - intersection

    iou = intersection / union
    return iou

def match_iou(lidar_bboxes, image_bboxes, lidar_3dbox, image_3dbox):
    from scipy.optimize import linear_sum_assignment
    iou_matrix = np.zeros((lidar_bboxes.shape[0], image_bboxes.shape[0]), dtype=np.float32)
    if (image_bboxes.shape[0]==0):
        return np.empty((0, 2), dtype=np.int32), \
               np.arange(lidar_bboxes.shape[0], dtype=np.int32), \
               np.empty((0), dtype=np.int32), \
               iou_matrix
    if (lidar_bboxes.shape[0]==0):
        return np.empty((0, 2), dtype=np.int32), \
               np.empty((0), dtype=np.int32), \
               np.arange(image_bboxes.shape[0], dtype=np.int32), \
               iou_matrix

    for d, lidar in enumerate(lidar_bboxes):
        for t, image in enumerate(image_bboxes):
            iou_matrix[d, t] = iou_xyxy(lidar, image)

    iou3d_matrix = iou3d_nms_utils.boxes_bev_iou_cpu(lidar_3dbox, image_3dbox)

    iou_matrix[np.isneginf(-iou_matrix) | np.isnan(-iou_matrix)] = -1e4
    iou3d_matrix[np.isneginf(-iou3d_matrix) | np.isnan(-iou3d_matrix)] = -1e4
    row_ind, col_ind = linear_sum_assignment(-iou_matrix * 0.5 - iou3d_matrix * 0.5)      # hougarian algorithm
    matched_indices = np.stack((row_ind, col_ind), axis=1)

    unmatched_lidar = []
    for d, det in enumerate(lidar_bboxes):
        if (d not in matched_indices[:, 0]): unmatched_lidar.append(d)
    unmatched_image = []
    for t, trk in enumerate(image_bboxes):
        if (t not in matched_indices[:, 1]): unmatched_image.append(t)

    #filter out matched with low IOU
    matches = []
    for m in matched_indices:
        if (iou_matrix[m[0], m[1]] < 0.4):
            unmatched_lidar.append(m[0])
            unmatched_image.append(m[1])
        else: matches.append(m.reshape(1, 2))
    if (len(matches) == 0):
        matches = np.empty((0, 2), dtype=np.int32)
    else: matches = np.concatenate(matches, axis=0)

    # check unmatched image's iou with lidar, if overlay > 0.1, consider it as a duplicate
    valid_img_mask = np.ones(len(unmatched_image), dtype=np.bool)
    for t, img in enumerate(unmatched_image):
        img_drop = False
        for k, iou in enumerate(iou_matrix[:, img]):
            if iou > 0.1:
                img_drop = True
                break
        valid_img_mask[t] = (not img_drop)
    unmatched_image = np.array(unmatched_image, dtype=np.int32)[valid_img_mask]

    return matches, np.array(unmatched_lidar, dtype=np.int32), unmatched_image, iou3d_matrix

class Fusion(DetectTemplate):
    Lidar_Weight = 0.5
    Image_Weight = 0.5
    IOU_Gain = 0.2
    def __init__(self, cfg, logger = None):
        super().__init__(name = 'fusion', logger = logger)
        self.cfg = cfg
        self.logger = logger
        self.lidar_data = []
        self.image_data = []

    def do_fusion(self, lidar_data, image_data):
        result = lidar_data
        if len(self.cfg['source']) == 0:
            return result

        if 'camera' in self.cfg['source']:
            result = self.fusion_camera(result, image_data)

        return result

    def fusion_camera(self, result, image_data):
        lidar_pred_boxes = result['pred_boxes']
        lidar_pred_scores = result['pred_scores']
        lidar_pred_labels = result['pred_labels']
        lidar_pred_sensor = result['pred_sensor']

        image_det = image_data.pop('image_det')
        image_name = sorted(image_det.keys())[0]
        image_det = image_det[image_name]

        image_pred_boxes = image_det['pred_boxes']
        image_pred_scores = image_det['pred_scores']
        image_pred_labels = image_det['pred_labels']
        image_pred_sensor = image_det['pred_sensor']
        image_pred_bboxes = image_det['pred_bboxes']
        w = image_data['image_param'][image_name]['w']
        h = image_data['image_param'][image_name]['h']
        intrinsic = image_data['image_param'][image_name]['intrinsic']
        V2C = image_data['image_param'][image_name]['V2C'][:3]

        camera_boxes = boxes3d_lidar_to_camera(lidar_pred_boxes, V2C)
        box2d_corner, box2d_corner_depth = corners3d_to_img_boxes(boxes3d_to_corners3d_camera(camera_boxes), intrinsic)
        box_min = np.min(box2d_corner, axis=1)
        box_max = np.max(box2d_corner, axis=1)
        lidar_pred_bboxes = np.concatenate((box_min, box_max, lidar_pred_scores), axis=1)
        lidar_pred_bboxes[:, [0, 2]] = np.clip(lidar_pred_bboxes[:, [0, 2]], 0, w - 1)
        lidar_pred_bboxes[:, [1, 3]] = np.clip(lidar_pred_bboxes[:, [1, 3]], 0, h - 1)
        box_mask = np.bitwise_and((lidar_pred_bboxes[:, 2] - lidar_pred_bboxes[:, 0]) > 0 ,
                                  (lidar_pred_bboxes[:, 3] - lidar_pred_bboxes[:, 1]) > 0)
        depth_mask = np.all(box2d_corner_depth > 0, axis=1)
        mask = np.bitwise_and(box_mask, depth_mask)

        in_lidar_pred_boxes = lidar_pred_boxes[mask]
        in_lidar_pred_scores = lidar_pred_scores[mask]
        in_lidar_pred_labels = lidar_pred_labels[mask]
        in_lidar_pred_sensor = 3 * np.ones((in_lidar_pred_labels.shape[0], 1), dtype=np.int32) # Lidar-Camera
        in_lidar_pred_bboxes = lidar_pred_bboxes[mask]

        out_lidar_pred_boxes = lidar_pred_boxes[~mask]
        out_lidar_pred_scores = lidar_pred_scores[~mask]
        out_lidar_pred_labels = lidar_pred_labels[~mask]
        out_lidar_pred_sensor = lidar_pred_sensor[~mask]

        in_lidar_box2d_center, _ = lidar_to_img(copy.copy(in_lidar_pred_boxes[:, :3]), V2C, intrinsic)
        in_lidar_box2d_center[:, 0] = np.clip(in_lidar_box2d_center[:, 0], 0, w - 1)
        in_lidar_box2d_center[:, 1] = np.clip(in_lidar_box2d_center[:, 1], 0, h - 1)

        matched, unmatched_lidar, unmatched_image, iou_matrix = match_iou(in_lidar_pred_bboxes[:, :4],
                                                                          image_pred_bboxes[:, :4],
                                                                          in_lidar_pred_boxes[:, :7],
                                                                          image_pred_boxes[:, :7])

        un_image_pred_boxes = image_pred_boxes[unmatched_image]
        un_image_pred_scores = image_pred_scores[unmatched_image]
        un_image_pred_labels = image_pred_labels[unmatched_image]
        un_image_pred_sensor = image_pred_sensor[unmatched_image]

        for t, m in enumerate(matched):
            in_lidar_pred_scores[m[0]] = in_lidar_pred_scores[m[0]] * Fusion.Lidar_Weight + \
                                         image_pred_scores[m[1]] * Fusion.Image_Weight + \
                                         iou_matrix[m[0], m[1]] * Fusion.IOU_Gain
            in_lidar_pred_scores[m[0]] = min(1.0, in_lidar_pred_scores[m[0]])

        image_pred_heatmap = image_det.pop('pred_heatmap', None)
        trans_output = image_data['image_param'][image_name]['trans_output']
        if image_pred_heatmap is not None:
            search_range = 4
            o_w = image_pred_heatmap.shape[1]
            o_h = image_pred_heatmap.shape[0]
            for t, m in enumerate(unmatched_lidar):
                box2d_center = affine_transform(in_lidar_box2d_center[m], trans_output)
                box2d_center = box2d_center.astype(np.int32)
                class_channel = int(in_lidar_pred_labels[m] - 1)
                x_min = np.clip(box2d_center[0] - search_range, 0, o_w - 1)
                x_max = np.clip(box2d_center[0] + search_range, 1, o_w)
                y_min = np.clip(box2d_center[1] - search_range, 0, o_h - 1)
                y_max = np.clip(box2d_center[1] + search_range, 1, o_h)
                image_score = np.max(image_pred_heatmap[y_min:y_max, x_min:x_max, class_channel])

                in_lidar_pred_scores[m] = in_lidar_pred_scores[m] * Fusion.Lidar_Weight + \
                                          image_score * Fusion.Image_Weight

        pred_boxes = np.concatenate([out_lidar_pred_boxes, in_lidar_pred_boxes, un_image_pred_boxes], axis=0)
        pred_scores = np.concatenate([out_lidar_pred_scores, in_lidar_pred_scores, un_image_pred_scores], axis=0)
        pred_labels = np.concatenate([out_lidar_pred_labels, in_lidar_pred_labels, un_image_pred_labels], axis=0)
        pred_sensor = np.concatenate([out_lidar_pred_sensor, in_lidar_pred_sensor, un_image_pred_sensor], axis=0)

        result['pred_boxes'] = pred_boxes
        result['pred_scores'] = pred_scores
        result['pred_labels'] = pred_labels
        result['pred_sensor'] = pred_sensor

        return result

    def only_process_lidar(self, lidar_data):
        return lidar_data

    def only_process_image(self, image_data):
        image_det = image_data.pop('image_det')
        for name in image_det:
            image_det[name].pop('pred_heatmap', None)
        # only output the first camera detection
        image_name = sorted(image_det.keys())[0]
        image_data.update(image_det[image_name])
        return image_data

    def _run_thread(self):
        util.init_backtrace_handle()
        super()._run_thread()

    def process(self, input_dict):
        if 'lidar' in input_dict:
            self.lidar_data.append(input_dict['lidar'])
        elif 'image' in input_dict:
            self.image_data.append(input_dict['image'])
        else:
            return dict()
        while len(self.lidar_data) > 0 and len(self.image_data) > 0:
            head_lidar = self.lidar_data[0]
            head_image = self.image_data[0]
            if head_lidar['frame_start_timestamp'] == head_image['frame_start_timestamp']:
                self.lidar_data.remove(head_lidar)
                self.image_data.remove(head_image)
                fusion_data = self.do_fusion(head_lidar, head_image)
                return fusion_data
            elif head_lidar['frame_start_timestamp'] > head_image['frame_start_timestamp']:
                self.image_data.remove(head_image)
                return self.only_process_image(head_image)
            elif head_lidar['frame_start_timestamp'] < head_image['frame_start_timestamp']:
                self.lidar_data.remove(head_lidar)
                return self.only_process_lidar(head_lidar)

        if len(self.lidar_data) > 0:
            head_lidar = self.lidar_data[0]
            if not head_lidar['image_valid']:
                self.lidar_data.remove(head_lidar)
                return self.only_process_lidar(head_lidar)
            else:
                return dict()
        if len(self.image_data) > 0:
            head_image = self.image_data[0]
            if not head_image['lidar_valid']:
                self.image_data.remove(head_image)
                return self.only_process_image(head_image)
            else:
                return dict()

