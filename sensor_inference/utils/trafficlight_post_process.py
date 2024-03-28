import os
import numpy as np
from scipy.optimize import linear_sum_assignment
from sensor_inference.utils.parse_map import parse_xodr, parse_anchor
from sensor_inference.utils.image_ops_utils import xywh2xyxy, nms_2d_box, cost_matrix
from sensor_driver.common_lib.cpp_utils import get_global_transform, get_transform_from_cfg
from util.common_util import has_extension_disk

class PostProcesser():
    def __init__(self, model_cfg, num_class, class_names, ins_extrinsic, map_path):
        self.model_cfg = model_cfg
        self.num_class = num_class
        self.class_names = class_names

        has_disk, disk_name = has_extension_disk()
        self.light_list, self.light_pos = parse_xodr(os.path.join(disk_name, map_path, 'graph/map.xodr'))
        self.anchor = parse_anchor(os.path.join(disk_name, map_path, 'graph/map_info.txt'))
        self.ins_transform = get_transform_from_cfg(ins_extrinsic[0], ins_extrinsic[1], ins_extrinsic[2],
                                                    ins_extrinsic[3], ins_extrinsic[4], ins_extrinsic[5])
        self.global_transform = np.linalg.inv(get_global_transform(self.anchor.lat, self.anchor.lon, self.anchor.alt, 0, 0, 0)) if self.anchor else np.eye(4)

    def forward(self, image, preds, ins_data, cam_param):
        pred_dicts = self.post_processing(image, preds, ins_data, cam_param)
        return pred_dicts

    def post_processing(self, image, preds, ins_data, cam_param):
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

        colors = labels // 6
        pictograms = labels % 6

        result_dict = {
            'pred_boxes':  boxes,
            'pred_scores': scores,
            'pred_pictograms': pictograms,
            'pred_colors': colors,
            'pred_names': [''] * boxes.shape[0],
            'pred_ids': ['0'] * boxes.shape[0]      # 0 indicates an invalid map_primitive_id
        }

        if self.anchor and self.light_list.any() and ins_data:
            result_dict = self.matching_map(image, result_dict, ins_data, cam_param)

        return result_dict

    # match traffic light in xodr map
    def matching_map(self, image, result_dict, ins_data, cam_param):
        # project xodr lights to image coordinate
        poseV = np.matmul(np.matmul(self.global_transform, ins_data['pose']), self.ins_transform)
        lights_front, light_poseV_matrix = self.project2V(poseV)
        lights_image, lights_image_pos = self.project2C(image, lights_front, light_poseV_matrix, cam_param)

        # matching, Hungarian Algorithm
        dist_matrix = cost_matrix(lights_image_pos, result_dict['pred_boxes'])
        row_ind, col_ind = linear_sum_assignment(dist_matrix)
        for i, j in zip(row_ind, col_ind):
            if dist_matrix[i][j] < 10:
                result_dict['pred_names'][j] = lights_image[i].name
                result_dict['pred_ids'][j] = lights_image[i].id

        return result_dict

    # project light pose to vehicle coordinate
    def project2V(self, pose):
        light_pose_matrix = np.expand_dims(self.light_pos, axis=2)
        light_poseV_matrix = np.linalg.inv(pose) @ light_pose_matrix
        mask = np.reshape(np.logical_and(light_poseV_matrix[:, 0] > 2, np.linalg.norm(light_poseV_matrix, axis=1) < 100), (light_poseV_matrix.shape[0]))
        lights_forward = self.light_list[mask]
        light_poseV_matrix = light_poseV_matrix[mask]
        return lights_forward, light_poseV_matrix

    # project light pose (vehicle coordinate) to image coordinate
    def project2C(self, image, light_list, light_poseV_matrix, cam_param):
        cam_matrix = cam_param['V2C'] @ light_poseV_matrix
        img_matrix = cam_param['intrinsic'] @ cam_matrix
        img_matrix = np.reshape(img_matrix, (light_poseV_matrix.shape[0], 3))
        last_colum = np.expand_dims(img_matrix[:, -1], axis=1)
        img_matrix = img_matrix / last_colum
        valid0_ind = np.logical_and(img_matrix[:, 0] > 0, img_matrix[:, 0] < image.shape[1])
        valid1_ind = np.logical_and(img_matrix[:, 1] > 0, img_matrix[:, 1] < image.shape[0])
        mask = np.logical_and(valid0_ind, valid1_ind)
        lights_image = light_list[mask]
        lights_image_pos = img_matrix[mask]
        return lights_image, lights_image_pos
