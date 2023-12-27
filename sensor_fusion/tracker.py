import numpy as np

import sensor_driver.common_lib.cpp_utils as util
from module.detect.detect_template import DetectTemplate
from sensor_fusion.MOT3D.model import MOT3D, PassThrough

def seperate_predict(pred_dicts, cls_idx):
    cls_mask = (pred_dicts['pred_attr'][:, 6] == cls_idx)

    cls_boxes = pred_dicts['pred_boxes'][cls_mask]
    cls_score = pred_dicts['pred_attr'][cls_mask, 5, None]
    cls_label = pred_dicts['pred_attr'][cls_mask, 6, None]
    track_dict = dict()
    track_dict['detections']  = np.concatenate((cls_boxes, cls_score, cls_label), axis=1)

    non_cls_mask = ~cls_mask
    pred_dicts['pred_boxes'] = pred_dicts['pred_boxes'][non_cls_mask]
    pred_dicts['pred_attr']  = pred_dicts['pred_attr'][non_cls_mask]
    pred_dicts['pred_traj']  = pred_dicts['pred_traj'][non_cls_mask]
    return pred_dicts, track_dict

def concat_trackers(pred_dicts, tracklet, trajectory):
    pred_dicts['pred_boxes'] = np.concatenate((pred_dicts['pred_boxes'], tracklet[:, :7]), axis=0)
    pred_dicts['pred_attr']  = np.concatenate((pred_dicts['pred_attr'],  tracklet[:, 7:]), axis=0)
    pred_dicts['pred_traj']  = np.concatenate((pred_dicts['pred_traj'],  trajectory), axis=0)
    return pred_dicts

def prepare_tracking(pred_dicts, class_names):
    pred_labels  = pred_dicts.pop('pred_labels')
    pred_scores  = pred_dicts.pop('pred_scores')
    num_obstacle = pred_labels.shape[0]

    pred_state   = np.zeros((num_obstacle, 4), dtype=np.float32) # vx, vy, gyro, ax
    pred_ids     = np.zeros((num_obstacle, 1), dtype=np.int32)
    pred_age     = np.ones((num_obstacle, 1), dtype=np.int32)
    pred_valid   = np.ones((num_obstacle, 1), dtype=np.bool)
    pred_status  = np.zeros((num_obstacle, 1), dtype=np.int32)

    # vx, vy, gyro, ax, id, score, label, age, valid, status
    pred_dicts['pred_attr'] = np.concatenate([pred_state, pred_ids, pred_scores, pred_labels, pred_age, pred_valid, pred_status], axis=1)
    pred_dicts['pred_traj'] = np.zeros((num_obstacle, 20, 7), dtype=np.float32)

    track_data = []
    for cls_idx, cls_name in enumerate(class_names):
        pred_dicts, track_dict = seperate_predict(pred_dicts, cls_idx + 1)
        track_data.append(track_dict)

    return track_data, pred_dicts

class Tracker(DetectTemplate):
    def __init__(self, logger = None):
        super().__init__(name = 'tracker', logger = logger)
        self.class_names = ['Vehicle', 'Pedestrian', 'Cyclist', 'Traffic_Cone']

    def prepare(self):
        util.init_filters()
        MOT3D.reset()

        self.veh_tracker = MOT3D(logger=self.logger, config = dict(
            max_age=5, min_hits=2, giou_th=1.0, movable=True,  score_th=0.30
        ))
        self.ped_tracker = MOT3D(logger=self.logger, config = dict(
            max_age=5, min_hits=3, giou_th=1.2, movable=True,  score_th=0.25
        ))
        self.cyc_tracker = MOT3D(logger=self.logger, config = dict(
            max_age=5, min_hits=3, giou_th=1.5, movable=True,  score_th=0.25
        ))
        self.con_tracker = MOT3D(logger=self.logger, config = dict(
            max_age=5, min_hits=3, giou_th=1.5, movable=False, score_th=0.20
        ))
        self.trackers = [self.veh_tracker, self.ped_tracker, self.cyc_tracker, self.con_tracker]

    def process(self, input_dict):
        motion_valid   = input_dict.pop('motion_valid')
        motion_t       = input_dict.pop('motion_t')
        motion_heading = input_dict.pop('motion_heading')
        timestep       = input_dict.pop('timestep') / 1000000.0

        track_data, pred_dicts = prepare_tracking(input_dict, self.class_names)
        for idx, tracker in enumerate(self.trackers):
            tracklet, trajectory = tracker.update(track_data[idx], motion_t, motion_heading, motion_valid, timestep)
            pred_dicts = concat_trackers(pred_dicts, tracklet, trajectory)

        return pred_dicts
