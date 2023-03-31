import numpy as np

import sensor_driver.common_lib.cpp_utils as util
from module.detect.detect_template import DetectTemplate
from sensor_fusion.AB3DMOT.model import AB3DMOT, PassThrough

def seperate_predict(pred_dicts, key):
    cls_mask = (pred_dicts['pred_attr'][:, 6] == key)
    cls_dict = dict()
    score = np.expand_dims(pred_dicts['pred_attr'][cls_mask, 5], axis=1)
    cls_dict['dets'] = np.concatenate((pred_dicts['pred_boxes'][cls_mask], score), axis = 1).astype(np.float32)
    cls_dict['label'] = np.expand_dims(pred_dicts['pred_attr'][cls_mask, 6], axis=1)
    cls_dict['sensor'] = np.expand_dims(pred_dicts['pred_attr'][cls_mask, 10], axis=1)
    # remain the other class box
    non_cls_mask = ~cls_mask
    pred_dicts['pred_boxes'] = pred_dicts['pred_boxes'][non_cls_mask]
    pred_dicts['pred_attr'] = pred_dicts['pred_attr'][non_cls_mask]
    pred_dicts['pred_traj'] = pred_dicts['pred_traj'][non_cls_mask]
    return pred_dicts, cls_dict

def concat_trackers(track_dict, trackers, traj):
    track_dict['pred']['pred_boxes'] = np.concatenate((track_dict['pred']['pred_boxes'], trackers[:, 0:7]), axis=0)
    track_dict['pred']['pred_attr'] = np.concatenate((track_dict['pred']['pred_attr'], trackers[:, 7:]), axis=0)
    track_dict['pred']['pred_traj'] = np.concatenate((track_dict['pred']['pred_traj'], traj), axis=0)
    return track_dict

def prepare_tracking(pred_dicts, cls_dict):
    pred_labels = pred_dicts.pop('pred_labels')
    pred_scores = pred_dicts.pop('pred_scores')
    pred_sensor = pred_dicts.pop('pred_sensor')
    num_obstacle = pred_labels.shape[0]

    pred_ids = np.zeros((num_obstacle, 1), dtype = np.int32)
    pred_age = np.ones((num_obstacle, 1), dtype = np.int32)
    pred_valid = np.ones((num_obstacle, 1), dtype = np.bool)
    pred_status = np.zeros((num_obstacle, 1), dtype = np.int32)
    pred_state = np.zeros((num_obstacle, 4), dtype = np.float32)
    # velo_x, velo_y, heading_rate, accel_x, id, score, label, age, valid, status, sensor
    pred_dicts['pred_attr'] = np.concatenate([pred_state, pred_ids, pred_scores, pred_labels, pred_age, pred_valid, pred_status, pred_sensor], axis = 1)
    pred_dicts['pred_traj'] = np.zeros((num_obstacle, 20, 7), dtype = np.float32)
    for (key, cls_name) in cls_dict.items():
        if cls_name == 'Vehicle':
            pred_dicts, veh_dist = seperate_predict(pred_dicts, key)
        elif cls_name == 'Pedestrian':
            pred_dicts, ped_dist = seperate_predict(pred_dicts, key)
        elif cls_name == 'Cyclist':
            pred_dicts, cyc_dist = seperate_predict(pred_dicts, key)
        elif cls_name == 'Traffic_Cone':
            pred_dicts, con_dist = seperate_predict(pred_dicts, key)

    track_dict = {'veh': veh_dist, 'ped': ped_dist, 'cyc': cyc_dist, 'con': con_dist,'pred': pred_dicts}
    return track_dict

class Tracker(DetectTemplate):
    def __init__(self, logger = None):
        super().__init__(name = 'tracker', logger = logger)
        cls_name = ['Vehicle', 'Pedestrian', 'Cyclist', 'Traffic_Cone']
        self.cls_dict = dict()
        for i in range(0, len(cls_name)):
            self.cls_dict[i+1] = cls_name[i]

        self.veh_tracker = AB3DMOT(
            max_age=5, min_hits=2, method='IOU', threshold={True: 0.1, False : 0.01},
            movable=True, score_th=0.25
        )
        self.ped_tracker = AB3DMOT(
            max_age=5, min_hits=4, method='CenterDistance', threshold={True: 0.5, False: 3.0},
            movable=True, score_th=0.15
        )
        self.cyc_tracker = AB3DMOT(
            max_age=5, min_hits=4, method='CenterDistance', threshold={True: 2.0, False: 4.0},
            movable=True, score_th=0.15
        )
        self.con_tracker = AB3DMOT(
            max_age=5, min_hits=4, method='CenterDistance', threshold={True: 0.5, False: 3.0},
            movable=False, score_th=0.15
        )

    def _run_thread(self):
        util.init_backtrace_handle()
        super()._run_thread()

    def process(self, input_dict):
        motion_t = input_dict.pop('motion_t')
        motion_heading = input_dict.pop('motion_heading')
        ins_valid = input_dict.pop('ins_valid')
        timestep = input_dict.pop('timestep') / 1000000.0

        track_dict = prepare_tracking(input_dict, self.cls_dict)
        trackers_veh, veh_traj = self.veh_tracker.update(track_dict['veh'], motion_t, motion_heading, ins_valid, timestep)
        trackers_ped, ped_traj = self.ped_tracker.update(track_dict['ped'], motion_t, motion_heading, ins_valid, timestep)
        trackers_cyc, cyc_traj = self.cyc_tracker.update(track_dict['cyc'], motion_t, motion_heading, ins_valid, timestep)
        trackers_con, con_traj = self.con_tracker.update(track_dict['con'], motion_t, motion_heading, ins_valid, timestep)

        track_dict = concat_trackers(track_dict, trackers_veh, veh_traj)
        track_dict = concat_trackers(track_dict, trackers_ped, ped_traj)
        track_dict = concat_trackers(track_dict, trackers_cyc, cyc_traj)
        track_dict = concat_trackers(track_dict, trackers_con, con_traj)

        return track_dict['pred']
