import numpy as np
import sensor_driver.common_lib.cpp_utils as util
import sensor_driver.common_lib.iou3d_nms as iou3d_nms_utils
from sensor_fusion.AB3DMOT.kalman_filter import KalmanBoxTracker, KalmanStaticBoxTracker

def associate_detections_to_trackers(detections, trackers, method, threshold):
    from scipy.optimize import linear_sum_assignment
    """
    Assigns detections to tracked object (both represented as bounding boxes)
    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if (len(trackers)==0):
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 8, 3), dtype=int)
    if (len(detections)==0):
        return np.empty((0, 2), dtype=int), np.empty((0, 8, 3), dtype=int), np.arange(len(trackers))

    if method == 'IOU':
        iou_matrix = iou3d_nms_utils.boxes_bev_iou_cpu(detections, trackers)
    elif method == 'CenterDistance':
        threshold = -1 * threshold
        iou_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)
        util.get_distance_matrix(detections, trackers, iou_matrix)

    iou_matrix[np.isneginf(-iou_matrix) | np.isnan(-iou_matrix)] = -1e4
    row_ind, col_ind = linear_sum_assignment(-iou_matrix)      # hougarian algorithm
    matched_indices = np.stack((row_ind, col_ind), axis=1)

    return util.get_association(len(detections), len(trackers), matched_indices, threshold, iou_matrix)

class AB3DMOT(object):
    def __init__(self, max_age=2, min_hits=3, method='IOU', threshold=None, movable=None, score_th=None):
        """
        Sets key parameters for SORT
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.trackers = np.array([])
        self.method = method
        self.threshold = threshold
        self.movable = movable
        self.score_th = score_th

    def update(self, dets_all, motion_t, motion_heading, ins_valid, timestep):
        dets, label, sensor = dets_all['dets'], dets_all['label'], dets_all['sensor']

        trks = np.zeros((len(self.trackers), 7), dtype=np.float32)         # N x 7 , # get predicted locations from existing trackers.
        for t, trk in enumerate(trks):
            trks[t, :] = self.trackers[t].predict(motion_t, motion_heading, timestep)

        threshold = self.threshold[ins_valid]
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets[:, :7], trks, self.method, threshold)

        # update matched trackers with assigned detections
        for t, trk in enumerate(self.trackers):
            if t not in unmatched_trks:
                d = matched[np.where(matched[:, 1] == t)[0], 0]     # a list of index
                trk.update(dets[d, :][0], label[d, :][0], sensor[d, :][0], ins_valid)

        # create and initialise new trackers for unmatched detections (max 256 objects)
        if len(KalmanBoxTracker.idTable) >= len(unmatched_dets):
            if self.movable:
                new_trts = [KalmanBoxTracker(dets[i, :], label[i, :], sensor[i, :], self.movable) for i in unmatched_dets]
            else:
                new_trts = [KalmanStaticBoxTracker(dets[i, :], label[i, :], sensor[i, :], self.movable) for i in unmatched_dets]
            self.trackers = np.concatenate((self.trackers, new_trts), axis=0)

        trks_valid_mask = np.ones(len(self.trackers), dtype=np.bool)
        ret, trajectorys = [], []
        for t, trk in enumerate(self.trackers):
            d = trk.get_state()
            trajectory = trk.motion_prediction(ins_valid)

            if ((trk.hits >= self.min_hits) and (trk.hit_streak >= 2) and (trk.score > (self.score_th + trk.threshold_gate))):
                trk.set_detection(True)
                ret.append(np.concatenate((d, [trk.id, trk.score], trk.label, [trk.age, trk.valid, trk.status.value], trk.sensor)).reshape(1, -1)) # +1 as MOT benchmark requires positive
                trajectorys.append(trajectory)
            else:
                trk.set_detection(False)

            # remove dead tracklet
            if (trk.time_since_update >= self.max_age):
                KalmanBoxTracker.idTable.append(trk.id)
                trks_valid_mask[t] = False
        self.trackers = self.trackers[trks_valid_mask]
        if (len(ret) > 0): return np.concatenate(ret), np.concatenate(trajectorys) 			# x,y,z,w,h,l,theta,vx,vy,heading_rate,ax ID confidence label, age, valid, status
        return np.empty((0, 18)), np.empty((0, 20, 7))


class PassThrough(object):
    def __init__(self, max_age=2, min_hits=3, method='IOU', threshold=None, movable=None, score_th=None):
        pass

    def update(self, dets_all, motion_t, motion_heading, ins_valid, timestep):
        dets, label, sensor = dets_all['dets'], dets_all['label'], dets_all['sensor']

        ret, trajectorys = [], []
        for t, det in enumerate(dets):
            trajectory = np.zeros([1, 20, 7], dtype=np.float32)

            ret.append(np.concatenate((det[:7], [0, 0, 0, 0, 0, det[7]], label[t], [0, False, 0], sensor[t])).reshape(1, -1))
            trajectorys.append(trajectory)

        if (len(ret) > 0): return np.concatenate(ret), np.concatenate(trajectorys) 			# x,y,z,w,h,l,theta,vx,vy,heading_rate,ax ID confidence label, age, valid, status
        return np.empty((0, 18)), np.empty((0, 20, 7))