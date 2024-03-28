import numpy as np
import sensor_driver.common_lib.cpp_utils as util
import sensor_driver.inference.iou3d_nms as iou3d_nms_utils
from sensor_fusion.MOT3D.tracklet import IDTable, BoxTracker, StaticBoxTracker

def associate_detections_to_trackers(iou_matrix, detections, trackers, threshold):
    from scipy.optimize import linear_sum_assignment
    """
    Assigns detections to tracked object (both represented as bounding boxes)
    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if (len(trackers)==0):
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 8, 3), dtype=int)
    if (len(detections)==0):
        return np.empty((0, 2), dtype=int), np.empty((0, 8, 3), dtype=int), np.arange(len(trackers))

    row_ind, col_ind = linear_sum_assignment(iou_matrix)      # hougarian algorithm
    matched_indices = np.stack((row_ind, col_ind), axis=1)

    return util.get_association(len(detections), len(trackers), matched_indices, threshold, iou_matrix)

class MOT3D(object):
    def __init__(self, logger, config):
        self.logger = logger
        self.config = config
        self.motion_streak = 0
        self.Tracklet = BoxTracker if self.config['movable'] else StaticBoxTracker
        self.trackers = np.array([])

    @staticmethod
    def reset():
        IDTable.reset()

    def update(self, dets_all, motion_t, motion_heading, motion_valid, timestep):
        self.motion_streak = self.motion_streak + 1 if motion_valid else 0
        detections = dets_all['detections']

        # get predicted locations from existing trackers
        trks = np.zeros((len(self.trackers), 7), dtype=np.float32)
        for t, trk in enumerate(trks):
            trks[t, :] = self.trackers[t].predict(motion_t, motion_heading, timestep)

        # calculate the GIOU matrix
        iou_matrix = iou3d_nms_utils.boxes_giou3d_gpu(detections[:, :7], trks)
        iou_matrix = 1.0 - iou_matrix

        # split the detections by score
        mask = detections[:, 7] > self.config['score_th']
        detections = detections[mask]
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(iou_matrix[mask], detections, trks, self.config['giou_th'])

        # update matched trackers with assigned detections
        for t, trk in enumerate(self.trackers):
            if t not in unmatched_trks:
                d = matched[np.where(matched[:, 1] == t)[0], 0]
                trk.update(detections[d, :][0])
            elif iou_matrix.shape[0] > 0:
                d = np.argmin(iou_matrix[:, t])
                if iou_matrix[d, t] <= self.config['giou_th']:
                    trk.update()
            trk.update_state()

        # create and initialize new trackers for unmatched detections
        new_trts = [self.Tracklet(detections[i], self.config) for i in unmatched_dets]
        self.trackers = np.concatenate((self.trackers, new_trts), axis=0)

        # output and remove dead tracks
        trks_valid_mask = np.ones(len(self.trackers), dtype=np.bool)
        ret, trajectorys = [], []
        for t, trk in enumerate(self.trackers):
            trajectory = trk.motion_prediction((motion_valid and self.motion_streak > 10))

            if trk.is_valid():
                ret.append(np.concatenate((trk.get_state(), [trk.id, trk.score, trk.label, trk.age, trk.valid, trk.status.value])).reshape(1, -1)) # +1 as MOT benchmark requires positive
                trajectorys.append(trajectory)
            if trk.is_dead():
                IDTable.return_id(trk.id)
                trks_valid_mask[t] = False

        self.trackers = self.trackers[trks_valid_mask]
        if (len(ret) > 0): return np.concatenate(ret), np.concatenate(trajectorys) 			# x,y,z,l,w,h,theta,vx,vy,gyro,ax,id,score,label,age,valid,status
        return np.empty((0, 17)), np.empty((0, 20, 7))


class PassThrough(object):
    def __init__(self, logger, config):
        self.logger = logger
        self.config = config

    def update(self, dets_all, motion_t, motion_heading, motion_valid, timestep):
        detections = dets_all['detections']

        ret, trajectorys = [], []
        for t, det in enumerate(detections):
            ret.append(np.concatenate((det[:7], [0, 0, 0, 0, 0, det[7], det[8], 0, False, 0])).reshape(1, -1))
            trajectorys.append(np.zeros([1, 20, 7], dtype=np.float32))

        if (len(ret) > 0): return np.concatenate(ret), np.concatenate(trajectorys) 			# x,y,z,l,w,h,theta,vx,vy,gyro,ax,id,score,label,age,valid,status
        return np.empty((0, 17)), np.empty((0, 20, 7))