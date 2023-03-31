from enum import Enum
import numpy as np
import sensor_driver.common_lib.cpp_utils as util

util.init_filters()
class Status(Enum):
    Undefined = 0
    Static    = 1  # (non-movable)
    Stopped   = 2  # (movable)
    Moving    = 3
# ['Vehicle', 'Pedestrian', 'Cyclist', 'Traffic_Cone']
# [  1      ,  2          ,  3       ,  4            ]
class KalmanBoxTracker():
    """
    This class represents the internel state of individual tracked objects observed as bbox.
    """
    idTable = np.arange(255).tolist()
    def __init__(self, bbox3D, label, sensor, movable):
        """
        Initialises a tracker using initial bounding box.
        """
        self.id = KalmanBoxTracker.idTable[0] # New obstacles are given the last used free ID.
        KalmanBoxTracker.idTable.remove(self.id)
        # x,y,z,l,w,h,heading,vx,vy,heading_rate,acc_x,score
        self.x = np.zeros((12,), dtype=np.float32)
        self.x[:7] = bbox3D[:7]
        self.x[11] = bbox3D[7]
        util.use_filter(self.id, False, self.x)

        self.time_since_update = 0
        self.hits = 1           # number of total hits including the first detection
        self.hit_streak = 1     # number of continuing hit considering the first detection
        self.age = 1            # The age of the obstacle (in frames)
        self.valid = True       # True: New valid (detected this frame), False: Older valid
        self.status = Status.Undefined
        self.movable = movable
        self.label = label      # object label
        self.sensor = sensor
        self.set_detection(False)
        self.update_status()

    def update_status(self):
        if self.movable:
            vx = self.x[7]
            vy = self.x[8]
            velocity = np.sqrt((vx * vx) + (vy * vy))
            if velocity > 1.0:
                self.status = Status.Moving
            else:
                self.status = Status.Stopped
        else:
            self.status = Status.Static

    def update(self, bbox3D, label, sensor, ins_valid):
        """
        Updates the state vector with observed bbox.
        """
        # limit the delta heading of 'Vehicle', to stable the prediction trajectory
        dh_threshold = 45.0 if (ins_valid) else 90.0
        if util.filter_update(self.id, bbox3D, dh_threshold) is False:
            self.time_since_update -= 0.5
        else:
            self.x = util.filter_get_x(self.id)

            self.time_since_update = 0
            self.valid = True
            self.hits += 1
            self.hit_streak += 1

        self.label = label
        self.sensor = sensor

        self.update_status()

    def set_detection(self, isGood):
        self.threshold_gate = -0.05 if isGood else 0.05

    def predict(self, motion_t, motion_heading, timestep):
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """

        if self.time_since_update > 2:
            self.hit_streak = 0

        self.time_since_update += 1
        self.valid = False
        self.age += (1 if self.age < 255 else 0)

        self.x = util.filter_predict(self.id, timestep, motion_t, motion_heading)
        return self.x[:7]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        self.score = self.x[11]
        return self.x[:11]

    def motion_prediction(self, ins_valid):
        trajectory = np.zeros([1, 20, 7], dtype=np.float32)

        # only predict the trajectory of moving object
        if not ins_valid or self.status != Status.Moving or self.hit_streak < 10:
            return trajectory

        util.motion_prediction(self.id, trajectory)
        return trajectory

class KalmanStaticBoxTracker():
    def __init__(self, bbox3D, label, sensor, movable):
        self.id = KalmanBoxTracker.idTable[0] # New obstacles are given the last used free ID.
        KalmanBoxTracker.idTable.remove(self.id)
        # x,y,z,l,w,h,heading,score
        self.x = np.zeros((8,), dtype=np.float32)
        self.x = bbox3D
        util.use_filter(self.id, True, self.x)

        self.time_since_update = 0
        self.hits = 1           # number of total hits including the first detection
        self.hit_streak = 1     # number of continuing hit considering the first detection
        self.age = 1            # The age of the obstacle (in frames)
        self.valid = True       # True: New valid (detected this frame), False: Older valid
        self.label = label      # object label
        self.sensor = sensor
        self.set_detection(False)
        self.status = Status.Static

    def update(self, bbox3D, label, sensor, ins_valid):
        """
        Updates the state vector with observed bbox.
        """
        util.filter_update(self.id, bbox3D, 90.0)
        self.x = util.filter_get_x(self.id)

        self.time_since_update = 0
        self.valid = True
        self.hits += 1
        self.hit_streak += 1          # number of continuing hit
        self.label = label
        self.sensor = sensor

    def set_detection(self, isGood):
        self.threshold_gate = -0.05 if isGood else 0.05

    def predict(self, motion_t, motion_heading, timestep):
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """

        if self.time_since_update > 2:
            self.hit_streak = 0

        self.time_since_update += 1
        self.valid = False
        self.age += (1 if self.age < 255 else 0)

        self.x = util.filter_predict(self.id, timestep, motion_t, motion_heading)
        return self.x[:7]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        self.score = self.x[7]
        state = np.zeros((11, ), dtype=np.float32)
        state[:7] = self.x[:7]
        return state

    def motion_prediction(self, ins_valid):
        trajectory = np.zeros([1, 20, 7], dtype=np.float32)
        return trajectory