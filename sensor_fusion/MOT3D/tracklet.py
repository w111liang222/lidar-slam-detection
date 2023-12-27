from enum import Enum
import numpy as np
import sensor_driver.common_lib.cpp_utils as util

class Status(Enum):
    Undefined = 0
    Static    = 1  # (non-movable)
    Stopped   = 2  # (movable)
    Moving    = 3

class State(Enum):
    Birth     = 0
    Alive     = 1
    Dead      = 2

# class map
# ['Vehicle', 'Pedestrian', 'Cyclist', 'Traffic_Cone']
# [  1      ,  2          ,  3       ,  4            ]

class IDTable:
    IDs = []

    @staticmethod
    def reset():
        IDTable.IDs = np.arange(65535).tolist()

    @staticmethod
    def get_id():
        return IDTable.IDs.pop(0)

    @staticmethod
    def return_id(id):
        IDTable.IDs.append(id)

class BoxTracker():
    """
    This class represents the internel state of individual tracked objects observed as bbox.
    """
    def __init__(self, bbox3D, config):
        """
        Initialises a tracker using initial bounding box.
        """
        self.config = config

        self.id    = IDTable.get_id()                   # New obstacles are given the last used free ID.
        self.x     = np.zeros((11,), dtype=np.float32)  # x,y,z,l,w,h,heading,vx,vy,gyro,acc_x
        self.x[:7] = bbox3D[:7]
        self.score = bbox3D[7]
        self.label = bbox3D[8]
        util.use_filter(self.id, False, self.x)

        self.time_since_update = 0
        self.hits = 1                                   # number of total hits including the first detection
        self.hit_streak = 1                             # number of continuing hit considering the first detection
        self.age = 1                                    # The age of the obstacle (in frames)
        self.valid = True                               # True: New valid (detected this frame), False: Older valid
        self.status = Status.Undefined
        self.state = State.Birth
        self.update_status()

    def update_status(self):
        vx = self.x[7]
        vy = self.x[8]
        velocity = np.sqrt((vx * vx) + (vy * vy))
        self.status = Status.Moving if velocity > 1.0 else Status.Stopped

    def update_state(self):
        if self.state == State.Birth and self.hits >= self.config['min_hits']:
            self.state = State.Alive
        if self.time_since_update >= self.config['max_age']:
            self.state = State.Dead

    def update(self, bbox3D = None):
        """
        Updates the state vector with observed bbox.
        """
        if bbox3D is not None:
            self.score = self.score * 0.8 + bbox3D[7] * 0.2
            self.x = util.filter_update(self.id, bbox3D[:7])
        else:
            self.score = max(self.score - 0.05, 0)

        self.time_since_update = 0
        self.valid = True
        self.hits += 1
        self.hit_streak += 1
        self.update_status()

    def predict(self, motion_t, motion_heading, timestep):
        # predict based on motion model
        self.x = util.filter_predict(self.id, timestep, motion_t, motion_heading)

        # life manager
        self.age = min(self.age + 1, 255)
        if self.time_since_update > 0:
            self.hit_streak = 0

        self.time_since_update += 1
        self.valid = False
        return self.x[:7]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        return self.x

    def is_valid(self):
        valid = (self.state == State.Alive) and (self.hit_streak >= 1) and (self.score > (self.config['score_th'] + self.score_gate))
        self.score_gate = -0.1 if valid else 0.1
        return valid

    def is_dead(self):
        return (self.state == State.Dead)

    def motion_prediction(self, motion_valid):
        trajectory = np.zeros([1, 20, 7], dtype=np.float32)

        valid = (motion_valid) and (self.status == Status.Moving) and (self.score > (self.config['score_th'] + self.motion_gate))
        self.motion_gate = 0.0 if valid else 0.2

        if not valid:
            return trajectory

        util.motion_prediction(self.id, trajectory)
        return trajectory

class StaticBoxTracker():
    def __init__(self, bbox3D, config):
        self.config = config

        self.id    = IDTable.get_id()                   # New obstacles are given the last used free ID.
        self.x     = np.zeros((7,), dtype=np.float32)   # x,y,z,l,w,h,heading
        self.x[:7] = bbox3D[:7]
        self.score = bbox3D[7]
        self.label = bbox3D[8]
        util.use_filter(self.id, True, self.x)

        self.time_since_update = 0
        self.hits = 1                                   # number of total hits including the first detection
        self.hit_streak = 1                             # number of continuing hit considering the first detection
        self.age = 1                                    # The age of the obstacle (in frames)
        self.valid = True                               # True: New valid (detected this frame), False: Older valid
        self.status = Status.Static
        self.state = State.Birth

    def update_state(self):
        if self.state == State.Birth and self.hits >= self.config['min_hits']:
            self.state = State.Alive
        if self.time_since_update >= self.config['max_age']:
            self.state = State.Dead

    def update(self, bbox3D = None):
        """
        Updates the state vector with observed bbox.
        """
        if bbox3D is not None:
            self.score = self.score * 0.8 + bbox3D[7] * 0.2
            self.x = util.filter_update(self.id, bbox3D[:7])
        else:
            self.score = max(self.score - 0.05, 0)

        self.time_since_update = 0
        self.valid = True
        self.hits += 1
        self.hit_streak += 1

    def predict(self, motion_t, motion_heading, timestep):
        # predict based on motion model
        self.x = util.filter_predict(self.id, timestep, motion_t, motion_heading)

        # life manager
        self.age = min(self.age + 1, 255)
        if self.time_since_update > 0:
            self.hit_streak = 0

        self.time_since_update += 1
        self.valid = False
        return self.x[:7]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        state = np.zeros((11, ), dtype=np.float32)
        state[:7] = self.x[:7]
        return state

    def is_valid(self):
        valid = (self.state == State.Alive) and (self.hit_streak >= 1) and (self.score > (self.config['score_th'] + self.score_gate))
        self.score_gate = -0.1 if valid else 0.05
        return valid

    def is_dead(self):
        return (self.state == State.Dead)

    def motion_prediction(self, motion_valid):
        trajectory = np.zeros([1, 20, 7], dtype=np.float32)
        return trajectory