from scipy.spatial.transform import Rotation as R
import numpy as np


def xyzrpy_to_rt(xyzrpy: list):
    roll, pitch, yaw = xyzrpy[3:]
    rot = R.from_euler('ZXY', [yaw, pitch, roll], degrees=True).as_matrix()
    return rot, np.array(xyzrpy[:3])


def xyzrpy_to_transform(xyzrpy: list):
    roll, pitch, yaw = xyzrpy[3:]
    rot = R.from_euler('ZXY', [yaw, pitch, roll], degrees=True).as_matrix()
    T = np.eye(4, 4)
    T[:3, :3] = rot
    T[:3, -1] = np.array(xyzrpy[:3])
    return T


def rt_to_xyzrpy(rot, t):
    yaw, pitch, roll = R.from_matrix(rot).as_euler('ZXY', degrees=True)
    return list(t) + [roll, pitch, yaw]