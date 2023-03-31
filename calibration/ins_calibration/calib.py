import numpy as np
from module.export_interface import call
from proto import detection_pb2
from sensor_driver.calibration.calib_driver import (
    calib_ins_reset,
    calib_ins_add_points,
    calib_ins_add_odom,
    calib_ins_calibrate,
    calib_ins_get_calibration,
    calib_ins_get_calibration_transform,
)

from sensor_driver.common_lib.cpp_utils import (
    get_cfg_from_transform,
    get_transform_from_cfg,
    get_transform_from_rtk,
)

class InsCalib():
    is_origin_set = False
    origin = None
    last_position = None
    last_rotation = None
    staticTransform = np.eye(4)
    def __init__(self):
        pass

    @staticmethod
    def reset(extrinsic_parameters):
        InsCalib.is_origin_set = False
        InsCalib.origin = None
        InsCalib.last_position = None
        InsCalib.last_rotation = None
        InsCalib.staticTransform = get_transform_from_cfg(*extrinsic_parameters)
        calib_ins_reset(InsCalib.staticTransform)

    @staticmethod
    def parse_data(data_dict):
        latitude = data_dict['latitude']
        longitude = data_dict['longitude']
        altitude = data_dict['altitude']
        heading = data_dict['heading']
        pitch = data_dict['pitch']
        roll = data_dict['roll']
        return np.array([latitude, longitude, altitude, heading, pitch, roll])

    @staticmethod
    def set_origin(position):
        if not InsCalib.is_origin_set:
            InsCalib.is_origin_set = True
            InsCalib.origin = position
            InsCalib.last_position = np.array([0, 0, 0])
            InsCalib.last_rotation = position[3:]
            return True
        else:
            return False

    @staticmethod
    def is_keyframe(position, rotation):
        dist = np.linalg.norm(position - InsCalib.last_position, ord=2)
        rota = np.linalg.norm(rotation - InsCalib.last_rotation, ord=2)
        if dist > 5.0 or rota > 30.0:
            InsCalib.last_position = position
            InsCalib.last_rotation = rotation
            return True
        else:
            return False

    @staticmethod
    def getPositionPoints():
        det = detection_pb2.Detection()
        result_dict = call('sink.get_proto_http', raw_data = True)

        # points
        if 'points' in result_dict and result_dict['lidar_valid']:
            points = np.concatenate(list(result_dict['points'].values()), axis=0)
            det.points = points.tobytes()
        else:
            points = np.zeros((0, 4), dtype=np.float32)
            det.points = points.tobytes()

        if 'ins_data' in result_dict and result_dict['ins_valid']:
            position = InsCalib.parse_data(result_dict['ins_data'])
            InsCalib.set_origin(position)

            T = get_transform_from_rtk(*[*InsCalib.origin, *position])

            pose = get_cfg_from_transform(T)
            det.pose.x = pose[0]
            det.pose.y = pose[1]
            det.pose.z = pose[2]
            det.pose.heading = pose[3]
            det.pose.pitch = pose[4]
            det.pose.roll = pose[5]

            isKeyFrame = InsCalib.is_keyframe(T[:3, 3], position[3:])
            det.pose.status = 1 if isKeyFrame else 0

            if isKeyFrame:
                calib_ins_add_points(points, result_dict['frame_start_timestamp'])
            calib_ins_add_odom(T, result_dict['ins_data']['timestamp'])

        return det.SerializeToString()

    @staticmethod
    def calibration():
        calib_ins_calibrate()

    @staticmethod
    def getCalibration():
        return calib_ins_get_calibration()

    @staticmethod
    def getTransform():
        return calib_ins_get_calibration_transform()