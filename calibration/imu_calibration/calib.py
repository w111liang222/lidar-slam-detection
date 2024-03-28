import time
import numpy as np

from module.export_interface import call
from proto import detection_pb2
from sensor_driver.calibration.calib_driver import (
    calib_imu_reset,
    calib_imu_add_points,
    calib_imu_add_imu,
    calib_imu_calibrate,
    calib_imu_get_lidar_poses,
    calib_imu_get_imu_poses,
    calib_imu_get_lidar_T_R,
)

from sensor_driver.common_lib.cpp_utils import (
    get_cfg_from_transform,
    get_transform_from_cfg,
    get_transform_from_rtk,
)

class ImuCalib():
    is_origin_set = False
    origin = None
    last_stamp = 0
    last_pose = None
    last_imu_data = None
    last_position = None
    last_rotation = None
    staticTransform = np.eye(4)
    imu_transform = np.eye(4)
    def __init__(self):
        pass

    @staticmethod
    def reset(extrinsic_parameters=None):
        ImuCalib.is_origin_set = False
        ImuCalib.origin = None
        ImuCalib.last_stamp = 0
        ImuCalib.last_pose = None
        ImuCalib.last_imu_data = None
        ImuCalib.last_position = None
        ImuCalib.last_rotation = None
        if extrinsic_parameters is not None:
            ImuCalib.staticTransform = get_transform_from_cfg(*extrinsic_parameters)
        ImuCalib.imu_transform = np.eye(4)
        calib_imu_reset()

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
        if not ImuCalib.is_origin_set:
            ImuCalib.is_origin_set = True
            ImuCalib.origin = position
            return True
        else:
            return False

    @staticmethod
    def is_keyframe(position, rotation):
        if ImuCalib.last_position is None or ImuCalib.last_rotation is None:
            ImuCalib.last_position = position
            ImuCalib.last_rotation = rotation
            return False

        dist = np.linalg.norm(position - ImuCalib.last_position, ord=2)
        rota = np.linalg.norm(rotation - ImuCalib.last_rotation, ord=2)
        if dist > 2.0 or rota > 5.0:
            ImuCalib.last_position = position
            ImuCalib.last_rotation = rotation
            return True
        else:
            return False

    @staticmethod
    def getPositionPoints(config):
        det = detection_pb2.Detection()
        retry = 0
        while retry < 100:
            retry = retry + 1
            result_dict = call('bank.get_frame_data')
            if 'frame_start_timestamp' in result_dict and result_dict['frame_start_timestamp'] != ImuCalib.last_stamp:
                ImuCalib.last_stamp = result_dict['frame_start_timestamp']
                break
            time.sleep(0.01)

        if 'points' in result_dict and result_dict['lidar_valid']:
            points = np.concatenate(list(result_dict['points'].values()), axis=0)
            det.points = points.tobytes()
        else:
            points = np.zeros((0, 4), dtype=np.float32)
            det.points = points.tobytes()

        det.pose.x, det.pose.y, det.pose.z = 0, 0, 0
        det.pose.heading, det.pose.pitch, det.pose.roll = 0, 0, 0

        if points.shape[0] <= 0 or 'ins_data' not in result_dict or not result_dict['ins_valid']:
            print('Lidar / Ins / Imu data is invalid, wait for reset')
            det.pose.status = 2
            return det.SerializeToString()

        if ImuCalib.last_imu_data is not None:
            imu_diff_time = abs(result_dict['imu_data'][0, 0] - ImuCalib.last_imu_data[-1, 0])
        else:
            imu_diff_time = 0
        ImuCalib.last_imu_data = result_dict['imu_data']

        # check continous of imu data
        if imu_diff_time >= 1000000 or result_dict['imu_data'].shape[0] < 2:
            print('Imu data is non-continous, time diff {}, size {}'.format(imu_diff_time, result_dict['imu_data'].shape[0]))
            det.pose.status = 2
            return det.SerializeToString()

        position = ImuCalib.parse_data(result_dict['ins_data'])

        # judge whether gps latitude and longitude is zero
        if config['ins']['ins_type'] != '6D' or (result_dict['ins_data']['latitude'] == 0 and result_dict['ins_data']['longitude'] == 0):
            T = calib_imu_get_lidar_T_R(points)
        else:
            ImuCalib.set_origin(position)
            T = get_transform_from_rtk(*[*ImuCalib.origin, *position])
            T = T.dot(ImuCalib.staticTransform)

        pose = get_cfg_from_transform(T)
        det.pose.x, det.pose.y, det.pose.z = pose[0], pose[1], pose[2]
        det.pose.heading, det.pose.pitch, det.pose.roll = pose[3], pose[4], pose[5]

        isKeyFrame = ImuCalib.is_keyframe(np.array(pose[:3]), np.array(pose[3:]))
        det.pose.status = 1 if isKeyFrame else 0

        if isKeyFrame:
            if ImuCalib.last_pose is None:
                ImuCalib.last_pose = T
            delta_pose = np.linalg.inv(ImuCalib.last_pose).dot(T)
            ImuCalib.last_pose = T
            calib_imu_add_points(points, result_dict['frame_start_timestamp'], delta_pose, T)
        calib_imu_add_imu(result_dict['imu_data'])

        return det.SerializeToString()

    @staticmethod
    def calibration():
        ImuCalib.imu_transform[:3, :3] = calib_imu_calibrate()
        ImuCalib.evaluation()

    @staticmethod
    def getLidarPoses():
        poses = calib_imu_get_lidar_poses()
        vertexs = dict()
        for i in range(len(poses)):
            vertexs[str(i)] = poses[i].flatten().tolist()
        return vertexs

    @staticmethod
    def getImuPoses():
        poses = calib_imu_get_imu_poses()
        vertexs = dict()
        for i in range(len(poses)):
            vertexs[str(i)] = poses[i].flatten().tolist()
        return vertexs

    @staticmethod
    def getTransform():
        return ImuCalib.imu_transform

    @staticmethod
    def evaluation():
        lidar_poses = calib_imu_get_lidar_poses()
        imu_poses = calib_imu_get_imu_poses() 

        dist_error = 0
        rota_error = 0
        for i in range(len(lidar_poses)):
            lidar_pose = get_cfg_from_transform(lidar_poses[i])
            imu_pose = get_cfg_from_transform(imu_poses[i])
            each_dist_error = np.linalg.norm(np.array(lidar_pose[:2]) - np.array(imu_pose[:2]), ord = 2)
            each_rota_error = np.linalg.norm(np.array(lidar_pose[3:]) - np.array(imu_pose[3:]), ord = 2)
            dist_error += each_dist_error
            rota_error += each_rota_error
        mean_dist_error = (dist_error/len(lidar_poses))**0.5
        mean_rota_error = (rota_error/len(lidar_poses))**0.5

        print('the mean distance error of lidar and imu calib is:', mean_dist_error)
        print('the mean rotation error of lidar and imu calib is:', mean_rota_error)