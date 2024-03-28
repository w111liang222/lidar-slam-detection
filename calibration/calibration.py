import numpy as np

from module.export_interface import register_interface, call
from sensor_driver.common_lib.cpp_utils import (
    get_cfg_from_transform,
    get_transform_from_cfg,
    get_projection_forward,
    get_projection_backward,
)

def get_transform(extrinsic_parameters):
    return get_transform_from_cfg(*extrinsic_parameters).flatten().tolist()

def get_vector_from_transform(transform):
    return [ float(x) for x in get_cfg_from_transform(np.array(transform).reshape(4, 4)) ]

def finetune_lidar(config, lidarIndex, transform):
    T = get_transform_from_cfg(*config['lidar'][lidarIndex]['extrinsic_parameters'])
    dT = np.array(transform).reshape(4, 4)
    T = dT.dot(T)
    config['lidar'][lidarIndex]['extrinsic_parameters'] = [
        float(x) for x in get_cfg_from_transform(T)
    ]
    return dT.flatten().tolist(), config

def calibrate_ground(config, points, contour, key):
    import base64
    from calibration.lidar_calibration import align_points_to_xyplane

    T = get_transform_from_cfg(*config['lidar'][int(key)]['extrinsic_parameters'])
    points = np.frombuffer(base64.b64decode(points), dtype=np.float32)
    dT = align_points_to_xyplane(points, contour)
    T = dT.dot(T)
    config['lidar'][int(key)]['extrinsic_parameters'] = [
        float(x) for x in get_cfg_from_transform(T)
    ]
    return dT.flatten().tolist(), config

def calibrate_heading(config, source, target, key):
    from calibration.lidar_calibration import align_points
    T = get_transform_from_cfg(*config['lidar'][int(key)]['extrinsic_parameters'])
    dT = align_points(source, target)
    T = dT.dot(T)
    config['lidar'][int(key)]['extrinsic_parameters'] = [
        float(x) for x in get_cfg_from_transform(T)
    ]
    return dT.flatten().tolist(), config

def get_camera_index(config, cameraName):
    targetIndex = None
    for i, camCfg in enumerate(config['camera']):
        if camCfg['name'] == cameraName:
            targetIndex = i
            break
    if targetIndex is None:
        print("No such camera: " + cameraName)
    return targetIndex

def finetune_camera(config, cameraName, transform):
    targetIndex = get_camera_index(config, cameraName)
    if targetIndex is None:
        return np.eye(4, 4).flatten().tolist(), config

    T = get_transform_from_cfg(*config['camera'][targetIndex]['extrinsic_parameters'])
    dT = np.array(transform).reshape(4, 4)
    T = dT.dot(T)
    config['camera'][targetIndex]['extrinsic_parameters'] = [
        float(x) for x in get_cfg_from_transform(T)
    ]
    return T.flatten().tolist(), config

def calibrate_lidar_camera(config, pointsLidar, pointsCamera, cameraName):
    from calibration.lidar_camera import align_lidar_camera
    targetIndex = get_camera_index(config, cameraName)
    if targetIndex is None:
        return np.eye(4, 4).flatten().tolist(), config
    try:
        T = align_lidar_camera(config['camera'][targetIndex], pointsLidar, pointsCamera)
        config['camera'][targetIndex]['extrinsic_parameters'] = [
            float(x) for x in get_cfg_from_transform(T)
        ]
    except Exception as e:
        T = np.eye(4, 4)
        print(e)
    return T.flatten().tolist(), config

def find_corners(imageData, cameraName, config):
    from calibration.camera_calibration import CameraCalib
    return CameraCalib.detect_checkborad(imageData, cameraName, config)

def calibrate_camera(od_config, pointsCamera, cameraName, config):
    from calibration.camera_calibration import CameraCalib
    targetIndex = get_camera_index(od_config, cameraName)
    if targetIndex is None:
        return {"result": False}, od_config

    intrinsic = CameraCalib.calibration(pointsCamera, od_config['camera'][targetIndex], config)
    od_config['camera'][targetIndex]['intrinsic_parameters'] = [
        float(x) for x in intrinsic
    ]
    return {"result": True}, od_config

def get_calibrate_camera(config, do_distort):
    from util.image_util import undistort_image
    from proto.proto_serialize import serialize_to_string
    camera_config = dict()
    for idx, cfg in enumerate(config['camera']):
        camera_config[cfg['name']] = cfg
    data_dict = call('bank.get_frame_data')
    if do_distort == True:
        for name, img in data_dict['image'].items():
            data_dict['image'][name] = undistort_image(img, camera_config[name])

    return serialize_to_string(data_dict, use_raw_image=True)

def restart_lidar_ins_calibration(config):
    from calibration.ins_calibration import InsCalib
    InsCalib.reset(config['ins']['extrinsic_parameters'])

def get_position_points():
    from calibration.ins_calibration import InsCalib
    return InsCalib.getPositionPoints()

def calibrate_lidar_ins():
    from calibration.ins_calibration import InsCalib
    InsCalib.calibration()
    return {"result": True}

def get_lidar_ins_calibration():
    from calibration.ins_calibration import InsCalib
    return InsCalib.getCalibration()

def get_lidar_ins_transform():
    from calibration.ins_calibration import InsCalib
    return InsCalib.getTransform().flatten().tolist()

def set_lidar_ins_transform(config, transform):
    from calibration.ins_calibration import InsCalib
    T = np.array(transform).reshape(4, 4)
    config['ins']['extrinsic_parameters'] = [float(x) for x in get_cfg_from_transform(T)]
    InsCalib.reset(config['ins']['extrinsic_parameters'])
    return {"result": True}, config

def restart_lidar_imu_calibration(config):
    from calibration.imu_calibration import ImuCalib
    ImuCalib.reset(config['ins']['extrinsic_parameters'])

def get_imu_position_points(config):
    from calibration.imu_calibration import ImuCalib
    return ImuCalib.getPositionPoints(config)

def calibrate_lidar_imu():
    from calibration.imu_calibration import ImuCalib
    ImuCalib.calibration()
    return {"result": True}

def lidar_imu_get_lidar_poses():
    from calibration.imu_calibration import ImuCalib
    return ImuCalib.getLidarPoses()

def lidar_imu_get_imu_poses():
    from calibration.imu_calibration import ImuCalib
    return ImuCalib.getImuPoses()

def set_lidar_imu_extrinsics(config):
    from calibration.imu_calibration import ImuCalib
    T = ImuCalib.getTransform()
    config['ins']['imu_extrinsic_parameters'] = [float(x) for x in get_cfg_from_transform(T)]
    return config

def get_homography(cameras, name0, name1, image0, image1, kpoint0, kpoint1, order):
    from calibration.panorama_camera import PanoramaCameraCalib
    return PanoramaCameraCalib.get_homography(cameras, name0, name1, image0, image1, kpoint0, kpoint1, order)

def get_panorama():
    from calibration.panorama_camera import PanoramaCameraCalib
    return PanoramaCameraCalib.get_panorama()

def set_panorama_config(config):
    from calibration.panorama_camera import PanoramaCameraCalib
    panorama_config = PanoramaCameraCalib.get_panorama_config()
    config['panorama_camera']['sensor_input'] = panorama_config['cameras']
    config['panorama_camera']['parameters'] = panorama_config['parameters']
    return {"result": True}, config

register_interface('calibration.get_projection_forward', get_projection_forward)
register_interface('calibration.get_projection_backward', get_projection_backward)
register_interface('calibration.get_transform', get_transform)
register_interface('calibration.get_vector_from_transform', get_vector_from_transform)

# lidar calibration
register_interface('calibration.finetune_lidar', finetune_lidar)
register_interface('calibration.calibrate_ground', calibrate_ground)
register_interface('calibration.calibrate_heading', calibrate_heading)

# lidar - camera calibration
register_interface('calibration.finetune_camera', finetune_camera)
register_interface('calibration.calibrate_lidar_camera', calibrate_lidar_camera)

# camera calibration
register_interface('calibration.find_corners', find_corners)
register_interface('calibration.calibrate_camera', calibrate_camera)
register_interface('calibration.get_calibrate_camera', get_calibrate_camera)

# ins calibration
register_interface('calibration.restart_lidar_ins_calibration', restart_lidar_ins_calibration)
register_interface('calibration.get_position_points', get_position_points)
register_interface('calibration.calibrate_lidar_ins', calibrate_lidar_ins)
register_interface('calibration.get_lidar_ins_calibration', get_lidar_ins_calibration)
register_interface('calibration.get_lidar_ins_transform', get_lidar_ins_transform)
register_interface('calibration.set_lidar_ins_transform', set_lidar_ins_transform)

# imu calibration
register_interface('calibration.restart_lidar_imu_calibration', restart_lidar_imu_calibration)
register_interface('calibration.get_imu_position_points', get_imu_position_points)
register_interface('calibration.calibrate_lidar_imu', calibrate_lidar_imu)
register_interface('calibration.lidar_imu_get_lidar_poses', lidar_imu_get_lidar_poses)
register_interface('calibration.lidar_imu_get_imu_poses', lidar_imu_get_imu_poses)
register_interface('calibration.set_lidar_imu_extrinsics', set_lidar_imu_extrinsics)

# panorama camera calibration
register_interface('calibration.get_homography', get_homography)
register_interface('calibration.get_panorama', get_panorama)
register_interface('calibration.set_panorama_config', set_panorama_config)