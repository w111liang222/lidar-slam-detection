
from flask import request
from flask.helpers import make_response
from jsonrpc.backend.flask import api

add_method = api.dispatcher.add_method

class CalibrationServer:
    def __init__(self, app):
        self.app = app

    def setup(self, perception):
        self.perception = perception
        # calibration
        add_method(self.get_projection_forward, name='get_projection_forward')
        add_method(self.get_projection_backward, name='get_projection_backward')
        add_method(self.get_transform, name='get_transform')
        add_method(self.get_vector_from_transform, name='get_vector_from_transform')
        # lidar calibration
        add_method(self.finetune_lidar, name='finetune_lidar')
        add_method(self.calibrate_ground, name='calibrate_ground')
        add_method(self.calibrate_heading, name='calibrate_heading')
        # lidar - camera calibration
        add_method(self.finetune_camera, name='finetune_camera')
        add_method(self.calibrate_lidar_camera, name='calibrate_lidar_camera')
        # camera calibration
        add_method(self.find_corners, name='find_corners')
        add_method(self.calibrate_camera, name='calibrate_camera')
        self.app.add_url_rule("/v1/source-data", view_func=self.get_source_data, methods=["POST"])
        # ins calibration
        add_method(self.restart_lidar_ins_calibration, name='restart_lidar_ins_calibration')
        self.app.add_url_rule("/v1/get-position-points", view_func=self.get_position_points, methods=["GET"])
        add_method(self.calibrate_lidar_ins, name='calibrate_lidar_ins')
        add_method(self.get_lidar_ins_calibration, name='get_lidar_ins_calibration')
        add_method(self.get_lidar_ins_transform, name='get_lidar_ins_transform')
        add_method(self.set_lidar_ins_transform, name='set_lidar_ins_transform')
        # imu calibration
        add_method(self.restart_lidar_imu_calibration, name='restart_lidar_imu_calibration')
        self.app.add_url_rule("/v1/get-imu-position-points", view_func=self.get_imu_position_points, methods=["GET"])
        add_method(self.calibrate_lidar_imu, name='calibrate_lidar_imu')
        add_method(self.lidar_imu_get_lidar_poses, name='lidar_imu_get_lidar_poses')
        add_method(self.lidar_imu_get_imu_poses, name='lidar_imu_get_imu_poses')
        add_method(self.set_lidar_imu_extrinsics, name='set_lidar_imu_extrinsics')
        # panorama camera calibration
        add_method(self.get_homography, name='get_homography')
        self.app.add_url_rule("/v1/get-panorama", view_func=self.get_panorama, methods=["GET"])
        self.app.add_url_rule("/v1/set-panorama-config", view_func=self.set_panorama_config, methods=["GET"])

    def get_projection_forward(self, lat0, lon0, lat1, lon1):
        return self.perception.call('calibration.get_projection_forward',
                                    dict(lat0=lat0,
                                         lon0=lon0,
                                         lat1=lat1,
                                         lon1=lon1)
                                   )

    def get_projection_backward(self, lat0, lon0, x, y):
        return self.perception.call('calibration.get_projection_backward',
                                    dict(lat0=lat0,
                                         lon0=lon0,
                                         x=x,
                                         y=y)
                                   )

    def get_transform(self, extrinsic_parameters):
        return self.perception.call('calibration.get_transform',
                                    dict(extrinsic_parameters=extrinsic_parameters)
                                   )

    def get_vector_from_transform(self, transform):
        return self.perception.call('calibration.get_vector_from_transform',
                                    dict(transform=transform)
                                   )

    def finetune_lidar(self, lidarIndex, transform):
        result, config = self.perception.call('calibration.finetune_lidar',
                                              dict(config=self.perception.get_config(),
                                                   lidarIndex=lidarIndex,
                                                   transform=transform)
                                             )
        self.perception.set_config(config)
        return result

    def calibrate_ground(self, points, contour, key):
        result, config = self.perception.call('calibration.calibrate_ground',
                                              dict(config=self.perception.get_config(),
                                                   points=points,
                                                   contour=contour,
                                                   key=key)
                                             )
        self.perception.set_config(config)
        return result

    def calibrate_heading(self, source, target, key):
        result, config = self.perception.call('calibration.calibrate_heading',
                                              dict(config=self.perception.get_config(),
                                                   source=source,
                                                   target=target,
                                                   key=key)
                                             )
        self.perception.set_config(config)
        return result

    def finetune_camera(self, cameraName, transform):
        result, config = self.perception.call('calibration.finetune_camera',
                                              dict(config=self.perception.get_config(),
                                                   cameraName=cameraName,
                                                   transform=transform)
                                             )
        self.perception.set_config(config)
        return result

    def calibrate_lidar_camera(self, pointsLidar, pointsCamera, cameraName):
        result, config = self.perception.call('calibration.calibrate_lidar_camera',
                                              dict(config=self.perception.get_config(),
                                                   pointsLidar=pointsLidar,
                                                   pointsCamera=pointsCamera,
                                                   cameraName=cameraName)
                                             )
        self.perception.set_config(config)
        return result

    def find_corners(self, imageData, cameraName, config):
        return self.perception.call('calibration.find_corners',
                                    dict(imageData=imageData,
                                         cameraName=cameraName,
                                         config=config)
                                   )

    def calibrate_camera(self, pointsCamera, cameraName, config):
        result, config = self.perception.call('calibration.calibrate_camera',
                                              dict(od_config=self.perception.get_config(),
                                                   pointsCamera=pointsCamera,
                                                   cameraName=cameraName,
                                                   config=config)
                                             )
        self.perception.set_config(config)
        return result

    def get_source_data(self):
        response = make_response(self.perception.call('calibration.get_calibrate_camera',
                                                      dict(config=self.perception.get_config(),
                                                           do_distort=request.get_json()['do_distort'])
                                                     ))
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def restart_lidar_ins_calibration(self):
        self.perception.call('calibration.restart_lidar_ins_calibration',
                             dict(config=self.perception.get_config())
                            )

    def get_position_points(self):
        response = make_response(self.perception.call('calibration.get_position_points'))
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def calibrate_lidar_ins(self):
        return self.perception.call('calibration.calibrate_lidar_ins')

    def get_lidar_ins_calibration(self):
        return self.perception.call('calibration.get_lidar_ins_calibration')

    def get_lidar_ins_transform(self):
        return self.perception.call('calibration.get_lidar_ins_transform')

    def set_lidar_ins_transform(self, transform):
        result, config = self.perception.call('calibration.set_lidar_ins_transform',
                                              dict(config=self.perception.get_config(),
                                                   transform=transform)
                                             )
        self.perception.set_config(config)
        return result

    def restart_lidar_imu_calibration(self):
        return self.perception.call('calibration.restart_lidar_imu_calibration',
                                    dict(config=self.perception.get_config())
                                    )

    def get_imu_position_points(self):
        response = make_response(self.perception.call('calibration.get_imu_position_points', dict(config=self.perception.get_config())))
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def calibrate_lidar_imu(self):
        return self.perception.call('calibration.calibrate_lidar_imu')

    def lidar_imu_get_lidar_poses(self):
        return self.perception.call('calibration.lidar_imu_get_lidar_poses')

    def lidar_imu_get_imu_poses(self):
        return self.perception.call('calibration.lidar_imu_get_imu_poses')

    def set_lidar_imu_extrinsics(self):
        config = self.perception.call('calibration.set_lidar_imu_extrinsics',
                                      dict(config=self.perception.get_config())
                                     )
        self.perception.set_config(config)
        return ""

    def get_homography(self, cameras, name0, name1, image0, image1, kpoint0, kpoint1, order):
        return self.perception.call('calibration.get_homography',
                                    dict(cameras=cameras,
                                         name0=name0,
                                         name1=name1,
                                         image0=image0,
                                         image1=image1,
                                         kpoint0=kpoint0,
                                         kpoint1=kpoint1,
                                         order=order)
                                   )

    def get_panorama(self):
        response = make_response(self.perception.call('calibration.get_panorama'))
        response.headers["Content-Type"] = "application/octet-stream"
        return response

    def set_panorama_config(self):
        result, config = self.perception.call('calibration.set_panorama_config',
                                              dict(config=self.perception.get_config())
                                             )
        self.perception.set_config(config)
        return result
