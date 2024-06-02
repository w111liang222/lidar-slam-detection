import numpy as np
from sensor_driver.common_lib.cpp_utils import get_transform_from_RPYT
import lidar_driver_ext as driver

class LidarDriver():
    def __init__(self, name, port, token,
                 ex_param = [0, 0, 0, 0, 0, 0],
                 static_ex_param = [0, 0, 0, 0, 0, 0],
                 points_range = [-100, -100, -100, 100, 100, 100],
                 exclude = [0, 0, 0, 0, 0, 0],
                 logger = None):
        self.token = token
        self.name = name
        self.port = port
        self.ex_param = ex_param
        self.static_ex_param = static_ex_param
        self.points_range = points_range
        self.exclude = exclude
        self.logger = logger

    def open(self):
        driver.create_lidar(self.token, self.name)
        driver.set_external_param(self.token, self.name, self.ex_param[0], self.ex_param[1], self.ex_param[2],
                                  self.ex_param[5], self.ex_param[4], self.ex_param[3])
        driver.set_range_filter(self.token, self.name, self.points_range[0], self.points_range[1], self.points_range[2],
                                self.points_range[3], self.points_range[4], self.points_range[5])
        driver.set_exclude(self.token, self.name, self.exclude[0], self.exclude[1], self.exclude[2],
                           self.exclude[3], self.exclude[4], self.exclude[5])

    def close(self):
        driver.destory_lidar(self.token, self.name)

    def start(self):
        driver.start_capture(self.token, self.name, int(self.port))

    def stop(self):
        driver.stop_capture(self.token, self.name)

    def start_package_transfer(self, dest_ip):
        driver.start_package_transfer(self.token, self.name, dest_ip)

    def stop_package_transfer(self):
        driver.stop_package_transfer(self.token, self.name)

    def get_points_online(self, timeout):
        return driver.get_points_online(self.token, self.name, timeout)

    def set_external_param(self, ex_param):
        self.ex_param = ex_param
        driver.set_external_param(self.token, self.name, self.ex_param[0], self.ex_param[1], self.ex_param[2],
                                  self.ex_param[5], self.ex_param[4], self.ex_param[3])

    def set_range(self, points_range):
        self.points_range = points_range
        driver.set_range_filter(self.token, self.name, self.points_range[0], self.points_range[1], self.points_range[2],
                                self.points_range[3], self.points_range[4], self.points_range[5])

    def set_exclude(self, exclude):
        self.exclude = exclude
        driver.set_exclude(self.token, self.name, self.exclude[0], self.exclude[1], self.exclude[2],
                           self.exclude[3], self.exclude[4], self.exclude[5])

    def transform_points(self, points, attr):
        if abs(self.ex_param[0] - self.static_ex_param[0]) < 1e-4 and \
           abs(self.ex_param[1] - self.static_ex_param[1]) < 1e-4 and \
           abs(self.ex_param[2] - self.static_ex_param[2]) < 1e-4 and \
           abs(self.ex_param[3] - self.static_ex_param[3]) < 1e-4 and \
           abs(self.ex_param[4] - self.static_ex_param[4]) < 1e-4 and \
           abs(self.ex_param[5] - self.static_ex_param[5]) < 1e-4:
            pass
        else:
            tM = get_transform_from_RPYT(self.ex_param[0], self.ex_param[1], self.ex_param[2],
                                        self.ex_param[5], self.ex_param[4], self.ex_param[3])

            static_tM = get_transform_from_RPYT(self.static_ex_param[0], self.static_ex_param[1], self.static_ex_param[2],
                                                self.static_ex_param[5], self.static_ex_param[4], self.static_ex_param[3])

            tM = np.dot(tM, np.linalg.inv(static_tM))
            points_hom = np.hstack((points[:, :3], np.ones((points.shape[0], 1), dtype=np.float32)))
            points[:, 0:3] = np.dot(tM, points_hom.T).T[:, 0:3]

        x_idx  = np.bitwise_and(points[:, 0] < self.points_range[3], points[:, 0] > self.points_range[0])
        y_idx  = np.bitwise_and(points[:, 1] < self.points_range[4], points[:, 1] > self.points_range[1])
        z_idx  = np.bitwise_and(points[:, 2] < self.points_range[5], points[:, 2] > self.points_range[2])

        ex_idx = np.bitwise_and(points[:, 0] < self.exclude[3], points[:, 0] > self.exclude[0])
        ey_idx = np.bitwise_and(points[:, 1] < self.exclude[4], points[:, 1] > self.exclude[1])
        ez_idx = np.bitwise_and(points[:, 2] < self.exclude[5], points[:, 2] > self.exclude[2])
        xy_idx = ~(ex_idx & ey_idx & ez_idx) & x_idx & y_idx & z_idx

        points = points[xy_idx]
        attr['points_attr'] = attr['points_attr'][xy_idx]
        return points, attr