import collections
import numpy as np
from .data_manager_template import DataManagerTemplate
from sensor_driver.lidar_driver.lidar_driver import LidarDriver
from ..export_interface import register_interface

class LidarDataManager(DataManagerTemplate):
    def __init__(self, cfg, logger=None):
        super().__init__('Lidar', cfg, logger = logger)

        self.lidar = dict()
        self.lidar_info = dict()
        for i, lc in enumerate(cfg.lidar):
            if self.mode == "offline" and 'index' not in lc:
                continue
            idx = i if self.mode == "online" else lc.index
            name = str(idx) + '-' + lc.name
            self.lidar[name] = LidarDriver(name=lc.name, token=str(i),
                                           ex_param=lc.extrinsic_parameters,
                                           static_ex_param=lc.static_extrinsic_parameters if 'static_extrinsic_parameters' in lc else [0, 0, 0, 0, 0, 0],
                                           points_range=lc.range,
                                           exclude=lc.exclude if 'exclude' in lc else [-0.5, -0.5, 0, 0.5, 0.5, 0],
                                           logger=logger,
                                           port=lc.port)
            self.lidar_info[name] = {'timestamp' : 0, 'num' : [0, 4], 'valid' : False}

        for lidar in self.lidar.values():
            lidar.open()
        register_interface('lidar.get_status', self.get_status)

    def setup(self, cfg):
        super().setup(cfg)
        if cfg.output.point_cloud.use:
            self._start_package_transfer(cfg.output.point_cloud.destination)

    def loop_run_once(self, sensor, sensor_name):
        valid = True
        data = sensor.get_points_online(timeout=1000000)
        ret = dict()
        ret[sensor_name] = data
        if data["points"].shape[0] == 0:
            valid = False
        return ret, valid

    def start_capture(self):
        super().start_capture()
        if self.mode == "online":
            self.start_loop(self.lidar)
            for lidar in self.lidar.values():
                lidar.start()

    def stop_capture(self):
        if self.mode == "online":
            for lidar in self.lidar.values():
                lidar.stop()
            self.stop_loop()

    def release(self):
        for lidar in self.lidar.values():
            lidar.close()

    def get_status(self):
        return self.lidar_info

    def update_lidar_info(self, points_dict, points_attr):
        for name, info in self.lidar_info.items():
            if name in points_dict:
                info['valid'] = True
                info['timestamp'] = points_attr[name]["timestamp"]
                info['num'] = points_dict[name].shape
            else:
                info['valid'] = False

    def post_process_data(self, data_dict):
        points_dict = data_dict['points']
        points_attr = data_dict['points_attr']
        if self.mode == "offline":
            points_dict = collections.OrderedDict(sorted(points_dict.items()))
            reduce_num = 0
            for name, point in points_dict.copy().items():
                points_dict.pop(name)
                attr = points_attr.pop(name)
                if name not in self.lidar:
                    reduce_num = reduce_num + 1
                    continue
                if point.shape[1] == 3:
                    point = np.concatenate((point, np.zeros((point.shape[0], 1), dtype=np.float32)), axis=1)
                new_name = str(int(name[0]) - reduce_num) + name[1:]
                points_dict[new_name], points_attr[new_name] = self.lidar[name].transform_points(point, attr)

            data_dict['points'] = points_dict
            data_dict['points_attr'] = points_attr
            if not bool(points_dict):
                data_dict['lidar_valid'] = False

        self.update_lidar_info(points_dict, points_attr)
        return data_dict

    def get_data_online(self, data_dict):
        if not bool(self.lidar):
            return {'points' : dict(), 'points_attr' : dict(), 'lidar_valid' : False}

        frame_dict = self.get_loop_data()
        if not bool(frame_dict):
            return {'points' : dict(), 'points_attr' : dict(), 'lidar_valid' : False}

        points_dict = dict()
        for name in self.lidar:
            if name not in frame_dict:
                continue

            if 'frame_start_timestamp' not in data_dict:
                data_dict['frame_start_timestamp'] = frame_dict[name]["timestamp"]
            points_dict[name] = frame_dict[name].pop("points")

        return {'points' : points_dict, 'points_attr' : frame_dict, 'lidar_valid' : True}

    def get_data_offline(self, data_dict):
        return {'points' : dict(), 'points_attr' : dict(), 'lidar_valid' : False}

    def _start_package_transfer(self, dest_ip):
        for lidar in self.lidar.values():
            lidar.start_package_transfer(dest_ip)

    def _stop_package_transfer(self):
        for lidar in self.lidar.values():
            lidar.stop_package_transfer()