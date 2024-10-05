from threading import Thread
import queue
from .sink_template import SinkTemplate
import numpy as np

from sensor_driver.common_lib import cpp_utils
from sensor_driver.common_lib.cpp_utils import (
    get_transform_from_rtk,
)

class DumpSink(SinkTemplate):
    def __init__(self, cfg, logger):
        super().__init__('DumpSink', cfg, logger)
        self.f = open("output/dump_data.txt", "wb", 0)
        self.data_origin = None
        self.start()

    def set_config(self, cfg):
        self.cfg = cfg

    def start(self):
        if not self.is_start.value:
            self.input_queue = queue.Queue(maxsize = self.queue_max_size)
            self.is_start.value = True
            self.run_process = Thread(target=self._run, name=self.name)
            self.run_process.daemon = True
            self.run_process.start()

    def stop(self):
        if self.is_start.value:
            self.is_start.value = False
            self.run_process.join()

    def _run(self):
        self.logger.info('deamon %s starts' % self.name)
        self.prepare_run()
        while self.is_start.value:
            try:
                data_dict = self.input_queue.get(block=True, timeout=0.1)
            except queue.Empty:
                continue
            if not data_dict:
                continue
            self.sink(data_dict)
        self.logger.info('deamon %s stops' % self.name)

    def prepare_data(self, data_dict):
        data_dict.pop('motion_t', None)
        data_dict.pop('motion_heading', None)
        data_dict.pop('points', None)
        data_dict.pop('image', None)
        data_dict.pop('image_jpeg', None)
        data_dict.pop('image_param', None)
        return data_dict

    def sink(self, data_dict):
        if 'pose' in data_dict and data_dict['slam_valid'] and data_dict['pose']['state'] != "Initializing" and \
           'ins_data' in data_dict and data_dict['ins_valid'] and data_dict['ins_data']['Status'] != 0:
            pose_dict = data_dict['pose']
            [x, y, z, roll, pitch, yaw] = cpp_utils.get_cfg_from_transform(pose_dict['odom_matrix'])
            if abs(roll) >= 90.0 or abs(pitch) >= 90:
                [x, y, z, roll, pitch, yaw] = cpp_utils.get_cfg_from_transform(cpp_utils.get_transform_from_cfg(x, y, z, roll, pitch, -yaw))
            else:
                yaw = -yaw
            if yaw < 0:
                yaw += 360

            slam_data = np.array([pose_dict['latitude'], pose_dict['longitude'], pose_dict['altitude'], yaw, pitch, roll])
            rtk_data  = np.array([data_dict['ins_data']['latitude'], data_dict['ins_data']['longitude'], data_dict['ins_data']['altitude'], \
                                  data_dict['ins_data']['heading'], data_dict['ins_data']['pitch'], data_dict['ins_data']['roll']])

            if self.data_origin is None:
                self.data_origin = rtk_data

            slam_T = get_transform_from_rtk(*[*self.data_origin, *slam_data])
            rtk_T = get_transform_from_rtk(*[*self.data_origin, *rtk_data])

            slam_data = np.array([*slam_data, slam_T[0, 3], slam_T[1, 3], slam_T[2, 3]])
            rtk_data = np.array([*rtk_data, rtk_T[0, 3], rtk_T[1, 3], rtk_T[2, 3]])

            data = np.array([*slam_data, *rtk_data])
            data = data.reshape((1, -1))
            np.savetxt(self.f, data, fmt='%1.10f')