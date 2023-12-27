import time
import math
import numpy as np

from .data_manager_template import DataManagerTemplate
from sensor_driver.ins_driver.ins_driver import InsDriver
from ..export_interface import register_interface

class InsDataManager(DataManagerTemplate):
    def __init__(self, cfg, data_cfg, logger=None):
        self.ins = InsDriver(port=cfg.ins.port, device=cfg.ins.device, ins_type=cfg.ins.ins_type,
                             ex_param=cfg.ins.extrinsic_parameters, logger=logger)
        self.ins.open()

        super().__init__('Ins', cfg, data_cfg, logger = logger)
        self.ins_info = {'valid': False, 'latitude': 0, 'longitude': 0, 'heading': 0, 'velocity': 0}
        register_interface('ins.get_ins_status', self.get_status)

    def offline_init(self):
        self.ins.set_offline_mode()

    def setup(self, cfg):
        super().setup(cfg)
        if cfg.ins.relay.use:
            self.ins.start_relay(cfg.ins.relay.destination)
        self.ins.start()

    def start_capture(self):
        pass

    def stop_capture(self):
        pass

    def release(self):
        self.ins.stop()
        self.ins.close()

    def update_ins_info(self, data_dict):
        self.ins_info['valid']     = data_dict['ins_valid']
        self.ins_info['latitude']  = data_dict['ins_data']['latitude']
        self.ins_info['longitude'] = data_dict['ins_data']['longitude']
        self.ins_info['heading']   = data_dict['ins_data']['heading']
        self.ins_info['velocity']  = math.sqrt(data_dict['ins_data']['Ve'] * data_dict['ins_data']['Ve'] + \
                                               data_dict['ins_data']['Vn'] * data_dict['ins_data']['Vn'])

    def post_process_data(self, data_dict, **kwargs):
        self.update_ins_info(data_dict)
        if self.mode == "online":
            # workaround to limit the trigger frequence
            if data_dict['ins_valid'] and not data_dict['lidar_valid'] and \
               not data_dict['image_valid'] and not data_dict['radar_valid']:
               time.sleep(0.1)

        if self.mode == "offline":
            if data_dict['ins_valid']:
                self.ins.set_offline_data(data_dict['ins_data'])
                # previous recorded data may not has 'imu_data'
                if 'imu_data' not in data_dict:
                    data_dict['imu_data'] = np.asarray([[data_dict['ins_data']['timestamp'],
                                                         data_dict['ins_data']['gyro_x'],
                                                         data_dict['ins_data']['gyro_y'],
                                                         data_dict['ins_data']['gyro_z'],
                                                         data_dict['ins_data']['acc_x'],
                                                         data_dict['ins_data']['acc_y'],
                                                         data_dict['ins_data']['acc_z']]], dtype=np.float64)

            data = self.ins.trigger(data_dict['frame_start_timestamp'])
            data_dict['motion_valid']   = data['motion_valid']
            data_dict['motion_t']       = data['motion_t']
            data_dict['motion_heading'] = data['motion_heading']

        return data_dict

    def get_data(self, data_dict):
        if 'frame_start_timestamp' not in data_dict:
            data_dict['frame_start_timestamp'] = int(time.time() * 1000000)

        if self.mode == "online":
            data = self.ins.trigger(data_dict['frame_start_timestamp'])
            if data['ins_valid']:
                data['imu_data'] = np.asarray(data['imu_data'], dtype=np.float64)
        else:
            data = {'ins_valid': False}

        if not data['ins_valid']:
            data['ins_data'] = {'timestamp': 0, 'longitude': 0, 'latitude': 0, 'altitude': 0,
                                'heading': 0, 'pitch': 0, 'roll': 0, 'gyro_x': 0, 'gyro_y': 0, 'gyro_z': 0,
                                'acc_x': 0, 'acc_y': 0, 'acc_z': 0, 'Ve': 0, 'Vn': 0, 'Vu': 0, 'Status': 0}

        return data

    def get_status(self):
        valid_count = self.ins.get_valid_message_count()
        received_count = self.ins.get_receive_message_count()
        self.ins_info.update({'valid_count' : valid_count, 'received_count' : received_count})
        return self.ins_info