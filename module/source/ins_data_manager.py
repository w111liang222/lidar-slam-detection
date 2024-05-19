import time
import math

from .data_manager_template import DataManagerTemplate
from sensor_driver.ins_driver.ins_driver import InsDriver
from ..export_interface import register_interface

class InsDataManager(DataManagerTemplate):
    def __init__(self, cfg, logger=None):
        super().__init__('Ins', cfg, logger = logger)

        self.ins = InsDriver(port=cfg.ins.port, device=cfg.ins.device, ins_type=cfg.ins.ins_type,
                             mode=cfg.input.mode, extrinsic_param=cfg.ins.extrinsic_parameters, logger=logger)
        self.ins.open()
        self.ins_info = {'valid': False, 'latitude': 0, 'longitude': 0, 'heading': 0, 'velocity': 0}
        register_interface('ins.get_status', self.get_status)

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

    def post_process_data(self, data_dict):
        self.update_ins_info(data_dict)
        if self.mode == "online":
            # workaround to limit the trigger frequency
            if data_dict['ins_valid'] and not data_dict['lidar_valid'] and \
               not data_dict['image_valid'] and not data_dict['radar_valid']:
               time.sleep(0.1)
        # else:
        #     data = self.ins.trigger(data_dict['frame_start_timestamp'])
        #     data_dict['motion_valid']   = data['motion_valid']
        #     data_dict['motion_t']       = data['motion_t']
        #     data_dict['motion_heading'] = data['motion_heading']
        #     data_dict['ins_data']       = data['ins_data']

        return data_dict

    def get_data_online(self, data_dict):
        if 'frame_start_timestamp' not in data_dict:
            data_dict['frame_start_timestamp'] = int(time.time() * 1000000)
        return self.ins.trigger(data_dict['frame_start_timestamp'])

    def get_data_offline(self, data_dict):
        if 'frame_start_timestamp' not in data_dict:
            data_dict['frame_start_timestamp'] = int(time.time() * 1000000)
        # if 'ins_valid' in data_dict and data_dict['ins_valid']:
        #     self.ins.set_offline_data(data_dict['ins_data'], data_dict['imu_data'])
        return {'ins_valid': False, 'ins_data': {'latitude': 0, 'longitude': 0, 'heading': 0, 'Ve': 0, 'Vn': 0}}

    def get_status(self):
        return self.ins_info