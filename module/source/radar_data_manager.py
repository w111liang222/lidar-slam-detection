import time
from .data_manager_template import DataManagerTemplate
from sensor_driver.radar_driver.radar_driver import RadarDriver
from ..export_interface import register_interface

class RadarDataManager(DataManagerTemplate):
    def __init__(self, cfg, logger=None):
        super().__init__('Radar', cfg, logger = logger)

        self.radar = dict()
        self.radar_info = dict()
        for i in range(0, len(cfg.radar)):
            rc = cfg.radar[i]
            static_ex_param = [0, 0, 0, 0, 0, 0]
            if 'static_extrinsic_parameters' in rc:
                static_ex_param = rc.static_extrinsic_parameters
            self.radar[str(i) + '-' + rc.name] = RadarDriver(name=rc.name,
                                              device=rc.device,
                                              baud=rc.baud,
                                              token=str(i),
                                              ex_param=rc.extrinsic_parameters,
                                              static_ex_param=static_ex_param,
                                              logger=logger
                                             )
            self.radar_info[str(i) + '-' + rc.name] = {'num' : 0, 'valid' : False}
        for radar in self.radar.values():
            radar.open()
        register_interface('radar.get_status', self.get_status)

    def loop_run_once(self, sensor, sensor_name):
        valid = True
        frame = sensor.get_radar_frame_online(timeout=1000000)
        frame_dict = dict()
        frame_dict[sensor_name] = frame
        if frame['radar_boxes'].shape[0] == 0:
            valid = False
        return frame_dict, valid

    def start_capture(self):
        super().start_capture()
        if self.mode == "online":
            self.start_loop(self.radar)
            for radar in self.radar.values():
                radar.start()

    def stop_capture(self):
        if self.mode == "online":
            for radar in self.radar.values():
                radar.stop()
            self.stop_loop()

    def release(self):
        if self.mode == "online":
            for radar in self.radar.values():
                radar.close()

    def get_status(self):
        return self.radar_info

    def update_radar_info(self, radar_dict):
        for name, info in self.radar_info.items():
            if name in radar_dict:
                info['valid'] = True
                info['num'] = radar_dict[name]['radar_boxes'].shape[0]
            else:
                info['valid'] = False

    def post_process_data(self, data_dict):
        radar_dict = data_dict['radar']
        if self.mode == "offline":
            for name, data in radar_dict.copy().items():
                if name not in self.radar:
                    radar_dict.pop(name)
                    continue
                radar_dict[name] = self.radar[name].transform(data)
            data_dict['radar'] = radar_dict
            if not bool(radar_dict):
                data_dict['radar_valid'] = False
        self.update_radar_info(radar_dict)
        return data_dict

    def get_data_online(self, data_dict):
        if not bool(self.radar):
            return {'radar_valid': False, 'radar': dict()}

        timeout = 0 if 'frame_start_timestamp' in data_dict else 1.0
        frame_dict = self.get_loop_data(timeout)
        if not bool(frame_dict):
            return {'radar_valid': False, 'radar': dict()}

        if 'frame_start_timestamp' not in data_dict:
            data_dict['frame_start_timestamp'] = int(time.time() * 1000000)

        return {'radar_valid': True, 'radar': frame_dict}

    def get_data_offline(self, data_dict):
        return {'radar_valid': False, 'radar': dict()}
