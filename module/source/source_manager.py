import collections
import time

from .player_data_manager import PlayerDataManager
from .lidar_data_manager import LidarDataManager
from .camera_data_manager import CameraDataManager
from .ins_data_manager import InsDataManager
from .radar_data_manager import RadarDataManager
from ..manager_template import ManagerTemplate
from ..export_interface import register_interface

__all__ = {
    'player': PlayerDataManager,
    'lidar': LidarDataManager,
    'camera': CameraDataManager,
    'ins': InsDataManager,
    'radar': RadarDataManager,
}

class SourceManager(ManagerTemplate):
    def __init__(self, cfg, logger, system):
        super().__init__('Source', cfg, logger, system)

        self.register = ['lidar', 'camera', 'radar', 'ins']
        self.last_timestep = 0
        self.client_connect_time = 0
        register_interface('source.get_source_data_pb', self.get_source_data)

    def setup(self, cfg):
        self.cfg = cfg
        self.source_dict = collections.OrderedDict()
        self.source_dict['player'] = __all__['player'](cfg, cfg['input'], self.system, self.logger)
        for device in self.register:
            if device in cfg:
                self.logger.info('%s, start to setup: %s' % (self.name, device))
                source = __all__[device](cfg, cfg['input'], self.logger)
                source.setup(cfg)
                self.logger.info('%s, setup: %s, done' % (self.name, device))
                self.source_dict[device] = source

    def release(self):
        for device, source in self.source_dict.items():
            self.logger.info('%s, start to release: %s' % (self.name, device))
            source.release()
            self.logger.info('%s, release: %s, done' % (self.name, device))

    def start(self):
        for device, source in self.source_dict.items():
            source.start_capture()

    def stop(self):
        for device, source in self.source_dict.items():
            self.logger.info('%s, try stop capture: %s' % (self.name, device))
            source.stop_capture()
            self.logger.info('%s, stop capture: %s, done' % (self.name, device))

    def is_init_done(self):
        return True

    def set_config(self, cfg):
        for device, source in self.source_dict.items():
            source.set_config(cfg)

    def try_enqueue(self):
        raise RuntimeError("Source module has no input")

    def enqueue(self, data_dict, module_name):
        raise RuntimeError("Source module has no input")

    def get_data(self, **kwargs):
        # check system query or http query
        if (time.monotonic() - self.client_connect_time) < 2.0 and 'http_query' not in kwargs:
            time.sleep(1.0)
            return dict()

        data_dict = dict()
        for device, source in self.source_dict.items():
            data = source.get_data(data_dict)
            # merge source data
            data.update(data_dict)
            data_dict = data

        if 'frame_timestamp_monotonic' not in data_dict:
            data_dict['frame_timestamp_monotonic'] = int(time.monotonic() * 1000000)

        for device, source in self.source_dict.items():
            data_dict = source.post_process_data(data_dict, **kwargs)

        # system is paused
        if data_dict['frame_timestamp_monotonic'] == self.last_timestep:
            return dict()

        # check data valid
        data_valid = False
        for valid in ['lidar_valid', 'image_valid', 'radar_valid', 'ins_valid']:
            if data_dict[valid]:
                data_valid = True
                break
        if not data_valid:
            time.sleep(1.0)
            return dict()

        if self.last_timestep == 0:
            timestep = 100000 # 100 ms
        else:
            timestep = data_dict['frame_timestamp_monotonic'] - self.last_timestep

        # check the difference of two frame
        if timestep < 0:
            self.logger.warn('current time: %d, is less than last %d' % (data_dict['frame_timestamp_monotonic'], self.last_timestep))
            timestep = 100000 # 100 ms

        data_dict['timestep'] = timestep
        self.last_timestep = data_dict['frame_timestamp_monotonic']

        return data_dict

    def get_source_data(self, **kwargs):
        from proto.proto_serialize import serialize_to_string
        self.client_connect_time = time.monotonic()
        kwargs['http_query'] = True
        data_dict = self.get_data(**kwargs)
        if 'raw_data' in kwargs and kwargs['raw_data']:
            return data_dict
        else:
            return serialize_to_string(data_dict, use_raw_image=True)