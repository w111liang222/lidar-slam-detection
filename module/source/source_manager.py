import collections
import time

from .player_data_manager import PlayerDataManager
from .lidar_data_manager  import LidarDataManager
from .camera_data_manager import CameraDataManager
from .ins_data_manager    import InsDataManager
from .radar_data_manager  import RadarDataManager
from ..manager_template   import ManagerTemplate

__all__ = {
    'player': PlayerDataManager,
    'lidar' : LidarDataManager,
    'camera': CameraDataManager,
    'ins'   : InsDataManager,
    'radar' : RadarDataManager,
}

class SourceManager(ManagerTemplate):
    def __init__(self, cfg, logger, system):
        super().__init__('Source', cfg, logger, system)

        self.register = ['lidar', 'camera', 'radar', 'ins']
        self.last_frame_timestamp = int(time.monotonic() * 1000000)

    def setup(self, cfg):
        self.cfg = cfg
        self.source_dict = collections.OrderedDict()
        self.source_dict['player'] = __all__['player'](cfg, self.system, self.logger)
        for device in self.register:
            self.logger.info(f'{self.name}, start to setup: {device}')
            source = __all__[device](cfg, self.logger)
            source.setup(cfg)
            self.logger.info(f'{self.name}, setup: {device}, done')
            self.source_dict[device] = source

    def release(self):
        for device, source in self.source_dict.items():
            self.logger.info(f'{self.name}, start to release: {device}')
            source.release()
            self.logger.info(f'{self.name}, release: {device}, done')

    def start(self):
        for device, source in self.source_dict.items():
            source.start_capture()

    def stop(self):
        for device, source in self.source_dict.items():
            self.logger.info(f'{self.name}, try stop capture: {device}')
            source.stop_capture()
            self.logger.info(f'{self.name}, stop capture: {device}, done')

    def is_init_done(self):
        return True

    def set_config(self, cfg):
        for device, source in self.source_dict.items():
            source.set_config(cfg)

    def try_enqueue(self):
        raise RuntimeError("Source module has no input")

    def enqueue(self, data_dict, module_name):
        raise RuntimeError("Source module has no input")

    def get_data(self):
        data_dict = dict(frame_timestamp_monotonic=int(time.monotonic() * 1000000))
        for device, source in self.source_dict.items():
            data = source.get_data(data_dict)
            # merge source data
            data.update(data_dict)
            data_dict = data

        for device, source in self.source_dict.items():
            data_dict = source.post_process_data(data_dict)

        # check data valid
        if (not data_dict['lidar_valid']) and (not data_dict['image_valid']) and \
           (not data_dict['radar_valid']) and (not data_dict['ins_valid']):
            time.sleep(1.0)
            return dict()

        # system is paused
        if data_dict['frame_timestamp_monotonic'] == self.last_frame_timestamp:
            return dict()

        timestep = data_dict['frame_timestamp_monotonic'] - self.last_frame_timestamp
        data_dict['timestep'] = timestep if timestep > 0 else 100000
        self.last_frame_timestamp = data_dict['frame_timestamp_monotonic']

        return data_dict
