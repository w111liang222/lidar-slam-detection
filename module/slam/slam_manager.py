import time
import queue
from easydict import EasyDict

from ..manager_template import ManagerTemplate
from slam.slam import SLAM
from util.common_util import has_extension_disk

from ..export_interface import register_interface

class SLAMManager(ManagerTemplate):
    def __init__(self, cfg, logger, system):
        super().__init__('SLAM', cfg, logger, system)
        register_interface('slam.restart_mapping', self.restart_mapping)
        register_interface('slam.save_mapping', self.save_mapping)
        register_interface('slam.get_save_progress', self.get_save_progress)

    def setup(self, cfg):
        self.cfg = cfg
        slam_config = cfg.slam
        resolution = slam_config.mapping.map_resolution if slam_config.mode == "mapping" else slam_config.localization.map_resolution
        sensor_input = slam_config.mapping.sensor_input if slam_config.mode == "mapping" else slam_config.localization.sensor_input
        key_frames_interval = slam_config.mapping.key_frames_interval if slam_config.mode == "mapping" else slam_config.localization.key_frames_interval

        has_disk, disk_name = has_extension_disk()
        map_path = disk_name + '/' + slam_config.localization.map_path

        self.slam = SLAM(
            mode=slam_config.mode,
            method=slam_config.method,
            map_path=map_path,
            sensor_input=sensor_input,
            resolution=resolution,
            key_frames_interval=key_frames_interval,
            config=cfg,
            logger=self.logger,
        )

        self.slam.init()
        self.frame_queue = queue.Queue(maxsize=3)

    def release(self):
        self.slam.deinit()

    def restart_mapping(self, config):
        self.release()
        self.setup(EasyDict(config))

    def save_mapping(self, map_name=None):
        has_disk, disk_name = has_extension_disk()
        if not has_disk:
            return "error"

        return self.slam.start_save_mapping(disk_name + "/lp_log/map", map_name)

    def get_save_progress(self):
        return self.slam.get_save_progress()

    def start(self):
        pass

    def stop(self):
        pass

    def is_init_done(self):
        return self.slam.isInited()

    def set_config(self, cfg):
        self.cfg = cfg

    def try_enqueue(self):
        retry = 0
        while True:
            if self.slam.is_overload() or self.frame_queue.full():
                retry = retry + 1
                if self.cfg.input.mode == 'offline' and retry < 1000:
                    time.sleep(1e-2)
                    continue
                else:
                    return False
            else:
                break

        return True

    def enqueue(self, input_dict, module_name):
        input_dict['do_slam'] = self.slam.enqueue(input_dict)
        self.frame_queue.put(input_dict)

    def get_data(self):
        # wait until get points or thread quit
        while self.system.is_initialized:
            try:
                frame_dict = self.frame_queue.get(block=True, timeout=1.0)
                break
            except queue.Empty:
                self.logger.warn('frame queue is Empty')
        # wait until get slam result or thread quit
        while self.system.is_initialized and frame_dict['do_slam']:
            try:
                slam_dict = self.slam.get_output(block=True, timeout=1.0)
                frame_dict.update(slam_dict)
                break
            except queue.Empty:
                self.logger.warn('SLAM output is Empty')

        if not self.system.is_initialized:
            return dict()

        frame_dict.pop('do_slam')
        return frame_dict