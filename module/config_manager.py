import os
from pathlib import Path
from threading import Thread
from easydict import EasyDict
import time
import yaml
import json
import copy

from util.common_util import run_cmd, get_network, opener
from util.setup_network import network_validation
from module.export_interface import register_interface

class ConfigManager():
    def __init__(self, config_path, logger):
        self.config_path = Path(config_path)
        self.config = EasyDict(yaml.safe_load(self.config_path.open()))
        self.logger = logger

        self.config = self.set_extra_config(self.config)
        _, self.config = self.check_config(self.config)

        register_interface('config.dump_config', self.dump_config)

    def get_config(self):
        config = copy.deepcopy(self.config)
        return config

    def set_temporary(self, config):
        self.temporary_config = config

    def get_temporary(self):
        return self.temporary_config

    def check_config(self, config):
        old_cfg = self.config

        if old_cfg.board != config.board or \
           old_cfg.pipeline != config.pipeline or \
           old_cfg.detection.freespace != config.detection.freespace or \
           old_cfg.system != config.system:
            return 'Reboot', config

        if old_cfg != config:
            if old_cfg.input != config.input and config.input.mode == "offline":
                config = self.set_extra_config(config)
            return 'Reset', config

        if isinstance(self.config.pipeline[0], str):
            self.logger.warn('upgrade to new pipeline config')
            self.config.pipeline = [self.config.pipeline]

        return 'Success', config

    def timed_restart(self):
        self.logger.info('Restarting')
        time.sleep(1)
        # restart system service
        run_cmd('systemctl restart perception.service')

    def set_extra_config(self, config):
        def replace_config(cfg1 , cfg2):
            cfg = cfg2
            for idx, c in enumerate(cfg):
                c['static_extrinsic_parameters'] = c['extrinsic_parameters']
                c['index'] = idx
                if 'undistortion' in c and c['undistortion']:
                    c['undistortion'] = False
            return cfg
        try:
            # replace 'lidar', 'camera', 'radar', 'ins' by record config
            if config.input.mode == "offline":
                record_config_path = Path(config.input.data_path).expanduser().absolute() / 'cfg.yaml'
                if record_config_path.exists():
                    record_config = EasyDict(yaml.safe_load(record_config_path.open()))
                    config.lidar = replace_config(config.lidar, record_config.lidar)
                    config.camera = replace_config(config.camera, record_config.camera)
                    config.radar = replace_config(config.radar, record_config.radar)
                    config.ins.update(record_config.ins)

            # check config completeness
            if 'imu_extrinsic_parameters' not in config.ins:
                config.ins['imu_extrinsic_parameters'] = [0, 0, 0, 0, 0, 0]

            # check the lidar name (replace Ouster-OSX-XXX with Ouster-OSX)
            for lidar in config.lidar:
                if 'Ouster-OS1' in lidar['name']:
                    lidar['name'] = 'Ouster-OS1'
                elif 'Ouster-OS2' in lidar['name']:
                    lidar['name'] = 'Ouster-OS2'

        except Exception as e:
            self.logger.error(e)

        return config

    def update_config(self, config, hostname="", restart_service=False):
        old_cfg = self.config
        self.config = config
        self.dump_config(self.config_path)

        if restart_service:
            self.timed_task = Thread(target=self.timed_restart, daemon=True)
            self.timed_task.start()

        return "localhost"

    def dump_config(self, path, config=None, sync=True):
        config = config if config is not None else self.config
        _d = json.loads(json.dumps(config))
        try:
            os.umask(0)
            with open(path, 'w', opener=opener) as f:
                yaml.dump(_d, f)
                if sync:
                    os.fsync(f)
        except Exception as e:
            self.logger.warn(e)

    def get_config_print(self, config):
        _d = json.loads(json.dumps(config))
        _d.pop('board', None)
        _d.pop('lidar_all', None)
        _d.pop('radar_all', None)
        _d.pop('system', None)
        return "\n" + yaml.dump(_d, default_flow_style=False)