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
        for idx, net in enumerate(config.board.network):
            if not net.DHCP:
                continue
            network = get_network('eth' + str(idx))
            if network:
                net.IP, net.mask, net.gateway = network
            else:
                net.IP, net.mask, net.gateway = "0.0.0.0", "0.0.0.0", "0.0.0.0"

        return config

    def set_temporary(self, config):
        self.temporary_config = config

    def get_temporary(self):
        return self.temporary_config

    def check_config(self, config):
        old_cfg = self.config
        # check network setting validation
        is_valid, err_msg = network_validation(config)
        if not is_valid:
            self.logger.error(err_msg)
            return 'Error', config

        # restore the network config when DHCP enabled
        if len(old_cfg.board.network) == len(config.board.network):
            for idx in range(len(config.board.network)):
                if old_cfg.board.network[idx].DHCP and config.board.network[idx].DHCP:
                    config.board.network[idx].IP = old_cfg.board.network[idx].IP
                    config.board.network[idx].mask = old_cfg.board.network[idx].mask
                    config.board.network[idx].gateway = old_cfg.board.network[idx].gateway

        if old_cfg.board != config.board or old_cfg.pipeline != config.pipeline:
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
        except Exception as e:
            self.logger.error(e)

        return config

    def update_config(self, config, hostname="", restart_service=True):
        old_cfg = self.config
        self.config = config
        self.dump_config(self.config_path)
        self.dump_config('/home/znqc/work/cfg/board_cfg_all.yaml')

        # get the modified network config interface
        init_interface = ""
        network_num = min(len(old_cfg.board.network), len(config.board.network))
        for idx in range(network_num):
            if old_cfg.board.network[idx] != config.board.network[idx]:
                init_interface = init_interface + str(idx) + " "
                self.logger.info('network interface: %d need to be re-init' % (idx))

        if init_interface != "":
            with open('/tmp/init_interface', 'w') as f:
                f.write(init_interface)
                os.fsync(f)

        # find the current access network card
        new_hostname = config.board.network[0].IP
        for idx, net in enumerate(old_cfg.board.network):
            if net.DHCP:
                network = get_network('eth' + str(idx), gw = False)
                ip_addr = net.IP if network is None else network[0]
            else:
                ip_addr = net.IP
            if hostname == ip_addr:
                new_hostname = hostname if (net.DHCP and config.board.network[idx].DHCP) else config.board.network[idx].IP
                break

        if restart_service and (old_cfg.board != config.board or old_cfg.pipeline != config.pipeline):
            self.timed_task = Thread(target=self.timed_restart, daemon=True)
            self.timed_task.start()

        return new_hostname

    def dump_config(self, path, sync=True):
        _d = json.loads(json.dumps(self.config))
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