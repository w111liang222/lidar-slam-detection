
from hardware.platform_common import VERSION, BOARD_NAME, MACHINE, JETPACK
print(f"Running on board: {VERSION}-{BOARD_NAME}-{MACHINE}-{JETPACK}")

from easydict import EasyDict

# register calibration interfaces
import calibration.calibration
import sensor_driver.common_lib.cpp_utils as util

from module.module_manager import ModuleManager, dump_threads_stack
from module.config_manager import ConfigManager
from module.export_interface import call

from util.log import get_logger_from_config

class Perception:
    def __init__(self, config_path="cfg/board_cfg_all.yaml"):
        util.init_backtrace_handle()
        util.set_thread_priority("Perception", 30)
        self.logger, log_level = get_logger_from_config(config_path)
        self.set_runtime_config({'log_level': log_level})

        self.config_manager = ConfigManager(config_path, self.logger)
        config = self.config_manager.get_config()

        self.module_manager = ModuleManager(config, self.logger, self)
        self.module_manager.init()

        self.setup(config)

    def setup(self, config):
        self.module_manager.setup(EasyDict(config))
        self.logger.info('start success')

    def release(self):
        self.module_manager.release()
        self.logger.info('release success')

    def start(self):
        self.module_manager.start()

    def pause(self):
        self.module_manager.stop()

    def get_config(self):
        return self.config_manager.get_config()

    def set_config(self, config):
        config = EasyDict(config)
        self.logger.info(self.config_manager.get_config_print(config))
        status, config = self.config_manager.check_config(config)
        self.config_manager.set_temporary(config)
        if status == 'Reset':
            self.release()
            self.setup(config)
            status = 'Success'

        if status == 'Success':
            self.config_manager.update_config(config)

        return dict(
            status=status,
            config=config,
        )

    def do_reboot(self, confirmed, hostname="", restart_service=True):
        if confirmed:
            config = self.config_manager.get_temporary()
            hostname = self.config_manager.update_config(config, hostname, restart_service)

        config = self.config_manager.get_config()
        return dict(
            hostname=hostname
        )

    def get_status(self):
        return dict(
            status = call('system.get_status'),
            disk   = call('frame.get_status'),
            lidar  = call('lidar.get_status'),
            camera = call('camera.get_status'),
            ins    = call('ins.get_status'),
            radar  = call('radar.get_status'),
        )

    def set_runtime_config(self, config):
        if 'log_level' in config:
            self.logger.setLevel(config['log_level'])
            util.set_logger_level(config['log_level'])

        if 'ipc_enable' in config:
            util.set_message_core(config['ipc_enable'])

    def call(self, interface, args = dict()):
        return call(interface, **args)

    def dump(self):
        return dict(
            stack=dump_threads_stack(),
        )