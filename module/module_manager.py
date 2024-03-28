import sys
import collections
import time
import threading
import traceback
from threading import Thread

from third_party import psutil
from .source.source_manager import SourceManager
from .detect.detect_manager import DetectManager
from .sink.sink_manager import SinkManager
from .slam.slam_manager import SLAMManager
from .common.data_splitter import DataSplit
from .common.data_merger import DataMerge
from .export_interface import register_interface

from util.common_util import run_cmd

__all__ = {
    'Split' : DataSplit,
    'Merge' : DataMerge,
    'Source': SourceManager,
    'Detect': DetectManager,
    'Sink'  : SinkManager,
    'SLAM'  : SLAMManager,
}

class ModuleManager():
    def __init__(self, cfg, logger, system):
        self.cfg = cfg
        self.logger = logger
        self.system = system
        self.modules_in_setup = ['Source']

        register_interface('system.get_status', self.get_status)

    def _setup_pipeline(self, pipeline):
        for module in pipeline:
            if module not in self.module_dict:
                manager = __all__[module](self.cfg, self.logger, self)
                self.module_dict[module] = manager

        for idx in range(len(pipeline) - 1):
            cur_module = self.module_dict[pipeline[idx]]
            next_module = self.module_dict[pipeline[idx + 1]]
            cur_module.connect(next_module)
            self.logger.info('pipeline: %s connect to %s' % (pipeline[idx], pipeline[idx + 1]))

    def init(self):
        self.is_initialized, self.is_running = False, False
        self.module_dict = collections.OrderedDict()

        for module in [item for pipe in self.cfg.pipeline for item in pipe]:
            if module not in self.modules_in_setup and module not in self.module_dict:
                manager = __all__[module](self.cfg, self.logger, self)
                self.module_dict[module] = manager

    def setup(self, cfg):
        self.cfg = cfg
        self.is_initialized, self.is_running = True, True

        for module in self.modules_in_setup:
            if module in [item for pipe in self.cfg.pipeline for item in pipe]:
                manager = __all__[module](cfg, self.logger, self)
                self.module_dict[module] = manager

        for pipe in cfg.pipeline:
            self._setup_pipeline(pipe)

        for device, module in self.module_dict.items():
            self.logger.info('start to setup module: %s' % (device))
            module.setup(cfg)
            self.logger.info('setup module: %s, done' % (device))

        for device, module in self.module_dict.items():
            module.start_loop()

        self.check_thread = Thread(target=self.check_status, name="Checker", daemon=True)
        self.check_thread.start()

    def release(self):
        self.is_initialized = False
        self.check_thread.join()
        for device, module in self.module_dict.items():
            module.tear_down()
        for device, module in self.module_dict.items():
            module.release()

    def start(self):
        self.is_running = True
        self.logger.info('system is started')

    def stop(self):
        self.is_running = False
        self.logger.info('system is paused')

    def set_config(self, cfg):
        for device, module in self.module_dict.items():
            module.set_config(cfg)

    def check_status(self):
        retry = 60 # 60s timeout
        status = "Booting"
        while retry > 0 and self.is_initialized:
            time.sleep(1)
            retry = retry - 1
            status = self.get_status()
            run_cmd('echo {} > output/logs/status'.format(status))
            if status == "Running" or status == "Paused":
                break

        if status != "Running" and status != "Paused":
            self.logger.error('timeout, system initilaizing error')
            return

        self.logger.info('system status {}, running thread/process information:'.format(status))
        for thread in threading.enumerate():
            if thread.name == "QueueFeederThread" or thread.name == "Checker":
                continue
            self.logger.info('Thread  {}:{:<10} daemon {} ident 0x{:x}'.format(thread.native_id, thread.name, thread.daemon, thread.ident))

        current_process = psutil.Process()
        processes = current_process.children(recursive=True)
        processes.append(current_process)
        counter = 0
        while self.is_initialized:
            time.sleep(1)
            counter = counter + 1
            if counter < 10: # 10s
                continue
            counter = 0

            # check system status
            for process in processes:
                time.sleep(0.1)
                mem = process.memory_info()._asdict()
                self.logger.info('Process {}:{:<10} CPU {:>5}%, Mem {} MB'.format(process.pid, process.name(), process.cpu_percent(), int(mem['rss'] / 1024 / 1024)))


    def is_init_done(self):
        init_done = True
        for device, module in self.module_dict.items():
            init_done = (init_done and module.is_init_done())
        return init_done

    def get_status(self):
        if not self.is_initialized:
            return "Booting"
        if not self.is_init_done():
            return "Initializing"
        if not self.is_running:
            return "Paused"
        else:
            return "Running"

def dump_threads_stack():
    stack_list = []
    frames = sys._current_frames()
    for thread in threading.enumerate():
        if thread.name == "QueueFeederThread":
            continue
        stack_list.append('Thread {}:{}, daemon {}, ident 0x{:x}\n'.format(thread.native_id, thread.name, thread.daemon, thread.ident))
        stack_list.extend(traceback.extract_stack(frames[thread.ident]).format())
        stack_list.append('\n')
    return stack_list