import time
from threading import Thread

from util.period_calculator import PeriodCalculator
import sensor_driver.common_lib.cpp_utils as util

class ManagerTemplate():
    def __init__(self, name, cfg, logger, system):
        self.name = name
        self.cfg = cfg
        self.logger = logger
        self.system = system

        self.peer = None
        self.loop_thread = None
        self.period = PeriodCalculator()

    def setup(self, cfg):
        raise NotImplementedError

    def tear_down(self):
        if self.loop_thread is not None:
            self.logger.info('%s wait for loop join' % (self.name))
            self.loop_thread.join()
        self.loop_thread = None
        self.peer = None
        self.logger.info('%s tear down success' % (self.name))

    def release(self):
        raise NotImplementedError

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def is_init_done(self):
        raise NotImplementedError

    def set_config(self, cfg):
        raise NotImplementedError

    def try_enqueue(self):
        raise NotImplementedError

    def enqueue(self, data_dict, module_name):
        raise NotImplementedError

    def get_data(self, **kwargs):
        raise NotImplementedError

    def connect(self, peer):
        self.peer = peer
        peer.notify_connect(self)

    def notify_connect(self, peer):
        pass

    def start_loop(self):
        if self.peer is not None:
            self.loop_thread = Thread(target=self.run_loop, daemon=True)
            self.loop_thread.start()

    def pre_process(self, data_dict):
        return data_dict

    def run_loop(self):
        util.set_thread_priority(self.name, 30)
        self.start()
        while self.system.is_initialized:
            if self.cfg.input.mode == "online" and not self.system.is_running:
                time.sleep(1e-2)
                continue

            fps = self.period.hit()
            if fps > 8.0:
                self.logger.debug('%s, FPS: %.1f' % (self.name, fps))
            else:
                self.logger.warn('%s, FPS: %.1f' % (self.name, fps))
            data_dict = self.get_data()
            if not bool(data_dict):
                if self.system.is_running:
                    self.logger.warn('%s has no data' % (self.name))
                continue

            if not self.peer.try_enqueue():
                self.logger.debug('%s is overload' % (self.peer.name))
                continue

            data_dict = self.peer.pre_process(data_dict)
            self.peer.enqueue(data_dict, self.name)

        self.logger.info('%s loop try to stop' % (self.name))
        self.stop()
        self.logger.info('%s loop stopped' % (self.name))
