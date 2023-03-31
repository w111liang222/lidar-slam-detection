import queue
import time

from ..manager_template import ManagerTemplate

class DataSplit(ManagerTemplate):
    def __init__(self, cfg, logger, system):
        super().__init__('Split', cfg, logger, system)
        self.peer = []

    def setup(self, cfg):
        self.cfg = cfg
        self.data_queue = queue.Queue(maxsize=3)

    def release(self):
        self.peer = []

    def is_init_done(self):
        return True

    def set_config(self, cfg):
        self.cfg = cfg

    def try_enqueue(self):
        return not self.data_queue.full()

    def enqueue(self, data_dict, module_name):
        self.data_queue.put_nowait(data_dict)

    def get_data(self, **kwargs):
        try:
            data_dict = self.data_queue.get(block=True, timeout=1.0)
        except queue.Empty:
            data_dict = dict()

        return data_dict

    def connect(self, peer):
        self.peer.append(peer)

    def run_loop(self):
        self.logger.info('data split peers: {}'.format([name.name for name in self.peer]))
        while self.system.is_initialized:
            if self.cfg.input.mode == "online" and not self.system.is_running:
                time.sleep(1e-2)
                continue

            data_dict = self.get_data()
            if not bool(data_dict):
                continue

            enqueue_ok = True
            for peer in self.peer:
                enqueue_ok = enqueue_ok and peer.try_enqueue()

            if not enqueue_ok:
                continue

            for peer in self.peer:
                data_dict = peer.pre_process(data_dict)

            for peer in self.peer:
                peer.enqueue(data_dict.copy(), self.name)

        self.logger.info('%s loop stopped' % (self.name))