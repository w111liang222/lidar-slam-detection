import queue

from ..manager_template import ManagerTemplate

class DataMerge(ManagerTemplate):
    def __init__(self, cfg, logger, system):
        super().__init__('Merge', cfg, logger, system)
        self.data_queues = dict()

    def setup(self, cfg):
        self.cfg = cfg

    def release(self):
        self.data_queues = dict()

    def start(self):
        pass

    def stop(self):
        pass

    def is_init_done(self):
        return True

    def set_config(self, cfg):
        self.cfg = cfg

    def try_enqueue(self):
        is_full = False
        for name, queue in self.data_queues.items():
            is_full = is_full or queue.full()
        return not is_full

    def enqueue(self, data_dict, module_name):
        self.data_queues[module_name].put_nowait(data_dict)

    def get_data(self, **kwargs):
        data_dict = dict()
        for name, data_queue in self.data_queues.items():
            while self.system.is_initialized:
                try:
                    data = data_queue.get(block=True, timeout=1.0)
                    data_dict.update(data)
                    break
                except queue.Empty:
                    self.logger.warn('%s has no output' % (name))
                    continue

        return data_dict

    def notify_connect(self, peer):
        self.data_queues[peer.name] = queue.Queue(maxsize=10)
