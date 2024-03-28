from multiprocessing import Process, Queue, Value
import queue
import sensor_driver.common_lib.cpp_utils as util

class SinkTemplate():
    def __init__(self, name, cfg, logger, queue_max_size = 10):
        self.name = name
        self.cfg = cfg
        self.logger = logger
        self.queue_max_size = queue_max_size
        self.is_start = Value('b', False)

    def start(self):
        if not self.is_start.value:
            self.is_start.value = True
            self.input_queue = Queue(maxsize = self.queue_max_size)
            self.run_process = Process(target=self._run, name=self.name)
            self.run_process.daemon = True
            self.run_process.start()

    def stop(self):
        if self.is_start.value:
            self.is_start.value = False
            self.logger.info('wait for %s process join' % self.name)
            self.run_process.join()
            self.logger.info('%s process join ok' % self.name)

    def set_config(self, cfg):
        self.cfg = cfg

    def prepare_data(self, data_dict):
        return data_dict

    def enqueue(self, data_dict):
        if not self.is_start.value:
            return
        data_dict = self.prepare_data(data_dict)
        if not self.input_queue.full():
            self.input_queue.put_nowait(data_dict)
        else:
            self.logger.warn('input queue is full, %s is too slow' % self.name)

    def sink(self, data_dict):
        raise NotImplementedError

    def prepare_run(self):
        return

    def _run(self):
        self.logger.info('deamon %s starts' % self.name)
        util.set_thread_priority(self.name, 30)
        self.prepare_run()
        while self.is_start.value:
            try:
                data_dict = self.input_queue.get(block=True, timeout=0.1)
            except queue.Empty:
                continue
            if not data_dict:
                continue
            self.sink(data_dict)

        self.logger.info('deamon %s stops' % self.name)
