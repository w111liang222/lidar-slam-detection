from multiprocessing import Process, Event, Queue, Value
import queue
import time
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
            self.input_queue = Queue(maxsize = self.queue_max_size)
            self.process_event = Event()
            self.is_start.value = True
            self.run_process = Process(target=self._run, name=self.name)
            self.run_process.daemon = True
            self.run_process.start()

    def stop(self):
        if self.is_start.value:
            while self.input_queue.qsize() > 0:
                self.logger.warn('%s, queue size %d' % (self.name, self.input_queue.qsize()))
                time.sleep(0.1)

            self.is_start.value = False
            self.process_event.wait(1.0)
            if not self.process_event.is_set():
                self.logger.warn('%s process is still running' % self.name)
            self.run_process.terminate()
            self.run_process.join()

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
        self.process_event.set()
