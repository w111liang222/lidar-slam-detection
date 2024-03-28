
from threading import Thread
import queue
import time
import sensor_driver.common_lib.cpp_utils as util

class SLAMTemplate():
    def __init__(self, name, logger, queue_max_size = 3):
        self.name = name
        self.logger = logger
        self.queue_max_size = queue_max_size
        self.thread_start = False

    def init(self):
        self.input_queue = queue.Queue(maxsize=self.queue_max_size)
        self.output_queue = queue.Queue(maxsize=self.queue_max_size)
        self.thread_start = True
        self.thread = Thread(target=self._run_thread, daemon=True)
        self.thread.start()

    def deinit(self):
        self.thread_start = False
        self.thread.join()

    def isInited(self):
        raise NotImplementedError

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def enqueue(self, input_dict):
        raise NotImplementedError

    def get_pose(self):
        raise NotImplementedError

    def get_key_frame(self, index):
        raise NotImplementedError

    def is_overload(self):
        return (self.input_queue.full() or self.output_queue.full() or not self.isInited())

    def process(self, input_dict):
        raise NotImplementedError

    def _run_thread(self):
        util.set_thread_priority(self.name, 30)
        self.start()
        while self.thread_start:
            try:
                input_dict = self.input_queue.get(block=True, timeout=0.1)
            except queue.Empty:
                continue
            if not input_dict:
                continue
            start_time = time.monotonic()
            output_dict = self.process(input_dict)
            process_time = (time.monotonic() - start_time) * 1000
            if process_time < 100:
                self.logger.debug(f'{self.name} process time is: {process_time:.1f} ms')
            else:
                self.logger.warn(f'{self.name} process time is: {process_time:.1f} ms')
            if not output_dict:
                continue
            self.output_queue.put(output_dict)
        self.stop()
        self.logger.info('%s stopped' % self.name)

    def get_output(self, block=True, timeout=None):
        return self.output_queue.get(block=block, timeout=timeout)