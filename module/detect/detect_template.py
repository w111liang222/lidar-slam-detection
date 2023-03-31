
from multiprocessing import Process, Event, Queue, Value
import queue
import time
import sensor_driver.common_lib.cpp_utils as util

class DetectTemplate():
    def __init__(self, name, logger, queue_max_size = 3):
        self.name = name
        self.logger = logger
        self.queue_max_size = queue_max_size
        self.tunnel_set = False
        self.thread_start = Value('b', False)

    def setup_tunnel(self, peer):
        self.tunnel_set = True
        self.input_queue = peer.get_output_queue()

    def init(self):
        if not self.tunnel_set:
            self.input_queue = Queue(maxsize=self.queue_max_size)
        self.output_queue = Queue(maxsize=self.queue_max_size)
        self.thread_event = Event()
        self.thread_start.value = True
        self.thread = Process(target=self._run_thread, name=self.name, args=())
        self.thread.daemon = True
        self.thread.start()

    def deinit(self):
        self.thread_start.value = False
        self.thread_event.wait(1.0)
        if not self.thread_event.is_set():
            self.logger.warn('%s process is still running' % self.name)
        self.thread.terminate()
        self.thread.join()

    def enqueue(self, input_dict):
        if not input_dict:
            return
        self.input_queue.put_nowait(input_dict)

    def is_overload(self):
        return self.input_queue.full()

    def process(self, input_dict):
        raise NotImplementedError

    def _run_thread(self):
        util.set_thread_priority(self.name, 30)
        while self.thread_start.value:
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
                self.logger.debug('%s process time is: %.1f ms' % (self.name, process_time))
            else:
                self.logger.warn('%s process time is: %.1f ms' % (self.name, process_time))
            if not output_dict:
                continue
            self.output_queue.put(output_dict)

        self.logger.info('%s stopped' % self.name)
        self.thread_event.set()

    def get_output(self, block=True, timeout=None):
        try:
            output_dict = self.output_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return dict()
        return output_dict

    def get_output_queue(self):
        return self.output_queue
