import queue
import time
from multiprocessing import Process, Queue, Value
import sensor_driver.common_lib.cpp_utils as util

def clear_queue(q):
    while not q.empty():
        try:
            q.get(False)
        except Exception as e:
            continue

class DetectTemplate():
    def __init__(self, name, logger, queue_max_size = 3):
        self.name = name
        self.logger = logger
        self.queue_max_size = queue_max_size
        self.tunnel_set = False
        self.engine_start = Value('b', False)
        self.thread_start = Value('b', False)
        self.is_stopped   = Value('b', True)
        self.output_queue = Queue(maxsize=self.queue_max_size)

    def initialize(self):
        if not self.tunnel_set:
            self.input_queue = Queue(maxsize=self.queue_max_size)

    def set_config(self, cfg):
        pass

    def setup_tunnel(self, peer):
        self.tunnel_set = True
        self.input_queue = peer.get_output_queue()

    def start(self):
        if self.engine_start.value:
            return

        self.engine_start.value = True
        self.is_stopped.value = False
        if not self.thread_start.value:
            self.thread_start.value = True
            self.thread = Process(target=self._run_thread, name=self.name, args=())
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        if not self.engine_start.value:
            return

        self.engine_start.value = False
        while not self.is_stopped.value:
            time.sleep(1e-2)

        clear_queue(self.input_queue)
        clear_queue(self.output_queue)
        self.logger.info('%s stopped' % self.name)

    def prepare(self):
        pass

    def enqueue(self, input_dict):
        self.input_queue.put_nowait(input_dict)

    def is_overload(self):
        return self.input_queue.full()

    def process(self, input_dict):
        raise NotImplementedError

    def _run_thread(self):
        util.init_backtrace_handle()
        util.set_thread_priority(self.name, 30)
        while self.thread_start.value:
            while not self.engine_start.value and self.thread_start.value:
                self.is_stopped.value = True
                time.sleep(1e-2)

            self.prepare()
            while self.engine_start.value:
                try:
                    input_dict = self.input_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue

                if self.engine_start.value is False:
                    break
                start_time = time.monotonic()
                output_dict = self.process(input_dict)
                process_time = (time.monotonic() - start_time) * 1000
                if process_time < 100:
                    self.logger.debug(f'{self.name} process time is: {process_time:.1f} ms')
                else:
                    self.logger.warn(f'{self.name} process time is: {process_time:.1f} ms')

                if not output_dict:
                    continue
                try:
                    self.output_queue.put(output_dict, block=True, timeout=1.0)
                except queue.Full:
                    self.logger.error('%s output queue is full' % (self.name))

    def get_output(self, block=True, timeout=None):
        return self.output_queue.get(block=block, timeout=timeout)

    def get_output_queue(self):
        return self.output_queue
