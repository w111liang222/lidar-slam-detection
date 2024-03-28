import time
import queue
from multiprocessing import Process, Queue, Value
import sensor_driver.common_lib.cpp_utils as util

def clear_queue(q):
    while not q.empty():
        try:
            q.get(False)
        except Exception as e:
            continue

class InferBase():
    def __init__(self, name, engine_start, cfg_file, logger, max_size):
        self.name          = name
        self.engine_start  = engine_start
        self.cfg_file      = cfg_file
        self.is_prepared   = Value('b', False)
        self.is_stopped    = Value('b', True)
        self.process_run   = Value('b', False)
        self.max_size      = max_size
        self.logger        = logger

    def initialize(self):
        raise NotImplementedError

    def set_config(self, base_cfg):
        self.base_cfg = base_cfg

    def set_output(self, output_queue):
        self.output_queue = output_queue

    def create_queue(self):
        self.input_queue = Queue(maxsize=self.max_size)

    def build_engine(self):
        raise NotImplementedError

    def reset_engine(self):
        raise NotImplementedError

    def prepare(self):
        if self.is_prepared.value:
            return

        self.build_engine()
        self.is_prepared.value = True

    def is_prepared_done(self):
        return self.is_prepared.value

    def start(self):
        self.is_stopped.value = False
        if not self.process_run.value:
            self.process_run.value = True
            self.thread = Process(target=self._infer_thread, args=())
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        while not self.is_stopped.value:
            time.sleep(1e-2)
        clear_queue(self.input_queue)
        self.logger.info('%s engine stopped' % (self.name))

    def is_overload(self):
        return (self.input_queue.full() or not self.is_prepared_done())

    def enqueue(self, data_dict):
        data_dict = self.prepare_data(data_dict)
        self.input_queue.put_nowait(data_dict)

    def prepare_data(self, data_dict):
        raise NotImplementedError

    def _infer_thread(self):
        util.set_thread_priority(self.name, 30)
        self.prepare()
        while self.process_run.value:
            while not self.engine_start.value and self.process_run.value:
                self.is_stopped.value = True
                time.sleep(1e-2)

            self.reset_engine()
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
                    self.logger.debug(f'{self.name} inference time is: {process_time:.1f} ms')
                else:
                    self.logger.warn(f'{self.name} inference time is: {process_time:.1f} ms')

                if output_dict is None:
                    continue
                self.output_queue.put(output_dict)

    def process(self, data_dict):
        raise NotImplementedError