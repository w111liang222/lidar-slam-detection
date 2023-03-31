import time
from multiprocessing import Process, Queue, Value
from util.common_util import clear_queue
from hardware.platform_common import is_jetson
import sensor_driver.common_lib.cpp_utils as util

class InferBase():
    def __init__(self, name, engine_start, cfg_file, serialize_engine, logger, max_size):
        self.name = name
        self.engine_start = engine_start
        self.cfg_file = cfg_file
        self.is_prepared = Value('b', False)
        self.is_stopped = Value('b', True)
        self.process_run = Value('b', False)
        self.serialize_engine = serialize_engine
        self.max_size = max_size
        self.logger = logger

    def initialize(self):
        raise NotImplementedError

    def set_output(self, output_queue):
        self.output_queue = output_queue

    def create_queue(self):
        self.input_queue = Queue(maxsize=self.max_size)

    def build_engine(self, calib):
        raise NotImplementedError

    def prepare(self, calib = None):
        if self.is_prepared.value:
            return
        if is_jetson():
            from pycuda import driver
            driver.init()
            self.ctx = driver.Device(0).make_context()
        self.build_engine(calib)
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
        try:
            self.input_queue.put_nowait(dict())
        except:
            pass
        while not self.is_stopped.value:
            time.sleep(1e-2)
        clear_queue(self.input_queue)
        self.logger.info('%s engine stopped' % (self.name))

    def __del__(self):
        self.stop()
        if self.process_run.value:
            self.process_run.value = False
            self.thread.join(0.1)
            if self.thread.is_alive():
                self.thread.terminate()

    def is_overload(self):
        return (self.input_queue.full() or not self.is_prepared_done())

    def enqueue(self, data_dict):
        start_time = time.monotonic()
        data = self.prepare_data(data_dict)
        self.logger.debug('%s, prepare_data time is: %.1f ms' % (self.name, (time.monotonic() - start_time)*1000))
        self.input_queue.put_nowait(data)

    def prepare_data(self, data_dict):
        raise NotImplementedError

    def _infer_thread(self):
        util.set_thread_priority(self.name, 30)
        self.prepare()
        while self.process_run.value:
            while not self.engine_start.value and self.process_run.value:
                self.is_stopped.value = True
                time.sleep(1e-2)
            while self.engine_start.value:
                data_dict = self.input_queue.get()
                if self.engine_start.value is False:
                    break
                start_time = time.monotonic()
                result = self.process(data_dict)
                infer_time = (time.monotonic() - start_time) * 1000
                if infer_time < 100:
                    self.logger.debug('%s inference time is: %.1f ms' % (self.name, infer_time))
                else:
                    self.logger.warn('%s inference time is: %.1f ms' % (self.name, infer_time))

                if result is None:
                    continue
                self.output_queue.put(result)

    def process(self, data_dict):
        raise NotImplementedError