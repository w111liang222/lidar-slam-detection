import queue
import time
from threading import Thread, Lock

from .sink_template import SinkTemplate
from ..export_interface import register_interface
from util.period_calculator import PeriodCalculator
from proto.proto_serialize import serialize_to_string, serialize_raw_to_string

class HttpSink(SinkTemplate):
    def __init__(self, cfg, logger):
        super().__init__('HTTP', cfg, logger)
        self.is_start = False
        self.is_process_start = False
        self.data_lock = Lock()
        self.query = PeriodCalculator()
        self.data = PeriodCalculator()

        self.last_connect_time = time.monotonic()
        self.client_args = dict()

        self.start()
        register_interface('sink.get_proto_http', self.get_proto_http)
        register_interface('sink.get_proto_http_raw', self.get_proto_http_raw)

    def start(self):
        if not self.is_start:
            self.input_queue = queue.Queue(maxsize=1)
            self.output_queue = queue.Queue(maxsize=1)
            self.is_start = True

    def stop(self):
        if self.is_start:
            self.is_start = False
            self.stop_process_thread()

    def start_process_thread(self):
        self.data_lock.acquire()
        if not self.is_process_start:
            self.is_process_start = True
            self.process_thread = Thread(target=self.run_process, name="HttpSink", daemon=True)
            self.process_thread.start()
        self.data_lock.release()

    def stop_process_thread(self):
        self.data_lock.acquire()
        if self.is_process_start:
            self.is_process_start = False
            try:
                self.input_queue.put_nowait(None)
            except:
                pass
            self.process_thread.join()
            self.logger.info('HttpSink, process thread join succeess')
        self.data_lock.release()

    def enqueue(self, data_dict):
        if not self.is_start or not self.is_process_start:
            return

        if (time.monotonic() - self.last_connect_time) > 2.0:
            self.logger.warn('http client disconnect')
            self.stop_process_thread()
            return

        data_fps = self.data.hit()
        data_dict['frame_fps'] = data_fps
        if not self.input_queue.full():
            self.input_queue.put_nowait(data_dict)

    def run_process(self):
        self.logger.info("http sink process loop start")
        while self.is_process_start:
            data = self.input_queue.get()
            if data is None:
                continue

            if 'raw_data' in self.client_args and self.client_args['raw_data']:
                result = data
            else:
                result = serialize_to_string(data, **self.client_args)

            if not self.output_queue.full():
                self.output_queue.put_nowait(result)
        self.logger.info("http sink process loop stop")

    def get_proto_http(self, **kwargs):
        query_fps = self.query.hit()
        if query_fps > 5.0:
            self.logger.debug('http query FPS: %.1f' % query_fps)
        else:
            self.logger.warn('http query FPS: %.1f' % query_fps)

        self.client_args = kwargs
        self.last_connect_time = time.monotonic()

        if not self.is_process_start:
            self.logger.info('http client connect')
            self.start_process_thread()

        try:
            result_dict = self.output_queue.get(block=True, timeout=0.5)
        except queue.Empty:
            result_dict = dict()

        if 'raw_data' in kwargs and kwargs['raw_data']:
            if not isinstance(result_dict, dict):
                return dict()
        else:
            if not isinstance(result_dict, bytes):
                return ""

        return result_dict

    def get_proto_http_raw(self):
        result_dict = self.get_proto_http(raw_data=True)
        return serialize_raw_to_string(result_dict)
