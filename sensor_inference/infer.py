import queue
from multiprocessing import Queue, Value
from sensor_inference.object_infer import ObjectInfer
from sensor_inference.trafficlight_infer import TrafficLightInfer

class DetInfer():
    def __init__(self, detection_capability, detection_config, logger = None, max_size = 1):
        self.logger = logger
        self.max_size = max_size
        self.capability = detection_capability
        self.engine_start = Value('b', False)
        self.engines = []
        if detection_capability.object:
            self.engines.append(ObjectInfer(self.engine_start, detection_config.object, logger, max_size))
        else:
            self.logger.warn('object detection is disabled')

        if detection_capability.trafficlight:
            self.engines.append(TrafficLightInfer(self.engine_start, detection_config.trafficlight, logger, max_size))
        else:
            self.logger.warn('trafficlight detection is disabled')

    def initialize(self):
        self.detection_queue = Queue(maxsize=self.max_size * 2)
        for engine in self.engines:
            engine.initialize()
            engine.set_output(self.detection_queue)

    def set_config(self, cfg):
        for engine in self.engines:
            engine.set_config(cfg)

    def is_prepared_done(self):
        prepared = True
        for engine in self.engines:
            prepared = (prepared and engine.is_prepared_done())
        return prepared

    def start(self):
        if self.engine_start.value:
            return

        self.engine_start.value = True
        for engine in self.engines:
            engine.start()

    def stop(self):
        if not self.engine_start.value:
            return

        self.engine_start.value = False
        for engine in self.engines:
            engine.stop()
        self.logger.info('inference engine stopped')

    def is_overload(self):
        overload = False
        for engine in self.engines:
            overload = (overload or engine.is_overload())
        return overload

    def enqueue(self, data_dict):
        if not self.engine_start.value:
            return False

        input_dict = data_dict.copy()
        # pop out unused data
        input_dict.pop('image_jpeg', None)
        input_dict.pop('radar', None)

        # add detection capability for fusion
        input_dict['detection_capability'] = self.capability.copy()
        is_enqueue = False
        for engine in self.engines:
            engine.enqueue(input_dict.copy())
            is_enqueue = True

        return is_enqueue

    def get_objects(self, block=True, timeout=None):
        try:
            data = self.detection_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return dict()
        return data

    def get_output_queue(self):
        return self.detection_queue