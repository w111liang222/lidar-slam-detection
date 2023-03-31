from multiprocessing import Queue, Value
import queue
from sensor_inference.lidar_infer import LidarInfer
from sensor_inference.image_infer import ImageInfer

class DetInfer():
    def __init__(self, lidar_cfg_file = None, image_cfg_file = None,
                 deactive_lidar = False, deactive_image = False,
                 serialize_engine = False, logger = None, max_size = 3):
        self.engine_start = Value('b', False)
        self.max_size = max_size
        self.logger = logger
        if not deactive_lidar:
            self.lidar_engine = LidarInfer(self.engine_start, lidar_cfg_file, serialize_engine, logger, max_size)
        else:
            self.lidar_engine = None
            self.logger.warn('lidar object detection is deactived')

        if not deactive_image:
            self.image_engine = ImageInfer(self.engine_start, image_cfg_file, serialize_engine, logger, max_size)
        else:
            self.image_engine = None
            self.logger.warn('image object detection is deactived')

        if deactive_lidar and deactive_image:
            self.logger.error('lidar or image detection must be actived one')

    def initialize(self):
        if self.lidar_engine is not None:
            self.lidar_engine.initialize()
        if self.image_engine is not None:
            self.image_engine.initialize()
        self.objects_queue = Queue(maxsize=self.max_size * 2)
        if self.lidar_engine is not None:
            self.lidar_engine.set_output(self.objects_queue)
        if self.image_engine is not None:
            self.image_engine.set_output(self.objects_queue)

    def prepare_lidar(self, calib=None):
        self.lidar_engine.prepare(calib)

    def prepare_image(self, calib=None):
        self.image_engine.prepare(calib)

    def is_prepared_done(self):
        lidar_engine_prepared = self.lidar_engine.is_prepared_done() if self.lidar_engine is not None else True
        image_engine_prepared = self.image_engine.is_prepared_done() if self.image_engine is not None else True
        return (lidar_engine_prepared and image_engine_prepared)

    def start(self):
        if self.engine_start.value:
            return
        self.engine_start.value = True
        if self.lidar_engine is not None:
            self.lidar_engine.start()
        if self.image_engine is not None:
            self.image_engine.start()

    def stop(self):
        if not self.engine_start.value:
            return
        self.engine_start.value = False
        if self.lidar_engine is not None:
            self.lidar_engine.stop()
        if self.image_engine is not None:
            self.image_engine.stop()
        self.logger.info('inference engine stopped')

    def is_overload(self):
        lidar_engine_overload = self.lidar_engine.is_overload() if self.lidar_engine is not None else False
        image_engine_overload = self.image_engine.is_overload() if self.image_engine is not None else False
        return (lidar_engine_overload or image_engine_overload)

    def enqueue(self, data_dict):
        if not self.engine_start.value:
            return False
        is_enqueue = False

        input_dict = data_dict.copy()
        # pop out unused data
        input_dict.pop('radar', None)
        input_dict.pop('image_jpeg', None)

        input_dict['lidar_valid'] = input_dict['lidar_valid'] if self.lidar_engine is not None else False
        input_dict['image_valid'] = input_dict['image_valid'] if self.image_engine is not None else False

        if self.lidar_engine is not None and input_dict['lidar_valid']:
            self.lidar_engine.enqueue(input_dict.copy())
            is_enqueue = True
        if self.image_engine is not None and input_dict['image_valid']:
            self.image_engine.enqueue(input_dict.copy())
            is_enqueue = True

        return is_enqueue

    def get_objects(self, block=True, timeout=None):
        try:
            pred_dicts = self.objects_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return dict()
        return pred_dicts

    def get_output_queue(self):
        return self.objects_queue