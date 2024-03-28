import time
import queue

from ..manager_template import ManagerTemplate
from sensor_inference.infer import DetInfer
from sensor_fusion.tracker import Tracker
from sensor_fusion.fusion import Fusion
from .object_filter import ObjectFilter

class DetectManager(ManagerTemplate):
    def __init__(self, cfg, logger, system):
        super().__init__('Detect', cfg, logger, system)

        # only initialize once
        self.detection = DetInfer(**cfg["board"]["inference_engine"], logger=logger)
        self.detection.initialize()

        self.fusion = Fusion(self.logger)
        self.fusion.setup_tunnel(self.detection)
        self.fusion.initialize()

        self.tracking = Tracker(self.logger)
        self.tracking.setup_tunnel(self.fusion)
        self.tracking.initialize()

        self.modules = [self.detection, self.fusion, self.tracking]

    def setup(self, cfg):
        self.cfg = cfg

        self.object_filter = ObjectFilter(self.logger)
        self.object_filter.set_config(cfg)

        for module in self.modules:
            module.set_config(cfg)
            module.start()

        self.frame_queue = queue.Queue(maxsize=3)

    def release(self):
        for module in self.modules:
            module.stop()

    def start(self):
        pass

    def stop(self):
        pass

    def is_init_done(self):
        return self.detection.is_prepared_done()

    def set_config(self, cfg):
        self.object_filter.set_config(cfg)
        for module in self.modules:
            module.set_config(cfg)

    def try_enqueue(self):
        if self.frame_queue.full() or self.detection.is_overload() or self.fusion.is_overload() or self.tracking.is_overload():
            self.logger.warn('overload: frame queue {}, detection {}, fusion {}, tracking {}'.format(
                              self.frame_queue.full(),
                              self.detection.is_overload(),
                              self.fusion.is_overload(),
                              self.tracking.is_overload()))
            return False

        return True

    def pre_process(self, data_dict):
        data_dict.pop('pred_boxes', None)
        data_dict.pop('pred_attr', None)
        data_dict.pop('pred_traj', None)
        return data_dict

    def enqueue(self, input_dict, module_name):
        input_dict['do_detection'] = self.detection.enqueue(input_dict)
        self.frame_queue.put(input_dict)

    def get_data(self, **kwargs):
        # wait until get points or thread quit
        while self.system.is_initialized:
            try:
                frame_dict = self.frame_queue.get(block=True, timeout=1.0)
                break
            except queue.Empty:
                self.logger.warn('frame queue is Empty')
        # wait until get tracking result or thread quit
        while self.system.is_initialized and frame_dict['do_detection']:
            try:
                track_dict = self.tracking.get_output(block=True, timeout=1.0)
                frame_dict.update(track_dict)
                break
            except queue.Empty:
                self.logger.warn('tracking queue is Empty')

        if not self.system.is_initialized:
            return dict()

        frame_dict.pop('do_detection')
        frame_dict = self.object_filter.filter(frame_dict)

        return frame_dict