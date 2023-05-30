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

        self.engine = DetInfer(
            **cfg["board"]["inference_engine"], logger=logger
        )
        self.engine.initialize()

    def setup(self, cfg):
        self.cfg = cfg
        self.engine.start()

        self.fusing = Fusion(cfg["board"]["fusion"], self.logger)
        self.fusing.setup_tunnel(self.engine)
        self.fusing.init()

        self.tracking = Tracker(self.logger)
        self.tracking.setup_tunnel(self.fusing)
        self.tracking.init()

        self.object_filter = ObjectFilter(self.logger)
        self.object_filter.set_output(cfg.output)
        if len(cfg.roi[0]['contour']) == 0:
            self.object_filter.disable_roi()
        else:
            self.object_filter.enable_roi(cfg.roi[0]['contour'], is_include=cfg.roi[0]['include'])

        self.frame_queue = queue.Queue(maxsize=10)

    def release(self):
        self.tracking.deinit()
        self.fusing.deinit()
        self.engine.stop()

    def start(self):
        pass

    def stop(self):
        pass

    def is_init_done(self):
        return self.engine.is_prepared_done()

    def set_config(self, cfg):
        self.object_filter.set_output(cfg.output)
        if len(cfg.roi[0]['contour']) == 0:
            self.object_filter.disable_roi()
        else:
            self.object_filter.enable_roi(cfg.roi[0]['contour'], is_include=cfg.roi[0]['include'])

    def try_enqueue(self):
        retry = 0
        while True:
            if self.frame_queue.full() or self.engine.is_overload() or self.fusing.is_overload() or \
               self.tracking.is_overload():
                retry = retry + 1
                if self.cfg.input.mode == 'offline' and retry < 100:
                    time.sleep(1e-2)
                    continue
                else:
                    self.logger.warn('overload: frame queue {}, engine {}, fusing {}, tracking {}'.format(
                                      self.frame_queue.full(),
                                      self.engine.is_overload(),
                                      self.fusing.is_overload(),
                                      self.tracking.is_overload()))
                    return False
            else:
                break

        return True

    def pre_process(self, data_dict):
        data_dict.pop('image_det', None)
        data_dict.pop('pred_boxes', None)
        data_dict.pop('pred_attr', None)
        data_dict.pop('pred_traj', None)
        return data_dict

    def enqueue(self, input_dict, module_name):
        input_dict['do_detection'] = self.engine.enqueue(input_dict)
        self.frame_queue.put_nowait(input_dict)

    def get_data(self, **kwargs):
        # wait until get points or thread quit
        while self.system.is_initialized:
            try:
                frame_dict = self.frame_queue.get(block=True, timeout=1.0)
                break
            except queue.Empty:
                self.logger.warn('frame queue is Empty')
                frame_dict = dict()
                continue
        # wait until get tracking result or thread quit
        while self.system.is_initialized and frame_dict['do_detection'] and (frame_dict['image_valid'] or frame_dict['lidar_valid']):
            track_dict = self.tracking.get_output(block=True, timeout=1.0)
            if track_dict:
                if frame_dict['frame_start_timestamp'] != track_dict['frame_start_timestamp']:
                    self.logger.warn('tracking output order is wrong!')
                else:
                    track_dict.pop('lidar_valid', None)
                    track_dict.pop('image_valid', None)
                    frame_dict.update(track_dict)
                    break
            else:
                self.logger.warn('tracking queue is Empty')

        if not self.system.is_initialized:
            return dict()

        frame_dict = self.object_filter.filter(frame_dict)
        frame_dict.pop('do_detection')

        return frame_dict