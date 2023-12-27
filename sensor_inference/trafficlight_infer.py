import os
import copy
import numpy as np
from easydict import EasyDict
from sensor_inference.infer_base import InferBase
from sensor_inference.utils.config import cfg_from_yaml_file
from util.image_util import cvt_image

def parse_config(cfg_file):
    cfg = EasyDict()
    cfg_from_yaml_file(cfg_file, cfg)
    return cfg

def build_engine(trt_file):
    from sensor_inference.trt_engine.runtime_engine import RuntimeBackend as backend
    engine = backend(trt_path=trt_file)
    return engine

class TrafficLightInfer(InferBase):
    def __init__(self, engine_start, cfg_file = None, logger = None, max_size = 3):
        super().__init__('TrafficLight', engine_start, cfg_file, logger, max_size)

    def initialize(self):
        self.cfg_file = self.cfg_file if os.path.exists(self.cfg_file) else 'sensor_inference/cfgs/detection_trafficlight.yaml'
        self.cfg = copy.deepcopy(parse_config(self.cfg_file))
        self.create_queue()

    def build_engine(self, calib):
        from sensor_inference.utils.trafficlight_post_process import PostProcesser
        self.engine = build_engine(self.cfg.TRT_FILE)
        self.post_processer = PostProcesser(model_cfg=self.cfg.MODEL,
                                            num_class=len(self.cfg.CLASS_NAMES),
                                            class_names=self.cfg.CLASS_NAMES)

    def prepare_data(self, data_dict):
        # pop out non-relative data of image
        data_dict.pop('points', None)

        # seperate the data dict
        images = data_dict.pop('image', None)

        image_data = dict()
        for sensor in self.base_cfg.sensor_input:
            if sensor in images:
                image_data[sensor] = images[sensor]
                break

        return {'image_data' : image_data, 'infos' : data_dict}

    def process(self, data_dict):
        if not data_dict:
            return None

        if self.engine is None or not bool(data_dict['image_data']):
            return {'trafficlight' : data_dict['infos']}

        # preprocess
        image_name, image = data_dict['image_data'].popitem()
        padding_width  = data_dict['infos']['image_param'][image_name]['w'] // 32 * 32
        padding_height = data_dict['infos']['image_param'][image_name]['h'] // 32 * 32
        image = cvt_image(image, padding_width, padding_height)
        image_infer = np.expand_dims(image, axis=0).astype(np.float32)

        # cnn inference
        preds = self.engine.run([image_infer])

        # postprocess
        pred_dicts = self.post_processer.forward(image, preds[0])
        pred_dicts['image_name'] = image_name

        data_dict['infos']['trafficlight'] = pred_dicts
        result = {'trafficlight' : data_dict['infos']}
        return result