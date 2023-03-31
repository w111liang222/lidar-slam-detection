import copy
import numpy as np
from sensor_inference.infer_base import InferBase
from sensor_inference.utils.config import cfg, cfg_from_yaml_file

from util.image_util import cvt_image

def parse_config(cfg_file):
    cfg_from_yaml_file(cfg_file, cfg)
    return cfg

def build_image_engine(onnx_file, trt_file, serialize_engine, calib=None):
    if serialize_engine:
        import onnx
        from sensor_inference.trt_engine.prepare_engine import PrepareBackend
        model = onnx.load(onnx_file)
        engine = PrepareBackend(model, build_fp16=True, trt_path=trt_file, serialize_engine=serialize_engine, calib=calib, use_DLA=True)
    else:
        from sensor_inference.trt_engine.runtime_engine import RuntimeBackend as backend
        engine = backend(trt_path=trt_file)
    return engine

def image_infer(engine, input_tuple):
    score_preds, indices, label_preds, hm, kps, dim, rot = engine.run(input_tuple)
    return hm, kps, dim, rot, score_preds, label_preds, indices

class ImageInfer(InferBase):
    def __init__(self, engine_start, cfg_file = None, serialize_engine = False, logger = None, max_size = 3):
        super().__init__('imageDet', engine_start, cfg_file, serialize_engine, logger, max_size)

    def initialize(self):
        if self.cfg_file is not None:
            self.cfg = copy.deepcopy(parse_config(self.cfg_file))
        self.create_queue()

    def build_engine(self, calib):
        from sensor_inference.image_model.post_process import ImagePostProcesser
        self.engine = build_image_engine(self.cfg.ONNX_FILE,
                                         self.cfg.TRT_FILE,
                                         self.serialize_engine,
                                         calib)
        self.post_processer = ImagePostProcesser(model_cfg=self.cfg.MODEL,
                                                 num_class=len(self.cfg.CLASS_NAMES))

    def prepare_data(self, data_dict):
        from sensor_inference.dataset.demo_dataset import RealtimeDataset
        # pop out non-relative data of image
        data_dict.pop('points', None)
        data_dict.pop('points_attr', None)

        # seperate the data dict
        images = data_dict.pop('image', None)
        image_name, image_data = RealtimeDataset.prepare_image_data(images, self.cfg)
        image_dict = {'image_data' : image_data, 'image_name' : image_name, 'infos' : data_dict}
        return image_dict

    def process(self, data_dict):
        if not data_dict or not data_dict['infos']['image_valid']:
            return None

        image, image_name = data_dict['image_data'], data_dict['image_name']

        # preprocess
        image = cvt_image(image, self.cfg.MODEL.IMG_SHAPE[1], self.cfg.MODEL.IMG_SHAPE[0])
        image = np.expand_dims(image, axis=0).astype(np.float32)

        # cnn
        hm, kps, dim, rot, score_preds, label_preds, indices = image_infer(self.engine, [image])

        # postprocess
        self.post_processer.set_camera_parameter(data_dict['infos']['image_param'])
        result_dicts = self.post_processer.forward(image_name, hm, kps, dim, rot, score_preds, label_preds, indices)

        data_dict['infos']['image_det'] = {image_name : result_dicts}
        result = {'image' : data_dict['infos']}
        return result