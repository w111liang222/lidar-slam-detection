import copy
import numpy as np
from sensor_inference.infer_base import InferBase
from hardware.platform_common import BOARD_NAME, is_jetson

from sensor_inference.utils.config import cfg, cfg_from_yaml_file
def parse_config(cfg_file):
    cfg_from_yaml_file(cfg_file, cfg)
    return cfg

def build_pointpillar_engine_trt(onnx_file, trt_file, serialize_engine, calib=None):
    if serialize_engine:
        import onnx
        from sensor_inference.trt_engine.prepare_engine import PrepareBackend
        model = onnx.load(onnx_file)
        dynamic_shape = dict()
        voxel_features_shapes = [(1,1,20,4), (1,20000,20,4), (1,40000,20,4)]
        dynamic_shape['voxel_features'] = voxel_features_shapes
        coords_shapes = [(1,1,3), (1,20000,3), (1,40000,3)]
        dynamic_shape['coords'] = coords_shapes
        mask_shapes = [(1,1,20,1), (1,20000,20,1), (1,40000,20,1)]
        dynamic_shape['mask'] = mask_shapes
        engine = PrepareBackend(model, build_fp16=True, trt_path=trt_file, serialize_engine=serialize_engine, calib=calib, dynamic_shape=dynamic_shape)
    else:
        from sensor_inference.trt_engine.runtime_engine import RuntimeBackend as backend
        engine = backend(trt_path=trt_file)
    return engine

def build_pointpillar_engine_onnx(onnx_file, trt_file, serialize_engine, calib=None):
    import onnxruntime as ort
    return ort.InferenceSession(onnx_file, providers=['OpenVINOExecutionProvider'], provider_options=[{'enable_dynamic_shapes' : True}])

def pointpillar_infer_trt(engine, input_tuple):
    cls_preds, box_preds, label_preds = engine.run(input_tuple)
    return cls_preds, box_preds, label_preds

def pointpillar_infer_onnx(engine, input_tuple):
    cls_preds, box_preds, label_preds = engine.run(None, {'voxel_features': input_tuple[0],
                                                          'coords': input_tuple[1],
                                                          'mask': input_tuple[2]})
    return cls_preds, box_preds, label_preds

def build_pointpillar_engine(onnx_file, trt_file, serialize_engine, calib=None):
    if is_jetson():
        return build_pointpillar_engine_trt(onnx_file, trt_file, serialize_engine, calib)
    elif BOARD_NAME == "IPC":
        return build_pointpillar_engine_onnx(onnx_file, trt_file, serialize_engine, calib)

def pointpillar_infer(engine, input_tuple):
    if is_jetson():
        return pointpillar_infer_trt(engine, input_tuple)
    elif BOARD_NAME == "IPC":
        return pointpillar_infer_onnx(engine, input_tuple)

class LidarInfer(InferBase):
    def __init__(self, engine_start, cfg_file = None, serialize_engine = False, logger = None, max_size = 3):
        super().__init__('lidarDet', engine_start, cfg_file, serialize_engine, logger, max_size)

    def initialize(self):
        if self.cfg_file is not None:
            self.cfg = copy.deepcopy(parse_config(self.cfg_file))

        self.create_queue()

    def build_engine(self, calib):
        from sensor_inference.dataset.demo_dataset import RealtimeDataset
        from sensor_inference.model.post_process import PostProcesser
        self.engine = build_pointpillar_engine(self.cfg.ONNX_FILE,
                                               self.cfg.TRT_FILE,
                                               self.serialize_engine,
                                               calib)
        self.dataset = RealtimeDataset(dataset_cfg=self.cfg.DATA_CONFIG,
                                       class_names=self.cfg.CLASS_NAMES,
                                       training=False)
        self.post_processer = PostProcesser(model_cfg=self.cfg.MODEL,
                                            num_class=len(self.cfg.CLASS_NAMES),
                                            dataset=self.dataset)

    def prepare_data(self, data_dict):
        # pop out non-relative data of lidar
        data_dict.pop('image', None)
        data_dict.pop('image_param', None)

        # seperate the data dict
        points = data_dict.pop('points', None)
        points_attr = data_dict.pop('points_attr', None)
        lidar_dict = {'lidar_data' : points, 'infos' : data_dict}
        return lidar_dict

    def process(self, data_dict):
        if not data_dict or not data_dict['infos']['lidar_valid']:
            return None

        points = data_dict['lidar_data']

        # preprocess
        lidar_data = self.dataset.prepare_data(np.concatenate(list(points.values()), axis=0))
        lidar_data = list(lidar_data.values())
        lidar_data[1] = lidar_data[1].astype(np.float32)

        # cnn
        cls_preds, box_preds, label_preds = pointpillar_infer(self.engine, lidar_data)

        # postprocess
        pred_dicts = self.post_processer.forward(cls_preds, box_preds, label_preds)
        data_dict['infos'].update(pred_dicts)

        result = {'lidar' : data_dict['infos']}
        return result