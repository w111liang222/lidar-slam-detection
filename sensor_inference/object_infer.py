import os
import copy
import numpy as np

from sensor_inference.infer_base import InferBase
from sensor_inference.utils.config import cfg, cfg_from_yaml_file

def parse_config(cfg_file):
    cfg_from_yaml_file(cfg_file, cfg)
    return cfg

def build_engine(config, scn_file, rpn_file):
    from sensor_driver.inference.inference import inference_init, inference_forward
    inference_init(
        scn_file       = scn_file,
        rpn_file       = rpn_file,
        voxel_size     = config.VOXELIZATION.VOXEL_SIZE,
        coors_range    = np.array(config.POINT_CLOUD_RANGE, dtype=np.float32),
        max_points     = config.VOXELIZATION.MAX_POINTS_PER_VOXEL,
        max_voxels     = config.VOXELIZATION.MAX_NUMBER_OF_VOXELS['test'],
        max_points_use = config.VOXELIZATION.MAX_POINTS,
        frame_num      = config.POINT_FRAME_NUM,
    )

    def engine(points, motion_t, realtime):
        return inference_forward(points, motion_t, realtime)

    return engine

def reset_engine():
    from sensor_driver.inference.inference import inference_reset
    return inference_reset()

class ObjectInfer(InferBase):
    def __init__(self, engine_start, cfg_file = None, logger = None, max_size = 3):
        super().__init__('Object', engine_start, cfg_file, logger, max_size)

    def initialize(self):
        self.cfg_file = self.cfg_file if os.path.exists(self.cfg_file) else 'sensor_inference/cfgs/detection_object.yaml'
        self.cfg = copy.deepcopy(parse_config(self.cfg_file))
        self.create_queue()

    def build_engine(self):
        from sensor_inference.utils.object_post_process import PostProcesser
        self.engine = build_engine(self.cfg.DATA_CONFIG, self.cfg.SCN_ONNX_FILE, self.cfg.RPN_TRT_FILE)
        self.post_processer = PostProcesser(model_cfg=self.cfg.MODEL,
                                            num_class=len(self.cfg.CLASS_NAMES),
                                            class_names=self.cfg.CLASS_NAMES)

    def reset_engine(self):
        reset_engine()

    def prepare_data(self, data_dict):
        # pop out non-relative data of lidar
        data_dict.pop('image', None)
        data_dict.pop('image_param', None)

        # seperate the data dict
        points = data_dict.pop('points', None)

        lidar_data = dict()
        for sensor in self.base_cfg.detection.sensor_input:
            if sensor in points:
                lidar_data[sensor] = points[sensor]

        return {'lidar_data' : lidar_data, 'infos' : data_dict}

    def process(self, data_dict):
        if not data_dict:
            return None

        # prepare
        motion_t = np.ascontiguousarray(data_dict['infos']['motion_t'], dtype=np.float32)
        points = np.concatenate(list(data_dict['lidar_data'].values()), axis=0) if bool(data_dict['lidar_data']) else np.zeros((1, 4), dtype=np.float32)
        points = np.concatenate((points, np.zeros((points.shape[0], 1))), axis=1)
        cls_preds, box_preds, label_preds, freespace = self.engine(points, motion_t, True)

        # postprocess
        pred_dicts = self.post_processer.forward(cls_preds, box_preds, label_preds, freespace)
        data_dict['infos'].update(pred_dicts)


        data_dict['infos'].pop('points_attr', None)
        data_dict['infos'].pop('ins_data', None)
        data_dict['infos'].pop('imu_data', None)
        result = {'object' : data_dict['infos']}
        return result