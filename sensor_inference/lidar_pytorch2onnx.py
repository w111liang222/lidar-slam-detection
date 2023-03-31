import sys
import os
sys.path.append(os.getcwd())

import argparse
from pathlib import Path

import mayavi.mlab as mlab
import numpy as np
import torch
import torch.onnx
import time
import onnx
from onnxsim import simplify

from dataset.point_loader import PointLoader
from dataset.demo_dataset import DemoDataset
from utils.config import cfg, cfg_from_yaml_file
from utils.visualize_utils import draw_scenes
from model.point_pillar import PointPillar
from model.post_process import PostProcesser

def load_data_to_gpu(batch_dict):
    for key, val in batch_dict.items():
        if not isinstance(val, np.ndarray):
            continue
        batch_dict[key] = torch.from_numpy(val).float().cuda()

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default=None,
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')
    parser.add_argument('--ext', type=str, default='.bin', help='specify the extension of your point cloud data file')

    args = parser.parse_args()
    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg

def main():
    args, cfg = parse_config()
    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
    )
    point_loader = PointLoader(root_path=Path(args.data_path), ext=args.ext)

    model = PointPillar(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    model.load_params_from_file(filename=args.ckpt, to_cpu=True)
    model.cuda()
    model.eval()

    post_processer = PostProcesser(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)

    with torch.no_grad():
        points = point_loader.getitem(0)
        start_time_s = time.time()
        data_dict, points = demo_dataset.prepare_data(points)
        load_data_to_gpu(data_dict)

        voxel_features, coords, mask = data_dict['voxels'], data_dict['coordinates'], data_dict['voxel_point_mask']

        input_tuple = list(data_dict.values())

        start_time = time.time()
        cls_preds, box_preds, label_preds = model(input_tuple)
        cls_preds = cls_preds.cpu().numpy()
        box_preds = box_preds.cpu().numpy()
        label_preds = label_preds.cpu().numpy()
        end_time = time.time()
        print('pointpillar time is : ', (end_time - start_time))

        input_names = ["voxel_features", "coords", "mask"]
        output_names = ["cls_preds", "box_preds", "label_preds"]
        dynamic_ax = {'voxel_features' : {1 : 'voxel_num'},
                      'coords' : {1 : 'voxel_num'},
                      'mask' : {1 : 'voxel_num'}}
        torch.onnx.export(model, input_tuple, cfg.ONNX_FILE, verbose=False, export_params=True, opset_version=10, input_names=input_names, output_names=output_names, dynamic_axes=dynamic_ax)

        onnx_model = onnx.load(cfg.ONNX_FILE)
        input_shape = {
            'voxel_features': voxel_features.shape,
            'coords': coords.shape,
            'mask': mask.shape,
        }
        model_simp, check = simplify(onnx_model, test_input_shapes=input_shape)
        assert check, "Simplified ONNX model could not be validated"
        onnx.save(model_simp, cfg.ONNX_FILE)

        pred_dicts = post_processer.forward(cls_preds, box_preds, label_preds)
        end_time_s = time.time()
        print('inference time is : ', (end_time_s - start_time_s))

        # points.tofile('sensor_inference/test_points.npy')
        pred_dicts['pred_scores'] = pred_dicts['pred_scores'].reshape(pred_dicts['pred_scores'].shape[0])
        pred_dicts['pred_labels'] = pred_dicts['pred_labels'].reshape(pred_dicts['pred_labels'].shape[0])
        draw_scenes(
            points=points, ref_boxes=pred_dicts['pred_boxes'],
            ref_scores=pred_dicts['pred_scores'], ref_labels=pred_dicts['pred_labels'],
            class_names=cfg.CLASS_NAMES,
        )
        mlab.show(stop=True)

if __name__ == '__main__':
    main()