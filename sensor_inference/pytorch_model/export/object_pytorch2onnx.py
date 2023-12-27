import sys
import os
sys.path.append(os.getcwd())

import numpy as np
import argparse
import torch
import torch.onnx
from torch import nn
import onnx
from onnxsim import simplify

from sensor_inference.utils.config import cfg, cfg_from_yaml_file
from sensor_inference.pytorch_model.object_model.point_pillar import PointPillar

import funcs
import exptool
from sparseconv_quantization import initialize, disable_quantization, quant_sparseconv_module, quant_add_module

def simplify_model(model_path):
    model = onnx.load(model_path)
    if model is None:
        print("File %s is not find! "%model_path)
    return simplify(model)

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default=None,
                        help='specify the config for demo')
    parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')

    args = parser.parse_args()
    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg

class DemoDataset():
    def __init__(self, dataset_cfg, class_names, training=True):
        self.dataset_cfg = dataset_cfg
        self.training = training
        self.class_names = class_names
        if self.dataset_cfg is None or class_names is None:
            return

        self.point_cloud_range = np.array(self.dataset_cfg.POINT_CLOUD_RANGE, dtype=np.float32)
        grid_size = (self.point_cloud_range[3:6] - self.point_cloud_range[0:3]) / np.array(self.dataset_cfg.VOXELIZATION.VOXEL_SIZE)
        self.grid_size = np.round(grid_size).astype(np.int64)
        self.voxel_size = self.dataset_cfg.VOXELIZATION.VOXEL_SIZE

class RPN(nn.Module):
    def __init__(self, model):
        super(RPN, self).__init__()
        self.model = model

    def forward(self, x):
        spatial_features_2d = self.model.backbone_2d(x)
        batch_cls_preds, batch_box_preds = self.model.dense_head(spatial_features_2d)
        cls_preds, box_preds, label_preds = self.model.post_processing(batch_cls_preds, batch_box_preds)

        return cls_preds, box_preds, label_preds

def main():
    initialize()
    args, cfg = parse_config()
    demo_dataset = DemoDataset(
        dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
    )

    model = PointPillar(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    model.load_params_from_file(filename=args.ckpt, to_cpu=True)
    model = model.cuda().eval().half()

    # export SCN
    scn = model.backbone_3d
    scn = funcs.layer_fusion_bn_relu(scn)

    voxels = torch.zeros(1, 5).half().cuda()
    coors  = torch.zeros(1, 4).int().cuda()
    exptool.export_onnx(scn.half(), voxels, coors, 1, None, cfg.SCN_ONNX_FILE, None)

    # export RPN
    model = PointPillar(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), dataset=demo_dataset)
    model.load_params_from_file(filename=args.ckpt, to_cpu=True)
    model = model.cuda().eval()

    spatial_features = model.backbone_3d(torch.zeros(1, 5).cuda(), torch.zeros(1, 4).int().cuda())
    rpn_model = RPN(model)
    rpn_model = rpn_model.cuda().eval()

    input_names = ["spatial_features"]
    output_names = ["cls_preds", "box_preds", "label_preds"]
    torch.onnx.export(rpn_model, spatial_features, cfg.RPN_ONNX_FILE, export_params=True, opset_version=11, do_constant_folding=True,
                      keep_initializers_as_inputs=False, input_names=input_names, output_names=output_names)

    sim_model, check = simplify_model(cfg.RPN_ONNX_FILE)
    assert check, "Simplified ONNX model could not be validated"
    onnx.save(sim_model, cfg.RPN_ONNX_FILE)
    print("export done")

if __name__ == '__main__':
    main()