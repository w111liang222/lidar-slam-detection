import sys
import os
sys.path.append(os.getcwd())

import argparse

import numpy as np
import torch
import torch.onnx
import cv2
import mayavi.mlab as mlab
from easydict import EasyDict
import onnx
from onnxsim import simplify

from utils.config import cfg, cfg_from_yaml_file
from utils.visualize_utils import draw_scenes
from image_model.image_detection import ImageDetection
from image_model.post_process import ImagePostProcesser

def parse_config():
    parser = argparse.ArgumentParser(description='arg parser')
    parser.add_argument('--cfg_file', type=str, default='sensor_inference/cfgs/detection_image.yaml',
                        help='specify the config for demo')
    parser.add_argument('--data_path', type=str, default='demo_data',
                        help='specify the point cloud data file or directory')
    parser.add_argument('--ckpt', type=str, default=None, help='specify the pretrained model')

    args = parser.parse_args()
    cfg_from_yaml_file(args.cfg_file, cfg)
    return args, cfg

def main():
    args, cfg = parse_config()

    model = ImageDetection(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES))
    model.load_params_from_file(filename=args.ckpt, to_cpu=True)
    model.cuda()
    model.eval()

    param = dict()
    param['name'] = '0'
    param['height'] = 352
    param['width'] = 640
    param['extrinsic_parameters'] = [0, 0, 0, -90, 0, 90]
    param['intrinsic_parameters'] = [7.215377e+02, 7.215377e+02, 6.095593e+02, 1.72854e+02, 0 , 0, 0, 0]

    cam_param = [EasyDict(param)]

    post_processer = ImagePostProcesser(model_cfg=cfg.MODEL, num_class=len(cfg.CLASS_NAMES), cam_param=cam_param)
    with torch.no_grad():
        image = cv2.imread(args.data_path)
        images = np.expand_dims(image, axis=0)
        images = torch.from_numpy(images)
        images = images.float().cuda()

        hm, kps, dim, rot, score_preds, label_preds, indices = model(images)
        hm  = hm.cpu().numpy()
        kps = kps.cpu().numpy()
        dim = dim.cpu().numpy()
        rot = rot.cpu().numpy()
        score_preds = score_preds.cpu().numpy()
        label_preds = label_preds.cpu().numpy()
        indices = indices.cpu().numpy()

        input_names = ["image"]
        output_names = ["hm", "kps", "dim", "rot", "score_preds", "label_preds", "indices"]
        torch.onnx.export(model, images, cfg.ONNX_FILE, verbose=False, export_params=True, opset_version=10, input_names=input_names, output_names=output_names)

        onnx_model = onnx.load(cfg.ONNX_FILE)
        model_simp, check = simplify(onnx_model, dynamic_input_shape=False, input_shapes=None)
        assert check, "Simplified ONNX model could not be validated"
        onnx.save(model_simp, cfg.ONNX_FILE)

        pred_dicts = post_processer.forward('0', hm, kps, dim, rot, score_preds, label_preds, indices)

        for bbox in pred_dicts['pred_bboxes']:
            bbox = bbox.astype(np.int32)
            cv2.rectangle(image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 255), 1)

        cv2.imshow('result', image)
        cv2.waitKey(1)

        pred_dicts['pred_scores'] = pred_dicts['pred_scores'].reshape(pred_dicts['pred_scores'].shape[0])
        pred_dicts['pred_labels'] = pred_dicts['pred_labels'].reshape(pred_dicts['pred_labels'].shape[0])
        draw_scenes(
            points=np.zeros((0, 3), dtype=np.float32), ref_boxes=pred_dicts['pred_boxes'],
            ref_scores=pred_dicts['pred_scores'], ref_labels=pred_dicts['pred_labels'],
            class_names=cfg.CLASS_NAMES,
        )
        mlab.show(stop=True)

if __name__ == '__main__':
    main()