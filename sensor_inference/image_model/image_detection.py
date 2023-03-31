import os

import numpy as np
import torch
import torch.nn as nn

from sensor_inference.image_model.yolo_pafpn import YOLOPAFPN
from sensor_inference.image_model.rtm3d_head import RTM3DHEAD

class ImageDetection(nn.Module):
    def __init__(self, model_cfg, num_class):
        super().__init__()
        self.model_cfg = model_cfg
        self.num_class = num_class
        self.build_backbone_networks(model_cfg)
        self.build_rpn_networks()

    def forward(self, image):
        image = image / 255.
        images = image.permute(0, 3, 1, 2)

        backbone_x = self.backbone_2d(images)
        return self.dense_head(backbone_x)

    def build_backbone_networks(self, model_cfg):
        module = self.build_backbone(model_cfg=model_cfg)
        self.add_module('backbone_2d', module)

    def build_backbone(self, model_cfg):
        backbone = YOLOPAFPN(model_cfg=model_cfg, input_channels=3)
        return backbone

    def build_rpn_networks(self):
        module = self.build_dense_head()
        self.add_module('dense_head', module)

    def build_dense_head(self):
        if self.model_cfg.get('DENSE_HEAD', None) is None:
            return None
        dense_head_module = RTM3DHEAD(
            model_cfg=self.model_cfg.DENSE_HEAD,
            image_shape=self.model_cfg.IMG_SHAPE,
            downscale=self.model_cfg.OUTPUT_DOWN_SCALE,
            num_cls=self.num_class,
            input_channels=self.backbone_2d.num_bev_features,
        )
        return dense_head_module

    def load_params_from_file(self, filename, to_cpu=False):
        if not os.path.isfile(filename):
            raise FileNotFoundError

        print('==> Loading parameters from checkpoint %s to %s' % (filename, 'CPU' if to_cpu else 'GPU'))
        loc_type = torch.device('cpu') if to_cpu else None
        checkpoint = torch.load(filename, map_location=loc_type)
        model_state_disk = checkpoint['model_state']

        if 'version' in checkpoint:
            print('==> Checkpoint trained from version: %s' % checkpoint['version'])

        update_model_state = {}
        for key, val in model_state_disk.items():
            if key in self.state_dict() and self.state_dict()[key].shape == model_state_disk[key].shape:
                update_model_state[key] = val

        state_dict = self.state_dict()
        state_dict.update(update_model_state)
        self.load_state_dict(state_dict)

        for key in state_dict:
            if key not in update_model_state:
                print('Not updated weight %s: %s' % (key, str(state_dict[key].shape)))

        print('==> Done (loaded %d/%d)' % (len(update_model_state), len(self.state_dict())))