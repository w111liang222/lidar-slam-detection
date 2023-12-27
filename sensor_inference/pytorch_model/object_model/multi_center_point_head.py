import torch
import copy
from torch import double, nn

from .center_point_head import CenterHead

class MultiCenterHead(nn.Module):
    def __init__(self, model_cfg, input_channels, num_class, class_names, grid_size, point_cloud_range, voxel_size,
                 predict_boxes_when_training=True, **kwargs):
        super(MultiCenterHead, self).__init__()
        model_cfg_1x = copy.deepcopy(model_cfg)
        model_cfg_4x = copy.deepcopy(model_cfg)
        model_cfg_4x.FEATURE_MAP_STRIDE = model_cfg_4x.FEATURE_MAP_STRIDE * 4

        input_channels_1x = input_channels * 1
        input_channels_4x = input_channels * 4
        self.center_head_1x  = CenterHead(model_cfg_1x, input_channels_1x, num_class, class_names, grid_size, point_cloud_range, voxel_size,
                                          predict_boxes_when_training, **kwargs)

        self.center_head_4x  = CenterHead(model_cfg_4x, input_channels_4x, num_class, class_names, grid_size, point_cloud_range, voxel_size,
                                          predict_boxes_when_training, **kwargs)

    def forward(self, x, x_4x):
        batch_cls_preds_1x, batch_box_preds_1x = self.center_head_1x(x)
        batch_cls_preds_4x, batch_box_preds_4x = self.center_head_4x(x_4x)

        batch_cls_preds = torch.cat((batch_cls_preds_1x, batch_cls_preds_4x), dim=0)
        batch_box_preds = torch.cat((batch_box_preds_1x, batch_box_preds_4x), dim=0)

        return batch_cls_preds, batch_box_preds