import torch
from torch import nn

from .center_point_head import CenterHead
from .segment_head_bev import BEVSegmentationHead

class BEVDetectionSegmentationHead(nn.Module):
    def __init__(
        self,
        model_cfg, input_channels, num_class, class_names, grid_size, point_cloud_range, voxel_size, predict_boxes_when_training=True,
    ):
        super(BEVDetectionSegmentationHead, self).__init__()
        self.detection_head = CenterHead(model_cfg.DETECTION, input_channels, num_class, class_names, grid_size, point_cloud_range, voxel_size, predict_boxes_when_training)
        self.segment_head = BEVSegmentationHead(model_cfg.SEGMENT, input_channels, num_class, class_names, grid_size, point_cloud_range, voxel_size, predict_boxes_when_training)

    def forward(self, x):
        batch_cls_preds, batch_box_preds = self.detection_head(x)
        masks_bev = self.segment_head(x)
        return batch_cls_preds, batch_box_preds, masks_bev
