from typing import Any, Dict, List, Optional, Tuple, Union

import torch
from torch import nn
from torch.nn import functional as F


class BEVGridTransform(nn.Module):
    def __init__(
        self,
        *,
        input_scope: List[Tuple[float, float, float]],
        output_scope: List[Tuple[float, float, float]],
        prescale_factor: float = 1,
    ) -> None:
        super().__init__()
        self.input_scope = input_scope
        self.output_scope = output_scope

        xbound = output_scope[0]
        ybound = output_scope[1]

        patch_h = ybound[1] - ybound[0]
        patch_w = xbound[1] - xbound[0]
        canvas_h = int(patch_h / ybound[2])
        canvas_w = int(patch_w / xbound[2])

        self.offset_x_min = int((xbound[0] - input_scope[0][0]) / input_scope[0][2])
        self.offset_x_max = int((xbound[1] - input_scope[0][0]) / input_scope[0][2])
        self.offset_y_min = int((ybound[0] - input_scope[1][0]) / input_scope[1][2])
        self.offset_y_max = int((ybound[1] - input_scope[1][0]) / input_scope[1][2])

        self.canvas_size = (canvas_h, canvas_w)
        self.prescale_factor = prescale_factor

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x[:, :, self.offset_y_min:self.offset_y_max, self.offset_x_min:self.offset_x_max]
        x = F.interpolate(
                x,
                size=self.canvas_size,
                mode="bilinear",
                align_corners=False,
            )
        return x

class BEVSegmentationHead(nn.Module):
    def __init__(
        self,
        model_cfg, input_channels, num_class, class_names, grid_size, point_cloud_range, voxel_size, predict_boxes_when_training=True,
    ):
        super(BEVSegmentationHead, self).__init__()
        self.in_channels = input_channels
        self.classes = model_cfg.CLASS_NAMES

        self.transform = BEVGridTransform(**model_cfg.GRID_TRANSFROM)
        self.classifier = nn.Sequential(
            nn.Conv2d(self.in_channels, 64, 3, padding=1, bias=True),
            nn.BatchNorm2d(64),
            nn.ReLU(True),
            nn.Conv2d(64, 128, 3, padding=1, bias=True),
            nn.BatchNorm2d(128),
            nn.ReLU(True),
            nn.Conv2d(128, 128, 3, padding=1, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(True),
            nn.Conv2d(128, len(self.classes), 1),
        )

    def forward(self, x):
        x = self.transform(x)
        x = self.classifier(x)
        masks_bev = torch.sigmoid(x)

        return masks_bev