import torch
from torch import nn
from torch.nn.init import kaiming_normal_
import copy

import numpy as np
import math

def atan2(y, x):
    ans = torch.atan(y/x)
    mask = ((y>0) & (x<0)) | ((y<0) & (x<0))    # upper left quadrant, lower left quadrant
    ans = torch.where(mask, ans+math.pi, ans)
    # upper right quadrant and lower right quadrant, do nothing
    return ans

class SeparateHead(nn.Module):
    def __init__(self, input_channels, sep_head_dict, init_bias=-2.19, use_bias=False):
        super().__init__()
        self.sep_head_dict = sep_head_dict

        for cur_name in self.sep_head_dict:
            output_channels = self.sep_head_dict[cur_name]['out_channels']
            num_conv = self.sep_head_dict[cur_name]['num_conv']

            fc_list = []
            for k in range(num_conv - 1):
                fc_list.append(nn.Sequential(
                    nn.Conv2d(input_channels, input_channels, kernel_size=3, stride=1, padding=1, bias=use_bias),
                    nn.BatchNorm2d(input_channels),
                    nn.ReLU()
                ))
            fc_list.append(nn.Conv2d(input_channels, output_channels, kernel_size=3, stride=1, padding=1, bias=True))
            fc = nn.Sequential(*fc_list)
            if 'hm' in cur_name:
                fc[-1].bias.data.fill_(init_bias)
            else:
                for m in fc.modules():
                    if isinstance(m, nn.Conv2d):
                        kaiming_normal_(m.weight.data)
                        if hasattr(m, "bias") and m.bias is not None:
                            nn.init.constant_(m.bias, 0)

            self.__setattr__(cur_name, fc)

    def forward(self, x):
        ret_dict = {}
        for cur_name in self.sep_head_dict:
            ret_dict[cur_name] = self.__getattr__(cur_name)(x)

        return ret_dict

class CenterHead(nn.Module):
    def __init__(self, model_cfg, input_channels, num_class, class_names, grid_size, point_cloud_range, voxel_size,
                 predict_boxes_when_training=True, **kwargs):
        super(CenterHead, self).__init__()
        self.model_cfg = model_cfg
        self.num_class = num_class
        self.grid_size = grid_size
        self.point_cloud_range = point_cloud_range
        self.voxel_size = voxel_size
        self.feature_map_stride = self.model_cfg.get('FEATURE_MAP_STRIDE', None)
        self.engine = self.model_cfg.get('ENGINE', "")

        self.class_names = class_names
        self.class_names_each_head = []
        self.class_id_mapping_each_head = []

        for cur_class_names in self.model_cfg.CLASS_NAMES_EACH_HEAD:
            self.class_names_each_head.append([x for x in cur_class_names if x in class_names])
            cur_class_id_mapping = torch.from_numpy(np.array(
                [self.class_names.index(x) for x in cur_class_names if x in class_names]
            )).cuda()
            self.class_id_mapping_each_head.append(cur_class_id_mapping)

        total_classes = sum([len(x) for x in self.class_names_each_head])
        assert total_classes == len(self.class_names), f'class_names_each_head={self.class_names_each_head}'

        self.H = int(grid_size[0] / self.feature_map_stride)
        self.W = int(grid_size[1] / self.feature_map_stride)
        xs, ys = np.meshgrid(np.arange(0, self.H), np.arange(0, self.W))
        xs = torch.from_numpy(xs)
        ys = torch.from_numpy(ys)
        self.xs = xs.view(self.H, self.W).repeat(1, 1).view(-1, 1).cuda()
        self.ys = ys.view(self.H, self.W).repeat(1, 1).view(-1, 1).cuda()

        self.shared_conv_channel = 64

        # a shared convolution
        self.shared_conv = nn.Sequential(
            nn.Conv2d(input_channels, self.shared_conv_channel,
            kernel_size=3, padding=1, bias=True),
            nn.BatchNorm2d(self.shared_conv_channel),
            nn.ReLU(inplace=True),
            nn.Conv2d(self.shared_conv_channel, 128, 3, padding=1, bias=True),
            nn.BatchNorm2d(128),
            nn.ReLU(True),
        )

        self.heads_list = nn.ModuleList()

        for num_cls in [num_class]:
            heads = copy.deepcopy(model_cfg.COMMON_HEADS)

            heads['hm'] = dict(out_channels=num_cls, num_conv=2)
            self.heads_list.append(
                SeparateHead(128, heads, init_bias=-2.19, use_bias=True)
            )
        self.forward_ret_dict = {}

    def forward(self, x):
        ret_dicts = []

        x = self.shared_conv(x)

        for task in self.heads_list:
            ret_dicts.append(task(x))

        if self.engine == "TensorRT":
            batch_cls_preds, batch_box_preds = self.predict_tensorrt(ret_dicts)
        elif self.engine == "RKNN":
            batch_cls_preds, batch_box_preds = self.predict_rknn(ret_dicts)
        else:
            raise NotImplementedError

        return batch_cls_preds, batch_box_preds

    @torch.no_grad()
    def predict_tensorrt(self, preds_dicts):

        batch_box_preds_list = []
        batch_hm_list = []
        H = self.H
        W = self.W
        num_cls = self.num_class

        for task_id, preds_dict in enumerate(preds_dicts):
            # convert N C H W to N H W C
            for key, val in preds_dict.items():
                preds_dict[key] = val.permute(0, 2, 3, 1).contiguous()

            batch_hm = torch.sigmoid(preds_dict['hm'])

            batch_dim = torch.exp(preds_dict['dim'])

            batch_rots = preds_dict['rot'][..., 0:1]
            batch_rotc = preds_dict['rot'][..., 1:2]
            batch_reg = preds_dict['center']
            batch_hei = preds_dict['center_z']
            batch_iou = preds_dict['iou']

            batch_rot = atan2(batch_rots, batch_rotc)

            batch_reg = batch_reg.reshape(H*W, 2)
            batch_hei = batch_hei.reshape(H*W, 1)

            batch_rot = batch_rot.reshape(H*W, 1)
            batch_dim = batch_dim.reshape(H*W, 3)
            batch_hm = batch_hm.reshape(H*W, num_cls)

            # multiply together for the final score
            batch_iou = torch.clamp(batch_iou.reshape(H*W, 1), min=0, max=1)
            batch_hm = batch_hm * batch_iou

            xs = self.xs + batch_reg[:, 0:1]
            ys = self.ys + batch_reg[:, 1:2]

            xs = xs * self.feature_map_stride * self.voxel_size[0] + self.point_cloud_range[0]
            ys = ys * self.feature_map_stride * self.voxel_size[1] + self.point_cloud_range[1]

            if 'vel' in preds_dict:
                batch_vel = preds_dict['vel']
                batch_vel = batch_vel.reshape(H*W, 2)
                batch_box_preds = torch.cat([xs, ys, batch_hei, batch_dim, batch_vel, batch_rot], dim=1)
            else:
                batch_box_preds = torch.cat([xs, ys, batch_hei, batch_dim, batch_rot], dim=1)

            batch_box_preds_list.append(batch_box_preds)
            batch_hm_list.append(batch_hm)

        batch_box_preds = torch.cat(batch_box_preds_list, dim=0)
        batch_cls_preds = torch.cat(batch_hm_list, dim=0)

        return batch_cls_preds, batch_box_preds

    @torch.no_grad()
    def predict_rknn(self, preds_dicts):

        batch_box_preds_list = []
        batch_hm_list = []
        H = self.H
        W = self.W
        num_cls = self.num_class

        for task_id, preds_dict in enumerate(preds_dicts):
            # convert N C H W to N H W C
            for key, val in preds_dict.items():
                preds_dict[key] = val.permute(0, 2, 3, 1).contiguous()

            batch_hm = torch.sigmoid(preds_dict['hm'])

            batch_dim = preds_dict['dim']

            batch_rot = preds_dict['rot']
            batch_reg = preds_dict['center']
            batch_hei = preds_dict['center_z']
            batch_iou = preds_dict['iou']

            batch_reg = batch_reg.reshape(H*W, 2)
            batch_hei = batch_hei.reshape(H*W, 1)

            batch_rot = batch_rot.reshape(H*W, 2)
            batch_dim = batch_dim.reshape(H*W, 3)
            batch_hm = batch_hm.reshape(H*W, num_cls)
            batch_iou = batch_iou.reshape(H*W, 1)

            if 'vel' in preds_dict:
                batch_vel = preds_dict['vel']
                batch_vel = batch_vel.reshape(H*W, 2)
                batch_box_preds = torch.cat([batch_reg, batch_hei, batch_dim, batch_vel, batch_rot, batch_iou], dim=1)
            else:
                batch_box_preds = torch.cat([batch_reg, batch_hei, batch_dim, batch_rot, batch_iou], dim=1)

            batch_box_preds_list.append(batch_box_preds)
            batch_hm_list.append(batch_hm)

        batch_box_preds = torch.cat(batch_box_preds_list, dim=0)
        batch_cls_preds = torch.cat(batch_hm_list, dim=0)

        return batch_cls_preds, batch_box_preds
