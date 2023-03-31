import torch
import torch.nn as nn
import torch.nn.functional as F

class PFNLayer(nn.Module):
    def __init__(self,
                 in_channels,
                 out_channels,
                 use_norm=True,
                 last_layer=False):
        super().__init__()
        
        self.last_vfe = last_layer
        self.use_norm = use_norm

        self.linear = nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=1, bias=False)
        if self.use_norm:
            self.norm = nn.BatchNorm2d(out_channels, eps=1e-3, momentum=0.01)

    def forward(self, inputs):
        x = self.linear(inputs)
        x = self.norm(x) if self.use_norm else x
        x = F.relu(x)

        if self.last_vfe:
            x_max = torch.max(x, dim=3, keepdim=True)[0]
            return x_max
        else:
            return x

class PillarVFE(nn.Module):
    def __init__(self, model_cfg, num_point_features, voxel_size, point_cloud_range):
        super().__init__()
        self.model_cfg = model_cfg

        self.use_norm = self.model_cfg.USE_NORM
        self.with_distance = self.model_cfg.WITH_DISTANCE
        self.use_absolute_xyz = self.model_cfg.USE_ABSLOTE_XYZ
        num_point_features += 3 if self.use_absolute_xyz else 0
        if self.with_distance:
            num_point_features += 1

        self.num_filters = self.model_cfg.NUM_FILTERS
        assert len(self.num_filters) > 0
        num_filters = [num_point_features] + list(self.num_filters)

        pfn_layers = []
        for i in range(len(num_filters) - 1):
            in_filters = num_filters[i]
            out_filters = num_filters[i + 1]
            pfn_layers.append(
                PFNLayer(in_filters, out_filters, self.use_norm, last_layer=(i >= len(num_filters) - 2))
            )
        self.pfn_layers = nn.ModuleList(pfn_layers)

        self.voxel_x = voxel_size[0]
        self.voxel_y = voxel_size[1]
        self.voxel_z = voxel_size[2]
        self.x_offset = self.voxel_x / 2 + point_cloud_range[0]
        self.y_offset = self.voxel_y / 2 + point_cloud_range[1]
        self.z_offset = self.voxel_z / 2 + point_cloud_range[2]

    def get_output_feature_dim(self):
        return self.num_filters[-1]

    def forward(self, voxel_features, coords, mask):

        f_center_x = voxel_features[:, :, :, 0] - (coords[:, :, 2].unsqueeze(2) * self.voxel_x + self.x_offset)
        f_center_y = voxel_features[:, :, :, 1] - (coords[:, :, 1].unsqueeze(2) * self.voxel_y + self.y_offset)
        f_center_z = voxel_features[:, :, :, 2] - (coords[:, :, 0].unsqueeze(2) * self.voxel_z + self.z_offset)
        f_center = torch.stack([f_center_x, f_center_y, f_center_z],dim = 3)

        if self.use_absolute_xyz:
            features = [voxel_features, f_center]
        else:
            features = [voxel_features[..., 3:], f_center]

        if self.with_distance:
            points_dist = torch.norm(voxel_features[:, :, :3], 2, 2, keepdim=True)
            features.append(points_dist)
        features = torch.cat(features, dim=-1)
        features *= mask

        features = features.permute(0, 3, 1, 2)
        for pfn in self.pfn_layers:
            features = pfn(features)
        features = features.squeeze(0)
        features = features.squeeze(2)

        return features
