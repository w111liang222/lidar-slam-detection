import torch
import torch.nn as nn

class PointPillarScatter(nn.Module):
    def __init__(self, model_cfg, grid_size, **kwargs):
        super().__init__()
        self.model_cfg = model_cfg
        self.num_bev_features = self.model_cfg.NUM_BEV_FEATURES
        self.nx, self.ny, self.nz = grid_size
        assert self.nz == 1

    def forward(self, pillar_features, coords):
        spatial_feature = torch.zeros(
            self.num_bev_features,
            self.nz * self.nx * self.ny,
            dtype=pillar_features.dtype,
            device=pillar_features.device)

        indices = coords[:, :, 0] + coords[:, :, 1] * self.nx + coords[:, :, 2]
        ones = torch.ones(self.num_bev_features, 1, dtype=torch.float32, device=pillar_features.device)
        indices_num_channel = torch.matmul(ones, indices)
        indices_num_channel = indices_num_channel.type(torch.long)

        spatial_feature = spatial_feature.scatter_(1, indices_num_channel, pillar_features)
        spatial_feature = spatial_feature.view(1, self.num_bev_features * self.nz, self.ny, self.nx)

        return spatial_feature
