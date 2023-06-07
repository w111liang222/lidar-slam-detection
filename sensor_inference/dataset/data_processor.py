from functools import partial

import numpy as np

from sensor_inference.utils import common_utils


class DataProcessor(object):
    def __init__(self, processor_configs, point_cloud_range, training, realtime=False):
        self.point_cloud_range = point_cloud_range
        self.training = training
        self.mode = 'train' if training else 'test'
        self.grid_size = self.voxel_size = None
        self.realtime = realtime
        self.data_processor_queue = []

        # shuffle idx table, max points 500,000
        self.shuffle_idxs = []
        for i in range(500):
            shuffle_idx = np.random.permutation((i + 1) * 1000)
            self.shuffle_idxs.append(shuffle_idx)
        for cur_cfg in processor_configs:
            cur_processor = getattr(self, cur_cfg.NAME)(config=cur_cfg)
            self.data_processor_queue.append(cur_processor)

    def mask_points_and_boxes_outside_range(self, points=None, data_dict=None, config=None):
        if points is None:
            return partial(self.mask_points_and_boxes_outside_range, config=config)
        if self.realtime:
            return points, data_dict
        mask = common_utils.mask_points_by_range(points, self.point_cloud_range)
        points = points[mask]
        return points, data_dict

    def shuffle_points(self, points=None, data_dict=None, config=None):
        if points is None:
            return partial(self.shuffle_points, config=config)

        if config.SHUFFLE_ENABLED[self.mode]:
            shuffle_idx = np.random.permutation(points.shape[0])
            points = points[shuffle_idx]

        return points, data_dict

    def transform_points_to_voxels(self, points=None, data_dict=None, config=None, voxel_generator=None):
        if points is None:
            try:
                from sensor_driver.point2voxel.point2voxel import VoxelGeneratorV2 as VoxelGenerator
            except:
                from sensor_driver.point2voxel.point2voxel import VoxelGenerator

            voxel_generator = VoxelGenerator(
                voxel_size=config.VOXEL_SIZE,
                point_cloud_range=self.point_cloud_range,
                max_num_points=config.MAX_POINTS_PER_VOXEL,
                max_voxels=config.MAX_NUMBER_OF_VOXELS[self.mode],
                pad_output=False
            )
            grid_size = (self.point_cloud_range[3:6] - self.point_cloud_range[0:3]) / np.array(config.VOXEL_SIZE)
            self.grid_size = np.round(grid_size).astype(np.int64)
            self.voxel_size = config.VOXEL_SIZE
            self.max_number_of_voxels = config.MAX_NUMBER_OF_VOXELS[self.mode]
            self.max_points_per_voxel = config.MAX_POINTS_PER_VOXEL
            return partial(self.transform_points_to_voxels, config=config, voxel_generator=voxel_generator)

        num_points = points.shape[0]
        if num_points < 1000:
            shuffle_idx = np.random.permutation(num_points)
        else:
            idx = min(int(num_points / 1000), 500) - 1
            shuffle_idx = self.shuffle_idxs[idx]
        voxel_output = voxel_generator.generate(points, shuffle_idx, config.MAX_POINTS)
        if isinstance(voxel_output, dict):
            voxels, coordinates, mask = \
                voxel_output['voxels'], voxel_output['coordinates'], voxel_output["voxel_point_mask"]

        data_dict['voxels'] = np.expand_dims(voxels, axis=0)
        data_dict['coordinates'] = np.expand_dims(coordinates, axis=0)
        data_dict['voxel_point_mask'] = np.expand_dims(mask, axis=0)
        return points, data_dict

    def forward(self, points):
        """
        Args:
            data_dict:
                points: (N, 3 + C_in)
                gt_boxes: optional, (N, 7 + C) [x, y, z, dx, dy, dz, heading, ...]
                gt_names: optional, (N), string
                ...

        Returns:
        """
        data_dict = dict()
        for cur_processor in self.data_processor_queue:
            points, data_dict = cur_processor(points=points, data_dict=data_dict)

        return data_dict, points
