import numpy as np

from sensor_inference.dataset.data_processor import DataProcessor
from sensor_inference.dataset.point_feature_encoder import PointFeatureEncoder

class DemoDataset():
    def __init__(self, dataset_cfg, class_names, training=True):
        """
        Args:
            dataset_cfg:
            class_names:
            training:
        """
        self.dataset_cfg = dataset_cfg
        self.training = training
        self.class_names = class_names
        if self.dataset_cfg is None or class_names is None:
            return

        self.point_cloud_range = np.array(self.dataset_cfg.POINT_CLOUD_RANGE, dtype=np.float32)
        self.point_feature_encoder = PointFeatureEncoder(
            self.dataset_cfg.POINT_FEATURE_ENCODING,
            point_cloud_range=self.point_cloud_range
        )
        self.data_processor = DataProcessor(
            self.dataset_cfg.DATA_PROCESSOR, point_cloud_range=self.point_cloud_range, training=self.training
        )

        self.grid_size = self.data_processor.grid_size
        self.voxel_size = self.data_processor.voxel_size

    def prepare_data(self, points):
        """
        Args:
            data_dict:
                points: (N, 3 + C_in)
                gt_boxes: optional, (N, 7 + C) [x, y, z, dx, dy, dz, heading, ...]
                gt_names: optional, (N), string
                ...

        Returns:
            data_dict:
                frame_id: string
                points: (N, 3 + C_in)
                gt_boxes: optional, (N, 7 + C) [x, y, z, dx, dy, dz, heading, ...]
                gt_names: optional, (N), string
                voxels: optional (num_voxels, max_points_per_voxel, 3 + C)
                voxel_coords: optional (num_voxels, 3)
                voxel_num_points: optional (num_voxels)
                ...
        """
        points = self.point_feature_encoder.forward(points)

        data_dict, points = self.data_processor.forward(points)

        return data_dict, points

class RealtimeDataset():
    def __init__(self, dataset_cfg, class_names, training=True):
        """
        Args:
            dataset_cfg:
            class_names:
            training:
        """
        self.dataset_cfg = dataset_cfg
        self.training = training
        self.class_names = class_names
        if self.dataset_cfg is None or class_names is None:
            return

        self.point_cloud_range = np.array(self.dataset_cfg.POINT_CLOUD_RANGE, dtype=np.float32)

        self.data_processor = DataProcessor(
            self.dataset_cfg.DATA_PROCESSOR, point_cloud_range=self.point_cloud_range, training=self.training, realtime=True
        )

        self.grid_size = self.data_processor.grid_size
        self.voxel_size = self.data_processor.voxel_size

    def prepare_data(self, points):
        """
        Args:
            data_dict:
                points: (N, 3 + C_in)
                gt_boxes: optional, (N, 7 + C) [x, y, z, dx, dy, dz, heading, ...]
                gt_names: optional, (N), string
                ...

        Returns:
            data_dict:
                frame_id: string
                points: (N, 3 + C_in)
                gt_boxes: optional, (N, 7 + C) [x, y, z, dx, dy, dz, heading, ...]
                gt_names: optional, (N), string
                voxels: optional (num_voxels, max_points_per_voxel, 3 + C)
                voxel_coords: optional (num_voxels, 3)
                voxel_num_points: optional (num_voxels)
                ...
        """

        data_dict, points = self.data_processor.forward(points)

        return data_dict

    @staticmethod
    def prepare_image_data(images, cam_cfg):
        if images is None or not bool(images):
            return None, None

        image_name = sorted(images.keys())[0]
        image_data = images[image_name]
        return image_name, image_data