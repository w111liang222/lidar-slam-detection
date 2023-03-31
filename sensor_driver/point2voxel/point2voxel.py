import numpy as np

from point2voxel_ext import points_to_voxel_3d_np

def points_to_voxel(points,
                    voxel_size,
                    coors_range,
                    coor_to_voxelidx,
                    shuffle_idx,
                    max_points_use,
                    max_points=35,
                    max_voxels=20000):
    """convert 3d points(N, >=3) to voxels. This version calculate
    everything in one loop. now it takes only 0.8ms(~6k voxels)
    with c++ and 3.2ghz cpu.

    Args:
        points: [N, ndim] float tensor. points[:, :3] contain xyz points and
            points[:, 3:] contain other information such as reflectivity.
        voxel_size: [3] list/tuple or array, float. xyz, indicate voxel size
        coors_range: [6] list/tuple or array, float. indicate voxel range.
            format: xyzxyz, minmax
        coor_to_voxelidx: int array. used as a dense map.
        max_points: int. indicate maximum points contained in a voxel.
        max_voxels: int. indicate maximum voxels this function create.
            for voxelnet, 20000 is a good choice. you should shuffle points
            before call this function because max_voxels may drop some points.
        full_mean: bool. if true, all empty points in voxel will be filled with mean
            of exist points.
        block_filtering: filter voxels by height. used for lidar point cloud.
            use some visualization tool to see filtered result.
    Returns:
        voxels: [M, max_points, ndim] float tensor. only contain points.
        coordinates: [M, 3] int32 tensor. zyx format.
        num_points_per_voxel: [M] int32 tensor.
    """
    voxels = np.zeros(shape=(max_voxels, max_points, points.shape[-1]),
                      dtype=points.dtype)
    voxel_point_mask = np.zeros(shape=(max_voxels, max_points, 1),
                                dtype=points.dtype)
    coors = np.zeros(shape=(max_voxels, 3), dtype=np.int32)
    res = {
        "voxels": voxels,
        "coordinates": coors,
        "voxel_point_mask": voxel_point_mask,
    }

    voxel_num = points_to_voxel_3d_np(points, voxels, voxel_point_mask,
                                        coors,
                                        coor_to_voxelidx,
                                        voxel_size.tolist(),
                                        coors_range.tolist(), max_points,
                                        max_voxels,
                                        shuffle_idx,
                                        max_points_use)
    voxel_num = max(1, voxel_num)
    return res, voxel_num

class VoxelGeneratorV2:
    def __init__(self,
                 voxel_size,
                 point_cloud_range,
                 max_num_points,
                 max_voxels=20000,
                 pad_output=False):
        point_cloud_range = np.array(point_cloud_range, dtype=np.float32)
        voxel_size = np.array(voxel_size, dtype=np.float32)
        grid_size = (point_cloud_range[3:] -
                     point_cloud_range[:3]) / voxel_size
        grid_size = np.round(grid_size).astype(np.int64)

        voxelmap_shape = tuple(np.round(grid_size).astype(np.int32).tolist())
        voxelmap_shape = voxelmap_shape[::-1]
        self._coor_to_voxelidx = np.full(voxelmap_shape, -1, dtype=np.int32)
        self._voxel_size = voxel_size
        self._point_cloud_range = point_cloud_range
        self._max_num_points = max_num_points
        self._max_voxels = max_voxels
        self._grid_size = grid_size
        self._pad_output = pad_output

    def generate(self, points, shuffle_idx, max_points_use):
        res, voxel_num = points_to_voxel(points, self._voxel_size,
                              self._point_cloud_range, self._coor_to_voxelidx,
                              shuffle_idx, max_points_use,
                              self._max_num_points, self._max_voxels)
        if self._pad_output:
            return res
        for k, v in res.items():
            res[k] = v[:voxel_num]
        return res

    @property
    def voxel_size(self):
        return self._voxel_size

    @property
    def max_num_points_per_voxel(self):
        return self._max_num_points

    @property
    def point_cloud_range(self):
        return self._point_cloud_range

    @property
    def grid_size(self):
        return self._grid_size
