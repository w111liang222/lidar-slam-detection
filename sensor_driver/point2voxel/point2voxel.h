// Copyright 2019 Yan Yan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <pybind11/pybind11.h>
// must include pybind11/eigen.h if using eigen matrix as arguments.
// must include pybind11/stl.h if using containers in STL in arguments.
#include <algorithm>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
// #include <vector>
#include <iostream>
#include <math.h>
#include <random>
#include <chrono>
#include <numeric>

namespace spconv {
namespace py = pybind11;
using namespace pybind11::literals;

template <typename DType, int NDim>
int points_to_voxel_3d_np(py::array_t<DType> points, py::array_t<DType> voxels,
                          py::array_t<DType> voxel_point_mask,
                          py::array_t<int> coors,
                          py::array_t<int> coor_to_voxelidx,
                          std::vector<DType> voxel_size,
                          std::vector<DType> coors_range, int max_points,
                          int max_voxels,
                          py::array_t<int> shuffle_idx,
                          int max_points_use) {
  auto points_rw = points.template mutable_unchecked<2>();
  auto voxels_rw = voxels.template mutable_unchecked<3>();
  auto voxel_point_mask_rw = voxel_point_mask.template mutable_unchecked<3>();
  auto coors_rw = coors.mutable_unchecked<2>();
  std::vector<int> num_points_per_voxel_rw(max_voxels, 0);
  auto coor_to_voxelidx_rw = coor_to_voxelidx.mutable_unchecked<NDim>();
  auto shuffle_idx_rw = shuffle_idx.template mutable_unchecked<1>();
  auto N = max_points_use;

  auto num_features = points_rw.shape(1);
  int voxel_num = 0;
  int coor[NDim];
  int c;
  int grid_size[NDim];
  for (int i = 0; i < NDim; ++i) {
    grid_size[i] =
        round((coors_range[NDim + i] - coors_range[i]) / voxel_size[i]);
  }
  int voxelidx, num, i;
  for (int j = 0; j < N; ++j) {
    i = shuffle_idx_rw(j);

    // dim 0
    c = floor((points_rw(i, 0) - coors_range[0]) / voxel_size[0]);
    if ((c < 0 || c >= grid_size[0])) {
      continue;
    }
    coor[2] = c;

    // dim 1
    c = floor((points_rw(i, 1) - coors_range[1]) / voxel_size[1]);
    if ((c < 0 || c >= grid_size[1])) {
      continue;
    }
    coor[1] = c;

    // dim 2
    c = floor((points_rw(i, 2) - coors_range[2]) / voxel_size[2]);
    if ((c < 0 || c >= grid_size[2])) {
      continue;
    }
    coor[0] = c;

    voxelidx = coor_to_voxelidx_rw(coor[0], coor[1], coor[2]);
    if (voxelidx == -1) {
      voxelidx = voxel_num;
      if (voxel_num >= max_voxels)
        continue;
      voxel_num += 1;
      coor_to_voxelidx_rw(coor[0], coor[1], coor[2]) = voxelidx;
      for (int k = 0; k < NDim; ++k) {
        coors_rw(voxelidx, k) = coor[k];
      }
    }
    num = num_points_per_voxel_rw[voxelidx];
    if (num < max_points) {
      voxel_point_mask_rw(voxelidx, num, 0) = DType(1);
      for (int k = 0; k < num_features; ++k) {
        voxels_rw(voxelidx, num, k) = points_rw(i, k);
      }
      ++num_points_per_voxel_rw[voxelidx];
    }
  }
  for (int i = 0; i < voxel_num; ++i) {
    coor_to_voxelidx_rw(coors_rw(i, 0), coors_rw(i, 1), coors_rw(i, 2)) = -1;
  }
  return voxel_num;
}

} // namespace spconv
