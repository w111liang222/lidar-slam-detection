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

#include "point2voxel.h"
namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(point2voxel_ext, m) {
  m.def("points_to_voxel_3d_np", &spconv::points_to_voxel_3d_np<float, 3>,
        "matrix tensor_square", "points"_a = 1, "voxels"_a = 2,
        "voxel_point_mask"_a = 3, "coors"_a = 4,
        "coor_to_voxelidx"_a = 5, "voxel_size"_a = 6, "coors_range"_a = 7,
        "max_points"_a = 8, "max_voxels"_a = 9, "shuffle_idx"_a = 10, "max_points_use"_a = 11);
  m.def("points_to_voxel_3d_np", &spconv::points_to_voxel_3d_np<double, 3>,
        "matrix tensor_square", "points"_a = 1, "voxels"_a = 2,
        "voxel_point_mask"_a = 3, "coors"_a = 4,
        "coor_to_voxelidx"_a = 5, "voxel_size"_a = 6, "coors_range"_a = 7,
        "max_points"_a = 8, "max_voxels"_a = 9, "shuffle_idx"_a = 10, "max_points_use"_a = 11);
}