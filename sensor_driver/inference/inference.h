
#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "Logger.h"

namespace py = pybind11;
using namespace pybind11::literals;

#if defined(HAVE_CUDA_ENABLE)
#include "tensorRT/lidar_inference.h"

static std::shared_ptr<LidarInference> s_lidar_engine(nullptr);

void inference_init(std::string scn_file, std::string rpn_file,
                    std::vector<float> voxel_size,
                    std::vector<float> coors_range, int max_points,
                    int max_voxels, int max_points_use, int frame_num) {
  LidarEngineParameter parameter;
  parameter.scn_file = scn_file;
  parameter.rpn_file = rpn_file;

  parameter.proprecess.max_points   = max_points_use;
  parameter.proprecess.num_feature  = 5;
  parameter.proprecess.max_frame_num= frame_num;

  parameter.voxelization.min_range  = nvtype::Float3(coors_range[0], coors_range[1], coors_range[2]);
  parameter.voxelization.max_range  = nvtype::Float3(coors_range[3], coors_range[4], coors_range[5]);
  parameter.voxelization.voxel_size = nvtype::Float3(voxel_size[0], voxel_size[1], voxel_size[2]);
  parameter.voxelization.grid_size  = parameter.voxelization.compute_grid_size(parameter.voxelization.max_range, parameter.voxelization.min_range, parameter.voxelization.voxel_size);
  parameter.voxelization.max_points_per_voxel = max_points;
  parameter.voxelization.max_points = max_points_use;
  parameter.voxelization.max_voxels = max_voxels;
  parameter.voxelization.num_feature = 5;

  s_lidar_engine = create_engine(parameter);
}

void inference_forward(py::array_t<float> points, py::array_t<float> motion, bool runtime,
                       py::array_t<float> cls_preds, py::array_t<float> box_preds, py::array_t<int> label_preds, py::array_t<float> freespace) {
  if (s_lidar_engine == nullptr) {
    LOG_ERROR("Lidar engine is not initialzied");
    return;
  }

  s_lidar_engine->forward(points.data(), int(points.unchecked<2>().shape(0)), motion.data(), runtime);
  s_lidar_engine->get_output(cls_preds.mutable_data(), box_preds.mutable_data(), label_preds.mutable_data(), freespace.mutable_data());
}

int inference_reset() {
  if (s_lidar_engine == nullptr) {
    LOG_ERROR("Lidar engine is not initialzied");
    return -1;
  }

  s_lidar_engine->reset();
  return 0;
}

#else

void inference_init(std::string scn_file, std::string rpn_file,
                    std::vector<float> voxel_size,
                    std::vector<float> coors_range, int max_points,
                    int max_voxels, int max_points_use, int frame_num) {
}

void inference_forward(py::array_t<float> points, py::array_t<float> motion, bool runtime,
                       py::array_t<float> cls_preds, py::array_t<float> box_preds, py::array_t<int> label_preds, py::array_t<float> freespace) {
}

int inference_reset() {
  return 0;
}

#endif // HAVE_CUDA_ENABLE