/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <memory>
#include <cuda_fp16.h>
#include "spconv/engine.hpp"
#include "tensorrt.hpp"
#include "preprocess.hpp"
#include "voxelization.hpp"


#define checkCudaErrors(op)                                                                  \
  {                                                                                          \
    auto status = ((op));                                                                    \
    if (status != 0) {                                                                       \
      std::cout << "Cuda failure: " << cudaGetErrorString(status) << " in file " << __FILE__ \
                << ":" << __LINE__ << " error status: " << status << std::endl;              \
      abort();                                                                               \
    }                                                                                        \
  }

struct LidarEngineParameter {
  std::string scn_file;
  std::string rpn_file;
  PreprocessParameter proprecess;
  VoxelizationParameter voxelization;
};

class LidarInference {
  private:
    LidarEngineParameter parameter_;
    std::shared_ptr<Preprocess> preprocessor_;
    std::shared_ptr<Voxelization> voxelizer_;
    std::shared_ptr<spconv::Engine> scn_engine_;
    std::shared_ptr<TensorRT::Engine> rpn_engine_;
    cudaStream_t stream;

    // input
    half* d_voxel_features;
    unsigned int* d_voxel_indices;
    std::vector<int> sparse_shape;
    int max_points;

    // output
    float* d_bev_mask = nullptr;
    float* d_cls_preds = nullptr;
    float* d_box_preds = nullptr;
    int*   d_label_preds = nullptr;

    unsigned int bev_mask_size;
    unsigned int cls_preds_size;
    unsigned int box_preds_size;
    unsigned int label_preds_size;

  public:
    LidarInference(LidarEngineParameter parameter);
    ~LidarInference();

    void reset();
    int forward(const float* points, int point_num, const float* motion, bool runtime);
    void get_output(float* cls_preds, float* box_preds, int* label_preds, float* freespace);
};

std::shared_ptr<LidarInference> create_engine(LidarEngineParameter parameter);