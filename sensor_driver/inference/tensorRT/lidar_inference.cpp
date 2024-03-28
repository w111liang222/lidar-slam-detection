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

#ifdef HAVE_CUDA_ENABLE

#include "lidar_inference.h"
#include <iostream>
#include <cuda_runtime.h>

LidarInference::LidarInference(LidarEngineParameter parameter)
{
    parameter_ = parameter;
    max_points = parameter.voxelization.max_points;
    preprocessor_ = create_preprocess(parameter.proprecess);
    voxelizer_ = create_voxelization(parameter.voxelization);
    scn_engine_ = spconv::load_engine_from_onnx(parameter.scn_file, spconv::Precision::Int8);
    rpn_engine_ = TensorRT::load(parameter.rpn_file);
    cudaStreamCreate(&stream);

    sparse_shape.clear();
    sparse_shape.push_back(parameter.voxelization.grid_size.z + 1);
    sparse_shape.push_back(parameter.voxelization.grid_size.y);
    sparse_shape.push_back(parameter.voxelization.grid_size.x);

    cls_preds_size = rpn_engine_->getBindingNumel("cls_preds") * sizeof(float);
    box_preds_size = rpn_engine_->getBindingNumel("box_preds") * sizeof(float);
    label_preds_size = rpn_engine_->getBindingNumel("label_preds") * sizeof(int);

    checkCudaErrors(cudaMalloc((void **)&d_cls_preds, cls_preds_size));
    checkCudaErrors(cudaMalloc((void **)&d_box_preds, box_preds_size));
    checkCudaErrors(cudaMalloc((void **)&d_label_preds, label_preds_size));

    if (rpn_engine_->getBindingIndex("masks_bev") != -1) {
        bev_mask_size = rpn_engine_->getBindingNumel("masks_bev") * sizeof(float);
        checkCudaErrors(cudaMalloc((void **)&d_bev_mask, bev_mask_size));
    }
}

LidarInference::~LidarInference()
{
    preprocessor_.reset();
    voxelizer_.reset();
    scn_engine_.reset();
    rpn_engine_.reset();
    checkRuntime(cudaStreamDestroy(stream));

    if (d_bev_mask)     checkCudaErrors(cudaFree(d_bev_mask));
    if (d_cls_preds)    checkCudaErrors(cudaFree(d_cls_preds));
    if (d_box_preds)    checkCudaErrors(cudaFree(d_box_preds));
    if (d_label_preds)  checkCudaErrors(cudaFree(d_label_preds));
}

void LidarInference::reset()
{
    preprocessor_ = create_preprocess(parameter_.proprecess);
}

int LidarInference::forward(const float* points, int point_num, const float* motion, bool runtime)
{
    // voxelization
    int num_points = std::min(point_num, max_points);

    preprocessor_->forward(points, num_points, motion, runtime, stream);
    voxelizer_->forward(preprocessor_->get_points(), preprocessor_->get_points_num(), CoordinateOrder::ZYX, stream);
    unsigned int valid_num = voxelizer_->get_output(&d_voxel_features, &d_voxel_indices);

    // spconv
    auto spatial_features = scn_engine_->forward(
        {valid_num, parameter_.voxelization.num_feature}, spconv::DType::Float16, d_voxel_features,
        {valid_num, 4}, spconv::DType::Int32,   d_voxel_indices,
        1, sparse_shape, stream
    );

    // RPN
    if (d_bev_mask == nullptr) {
        rpn_engine_->forward({spatial_features->features_data(), d_cls_preds, d_box_preds, d_label_preds}, stream);
    } else {
        rpn_engine_->forward({spatial_features->features_data(), d_bev_mask, d_cls_preds, d_box_preds, d_label_preds}, stream);
    }
    return 0;
}

void LidarInference::get_output(float* cls_preds, float* box_preds, int* label_preds, float* freespace) {
    checkRuntime(cudaMemcpy(cls_preds, d_cls_preds, cls_preds_size, cudaMemcpyDeviceToHost));
    checkRuntime(cudaMemcpy(box_preds, d_box_preds, box_preds_size, cudaMemcpyDeviceToHost));
    checkRuntime(cudaMemcpy(label_preds, d_label_preds, label_preds_size, cudaMemcpyDeviceToHost));
    if (d_bev_mask) checkRuntime(cudaMemcpy(freespace, d_bev_mask, bev_mask_size, cudaMemcpyDeviceToHost));
}

std::shared_ptr<LidarInference> create_engine(LidarEngineParameter parameter) {
  std::shared_ptr<LidarInference> engine = std::shared_ptr<LidarInference>(new LidarInference(parameter));
  return engine;
}

#endif // HAVE_CUDA_ENABLE