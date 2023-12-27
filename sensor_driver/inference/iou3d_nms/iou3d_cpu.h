#ifndef IOU3D_CPU_H
#define IOU3D_CPU_H

#include <vector>
#ifdef HAVE_CUDA_ENABLE
#include <cuda.h>
#include <cuda_runtime_api.h>
#endif

float iou_bev(const float *box_a, const float *box_b);

#endif
