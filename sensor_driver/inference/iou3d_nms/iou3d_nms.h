#ifndef IOU3D_NMS_H
#define IOU3D_NMS_H

#include <vector>
#ifdef HAVE_CUDA_ENABLE
#include <cuda.h>
#include <cuda_runtime_api.h>
#endif

#define DIVUP(m,n) ((m) / (n) + ((m) % (n) > 0))

#endif
