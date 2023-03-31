#include <vector>
#include <pybind11/numpy.h>
#ifdef HAS_CUDA_ENABLE
#include <cuda.h>
#include <cuda_runtime_api.h>
#endif

#include "iou3d_cpu.h"
#include "iou3d_nms.h"

namespace py=pybind11;

#ifdef HAS_CUDA_ENABLE
#define CHECK_ERROR(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

const int THREADS_PER_BLOCK_NMS = sizeof(unsigned long long) * 8;

void nmsLauncher(const float *boxes, unsigned long long * mask, int boxes_num, float nms_overlap_thresh);
#endif

int boxes_iou_bev_cpu(py::array_t<float> &boxes_a_tensor, py::array_t<float> &boxes_b_tensor, py::array_t<float> &ans_iou_tensor){
    // params boxes_a_tensor: (N, 7) [x, y, z, dx, dy, dz, heading]
    // params boxes_b_tensor: (M, 7) [x, y, z, dx, dy, dz, heading]
    // params ans_iou_tensor: (N, M)

    int num_boxes_a = boxes_a_tensor.unchecked<2>().shape(0);
    int num_boxes_b = boxes_b_tensor.unchecked<2>().shape(0);
    const float *boxes_a = boxes_a_tensor.data();
    const float *boxes_b = boxes_b_tensor.data();
    float *ans_iou = ans_iou_tensor.mutable_data();

    for (int i = 0; i < num_boxes_a; i++){
        for (int j = 0; j < num_boxes_b; j++){
            ans_iou[i * num_boxes_b + j] = iou_bev(boxes_a + i * 7, boxes_b + j * 7);
        }
    }
    return 1;
}

#ifdef HAS_CUDA_ENABLE
int nms_gpu(py::array_t<float> &boxes, py::array_t<long> &keep, float nms_overlap_thresh){
    // params boxes: (N, 7) [x, y, z, dx, dy, dz, heading]
    // params keep: (N);

    int boxes_num = boxes.unchecked<2>().shape(0);

	float *boxes_data_cuda = NULL;
	cudaMalloc((void**)&boxes_data_cuda, boxes.nbytes());
	cudaMemcpy(boxes_data_cuda, boxes.data(), boxes.nbytes(), cudaMemcpyHostToDevice);

    const float * boxes_data = boxes_data_cuda;
    long * keep_data = keep.mutable_data();

    const int col_blocks = DIVUP(boxes_num, THREADS_PER_BLOCK_NMS);

    unsigned long long *mask_data = NULL;
    CHECK_ERROR(cudaMalloc((void**)&mask_data, boxes_num * col_blocks * sizeof(unsigned long long)));
    nmsLauncher(boxes_data, mask_data, boxes_num, nms_overlap_thresh);

    // unsigned long long mask_cpu[boxes_num * col_blocks];
    // unsigned long long *mask_cpu = new unsigned long long [boxes_num * col_blocks];
    std::vector<unsigned long long> mask_cpu(boxes_num * col_blocks);

//    printf("boxes_num=%d, col_blocks=%d\n", boxes_num, col_blocks);
    CHECK_ERROR(cudaMemcpy(&mask_cpu[0], mask_data, boxes_num * col_blocks * sizeof(unsigned long long),
                           cudaMemcpyDeviceToHost));

    cudaFree(mask_data);
	cudaFree(boxes_data_cuda);

    unsigned long long remv_cpu[col_blocks];
    memset(remv_cpu, 0, col_blocks * sizeof(unsigned long long));

    int num_to_keep = 0;

    for (int i = 0; i < boxes_num; i++){
        int nblock = i / THREADS_PER_BLOCK_NMS;
        int inblock = i % THREADS_PER_BLOCK_NMS;

        if (!(remv_cpu[nblock] & (1ULL << inblock))){
            keep_data[num_to_keep++] = i;
            unsigned long long *p = &mask_cpu[0] + i * col_blocks;
            for (int j = nblock; j < col_blocks; j++){
                remv_cpu[j] |= p[j];
            }
        }
    }
    if ( cudaSuccess != cudaGetLastError() ) printf( "Error!\n" );

    return num_to_keep;
}
#endif

PYBIND11_MODULE(iou3d_nms_ext, m) {
#ifdef HAS_CUDA_ENABLE
	m.def("nms_gpu", &nms_gpu, "oriented nms gpu",
		py::arg("boxes"), py::arg("keep"), py::arg("nms_overlap_thresh")
	);
#endif
	m.def("boxes_iou_bev_cpu", &boxes_iou_bev_cpu, "oriented boxes iou",
		py::arg("boxes_a_tensor"), py::arg("boxes_b_tensor"), py::arg("ans_iou_tensor")
	);
}
