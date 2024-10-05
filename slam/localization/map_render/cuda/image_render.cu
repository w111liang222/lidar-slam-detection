#include "image_render.hpp"
#include "launch.cuh"

static __global__ void check_pixels(size_t num_pixels, int cols, int rows, const float *depths, const float *pixel_data, int *pixel_valid) {
    int pixel_idx = cuda_linear_index;
    if (pixel_idx >= num_pixels) return;

    const float &depth = pixel_data[3 * pixel_idx + 0];
    const float &u     = pixel_data[3 * pixel_idx + 1];
    const float &v     = pixel_data[3 * pixel_idx + 2];

    float min_depth = depth;
    float max_depth = depth;
    for (int i = -4; i <= 4; i++) {
        for (int j = -4; j <= 4; j++) {
            int iu = u + i;
            int jv = v + j;
            int uv = jv * cols + iu;
            if (iu < 0 || iu >= cols || jv < 0 || jv >= rows) {
                continue;
            }
            if (depths[uv] < 0) {
                continue;
            }
            min_depth = min(min_depth, depths[uv]);
            max_depth = max(max_depth, depths[uv]);
        }
    }
    pixel_valid[pixel_idx] = ((max_depth - min_depth) > 2.0) ? 0 : 1;
}

ImageRenderCuda::ImageRenderCuda(cv::Mat K, Eigen::Matrix4d static_trans) : ImageRender(K, static_trans) {}

ImageRenderCuda::~ImageRenderCuda() {
    if (h_pixel_data)    checkRuntime(cudaFreeHost(h_pixel_data));
    if (d_pixel_data)    checkRuntime(cudaFree(d_pixel_data));
    if (h_pixel_valid)   checkRuntime(cudaFreeHost(h_pixel_valid));
    if (d_pixel_valid)   checkRuntime(cudaFree(d_pixel_valid));
    if (d_depths)        checkRuntime(cudaFree(d_depths));
}

void ImageRenderCuda::setPose(Eigen::Matrix4d &pose, int cols, int rows) {
    if (mCols != cols || mRows != rows) {
        checkRuntime(cudaMallocHost(&h_pixel_data, 3 * rows * cols * sizeof(float)));
        checkRuntime(cudaMalloc(&d_pixel_data, 3 * rows * cols * sizeof(float)));
        checkRuntime(cudaMallocHost(&h_pixel_valid, rows * cols * sizeof(int)));
        checkRuntime(cudaMalloc(&d_pixel_valid, rows * cols * sizeof(int)));
        checkRuntime(cudaMalloc(&d_depths, rows * cols * sizeof(float)));
    }
    ImageRender::setPose(pose, cols, rows);
}

void ImageRenderCuda::checkDepthContinuous(std::unordered_map<hash_uv, Render::PixelPtr> &pixels, std::vector<float> &depths) {
    cudaStream_t stream;
    cudaStreamCreate(&stream);

    // copy depths
    checkRuntime(cudaMemcpyAsync(d_depths, depths.data(), mRows * mCols * sizeof(float), cudaMemcpyHostToDevice, stream));

    // copy pixel depth/u/v
    int num_pixels = pixels.size();
    pixel_vector.resize(num_pixels);
    std::transform(pixels.begin(), pixels.end(), pixel_vector.begin(), [](auto pair){return pair.second;});
    for (int i = 0; i < num_pixels; i++) {
        h_pixel_data[3 * i + 0] = pixel_vector[i]->depth;
        h_pixel_data[3 * i + 1] = pixel_vector[i]->u;
        h_pixel_data[3 * i + 2] = pixel_vector[i]->v;
    }

    int pixel_data_size = 3 * num_pixels * sizeof(float);
    checkRuntime(cudaMemcpyAsync(d_pixel_data, h_pixel_data, pixel_data_size, cudaMemcpyHostToDevice, stream));

    // launch depth check kernel
    cuda_linear_launch(check_pixels, stream, num_pixels, mCols, mRows, d_depths, d_pixel_data, d_pixel_valid);

    // copy valid data from device
    int pixel_valid_size = num_pixels * sizeof(int);
    checkRuntime(cudaMemcpyAsync(h_pixel_valid, d_pixel_valid, pixel_valid_size, cudaMemcpyDeviceToHost, stream));

    // sync stream
    checkRuntime(cudaStreamSynchronize(stream));
    checkRuntime(cudaStreamDestroy(stream));
}

void ImageRenderCuda::updateVoxel() {
    int num_pixels = pixel_vector.size();
    for (int n = 0; n < num_pixels; n++) {
        if (h_pixel_valid[n] == 1) {
            update_voxel_rgb(pixel_vector[n]);
        }
    }
}