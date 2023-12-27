
#include "launch.cuh"
#include "preprocess.hpp"


static __global__ void transform_kernel(size_t points_size, int offset, const float *src_buffer, float *dst_buffer,
                                        PreprocessParameter param, float *motion) {
  int point_idx = cuda_linear_index;
  if (point_idx >= points_size) return;

  float px = src_buffer[param.num_feature * point_idx];
  float py = src_buffer[param.num_feature * point_idx + 1];
  float pz = src_buffer[param.num_feature * point_idx + 2];

  dst_buffer[offset + param.num_feature * point_idx]     = motion[0] * px + motion[1] * py + motion[2] * pz + motion[3];
  dst_buffer[offset + param.num_feature * point_idx + 1] = motion[4] * px + motion[5] * py + motion[6] * pz + motion[7];
  dst_buffer[offset + param.num_feature * point_idx + 2] = motion[8] * px + motion[9] * py + motion[10]* pz + motion[11];
  dst_buffer[offset + param.num_feature * point_idx + 3] = src_buffer[param.num_feature * point_idx + 3];
  dst_buffer[offset + param.num_feature * point_idx + 4] = src_buffer[param.num_feature * point_idx + 4] + 0.1;
}

class PreprocessImplement : public Preprocess {
 public:
  virtual ~PreprocessImplement() {
    for (size_t i = 0; i < points_devices_.size(); i++) {
      checkRuntime(cudaFree(points_devices_[i]));
    }
    points_devices_.clear();

    if (d_motion_)       checkRuntime(cudaFree(d_motion_));
  }

  bool init(PreprocessParameter param) {
    this->param_ = param;

    points_device_idx_ = 0;
    // allocate the A-B buffer for accumulate points
    points_devices_.resize(2);
    for (size_t i = 0; i < points_devices_.size(); i++) {
      int points_size = param_.max_points * param_.num_feature * sizeof(float);
      checkRuntime(cudaMalloc(&points_devices_[i], points_size));
    }

    // allocate the motion transform
    int motion_size = 4 * 4 * sizeof(float);
    checkRuntime(cudaMalloc(&d_motion_, motion_size));

    // allocate frame num buffer
    points_num_.resize(param_.max_frame_num, 0);
    total_points_num_ = 0;

    return true;
  }

  // points and voxels must be of half type
  virtual void forward(const float* points, int num_points, const float* motion, bool realtime, void* stream) override {
    cudaStream_t _stream = reinterpret_cast<cudaStream_t>(stream);
    checkRuntime(cudaMemcpyAsync(d_motion_, motion, 4 * 4 * sizeof(float), cudaMemcpyHostToDevice, _stream));

    if (realtime) {
      // pop out the last frame points
      total_points_num_ = total_points_num_ - points_num_.back();
      num_points = std::min(num_points, param_.max_points - total_points_num_);
      num_points = std::max(num_points, 1);

      // move all frames to next position
      for(size_t i = points_num_.size() - 1; i >= 1; i--) {
        points_num_[i] = points_num_[i - 1];
      }

      // determine the output buffer
      float* src_buffer;
      float* dst_buffer;
      if (points_device_idx_ == 0) {
        src_buffer = points_devices_[0];
        dst_buffer = points_devices_[1];
        points_device_idx_ = 1;
      } else {
        src_buffer = points_devices_[1];
        dst_buffer = points_devices_[0];
        points_device_idx_ = 0;
      }

      // transform the points in the sliding window
      if (total_points_num_ > 0) {
        cuda_linear_launch(transform_kernel, _stream, total_points_num_, num_points * param_.num_feature,
                           src_buffer, dst_buffer, param_, d_motion_);
      }
    } else {
      points_num_.resize(param_.max_frame_num, 0);
      total_points_num_ = 0;
      points_device_idx_ = 0;
    }

    // copy points from host to device
    points_num_[0] = num_points;
    total_points_num_ = total_points_num_ + num_points;
    size_t bytes_points = num_points * param_.num_feature * sizeof(float);
    checkRuntime(cudaMemcpyAsync(points_devices_[points_device_idx_], points, bytes_points, cudaMemcpyHostToDevice, _stream));
    checkRuntime(cudaStreamSynchronize(_stream));
  }

  virtual float* get_points() override {
    return points_devices_[points_device_idx_];
  }

  virtual int get_points_num() override {
    return total_points_num_;
  }

 private:
  PreprocessParameter param_;
  int points_device_idx_;
  std::vector<float*> points_devices_;
  std::vector<int>    points_num_;
  int                 total_points_num_;

  float *d_motion_ = nullptr;
};

std::shared_ptr<Preprocess> create_preprocess(PreprocessParameter param) {
  std::shared_ptr<PreprocessImplement> preprocessor = std::shared_ptr<PreprocessImplement>(new PreprocessImplement());
  preprocessor->init(param);
  return preprocessor;
}