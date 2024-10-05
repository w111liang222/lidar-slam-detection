#ifndef __IMAGE_RENDER_HPP__
#define __IMAGE_RENDER_HPP__

#include <memory>
#include <vector>
#include <Eigen/Eigen>

#include "render_common.h"

class ImageRender {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageRender(cv::Mat K, Eigen::Matrix4d static_trans) {
    mCols = 0;
    mRows = 0;

    fx = K.at<float>(0, 0);
    fy = K.at<float>(1, 1);
    cx = K.at<float>(0, 2);
    cy = K.at<float>(1, 2);

    Eigen::Matrix4d ext_i2c = static_trans.inverse();
    m_rot_ext_i2c = ext_i2c.topLeftCorner<3, 3>();
    m_pos_ext_i2c = ext_i2c.topRightCorner<3, 1>();
  }
  virtual ~ImageRender() {
    pixel_vector.clear();
  }
  virtual void setPose(Eigen::Matrix4d &pose, int cols, int rows) {
    mCols = cols;
    mRows = rows;

    Eigen::Vector3d    pose_w2c_t = pose.topLeftCorner<3, 3>() * m_pos_ext_i2c + pose.topRightCorner<3, 1>();
    Eigen::Quaterniond pose_w2c_q = Eigen::Quaterniond(pose.topLeftCorner<3, 3>() * m_rot_ext_i2c);

    m_pose_c2w_q = pose_w2c_q.inverse();
    m_pose_c2w_t = -(pose_w2c_q.inverse() * pose_w2c_t);
  }
  virtual void checkDepthContinuous(std::unordered_map<hash_uv, Render::PixelPtr> &pixels, std::vector<float> &depths) = 0;
  virtual void updateVoxel() = 0;

 protected:
  int mCols, mRows;
  double fx, fy, cx, cy;
  Eigen::Quaterniond m_pose_c2w_q;
  Eigen::Vector3d    m_pose_c2w_t;
  Eigen::Matrix3d    m_rot_ext_i2c;
  Eigen::Vector3d    m_pos_ext_i2c;

  std::vector<Render::PixelPtr> pixel_vector;
};

class ImageRenderCpu : public ImageRender {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageRenderCpu(cv::Mat K, Eigen::Matrix4d static_trans) : ImageRender(K, static_trans) {}
  virtual ~ImageRenderCpu() {
    if (h_pixel_valid)   delete[] h_pixel_valid;
  }
  void setPose(Eigen::Matrix4d &pose, int cols, int rows) override {
    if (mCols != cols || mRows != rows) {
      h_pixel_valid = new int[rows * cols];
    }
    ImageRender::setPose(pose, cols, rows);
  }
  void checkDepthContinuous(std::unordered_map<hash_uv, Render::PixelPtr> &pixels, std::vector<float> &depths) override {
    int num_pixels = pixels.size();
    pixel_vector.resize(num_pixels);
    std::transform(pixels.begin(), pixels.end(), pixel_vector.begin(), [](auto pair){return pair.second;});
    for (int n = 0; n < num_pixels; n++) {
      float min_depth = pixel_vector[n]->depth;
      float max_depth = pixel_vector[n]->depth;
      for (int i = -4; i <= 4; i++) {
        for (int j = -4; j <= 4; j++) {
          int iu = pixel_vector[n]->u + i;
          int jv = pixel_vector[n]->v + j;
          int uv = jv * mCols + iu;
          if (iu < 0 || iu >= mCols || jv < 0 || jv >= mRows) {
            continue;
          }
          if (depths[uv] < 0) {
            continue;
          }
          min_depth = std::min(min_depth, depths[uv]);
          max_depth = std::max(max_depth, depths[uv]);
        }
      }
      h_pixel_valid[n] = ((max_depth - min_depth) > 2.0) ? 0 : 1;
    }
  }

  void updateVoxel() override {
    int num_pixels = pixel_vector.size();
    for (int n = 0; n < num_pixels; n++) {
      if (h_pixel_valid[n] == 1) {
        update_voxel_rgb(pixel_vector[n]);
      }
    }
  }

 protected:
  int   *h_pixel_valid  = nullptr;
};

class ImageRenderCuda : public ImageRender {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageRenderCuda(cv::Mat K, Eigen::Matrix4d static_trans);
  virtual ~ImageRenderCuda();
  void setPose(Eigen::Matrix4d &pose, int cols, int rows) override;
  void checkDepthContinuous(std::unordered_map<hash_uv, Render::PixelPtr> &pixels, std::vector<float> &depths) override;
  void updateVoxel() override;

 protected:
  float *h_pixel_data   = nullptr;
  float *d_pixel_data   = nullptr;
  int   *h_pixel_valid  = nullptr;
  int   *d_pixel_valid  = nullptr;
  float *d_depths       = nullptr;
};

inline std::shared_ptr<ImageRender> create_image_render(cv::Mat K, Eigen::Matrix4d static_trans) {
#ifdef HAVE_CUDA_ENABLE
  return std::make_shared<ImageRenderCuda>(K, static_trans);
#else
  return std::make_shared<ImageRenderCpu>(K, static_trans);
#endif
}

#endif  // __IMAGE_RENDER_HPP__