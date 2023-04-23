#ifndef __VISUAL_ODOMETRY_H
#define __VISUAL_ODOMETRY_H

#include <thread>

#include "common_lib.h"
#include "image_frame.hpp"
#include "pointcloud_rgbd.hpp"
#include "rgbmap_tracker.hpp"

#include "mapping_types.h"

class VisualOdometry {
 public:
  VisualOdometry();
  virtual ~VisualOdometry();
  void setParameter(const CamParamType &camParam);
  bool isInitialized();
  void initialize(const uint64_t& stamp, const Eigen::Matrix4d &t, cv::Mat &image, PointCloud::Ptr& cloud);
  void feedImuData(ImuType &imu);
  bool feedImageData(const uint64_t& stamp, cv::Mat &image);
  bool getPose(Eigen::Matrix4d &pose);
  void updatePose(const Eigen::Matrix4d &t, PointCloud::Ptr &cloud);

  void setInitialCameraParameter(StatesGroup &state);
  void setImagePose(std::shared_ptr<Image_frame> &image_pose, const StatesGroup &state);
  bool vio_esikf(StatesGroup &state_in, Rgbmap_tracker &op_track);
  bool vio_photometric(StatesGroup &state_in, Rgbmap_tracker &op_track, std::shared_ptr<Image_frame> &image);

 protected:
  void processLoop();
  std::shared_ptr<Image_frame> preprocessImage(uint64_t stamp, cv::Mat &image);
  void appendPoints(const Eigen::Matrix4d &t, PointCloud::Ptr &cloud);

 protected:
  CamParamType mCameraParam;
  int mCol, mRow;

  bool mInitialized;
  bool mThreadStart;
  std::unique_ptr<std::thread> mProcessThread;
  RWQueue<std::pair<uint64_t, cv::Mat>> mImageQueue;
  RWQueue<Eigen::Matrix4d> mPoseQueue;

  // R3Live
  StatesGroup                  mState;
  std::shared_ptr<Image_frame> mImagePose;
  mat_3_3                      mInitialRotExtrinicL2c;
  vec_3                        mInitialPosExtrinicL2c;
  Eigen::Matrix3d              mCamK;
  Eigen::Matrix<double, 5, 1>  mCamDist;
  int                          mFrameIdx;
  double                       mCamMeasurementWeight;
  Rgbmap_tracker               mTracker;
  Global_map                   mGlobalRGBMap;
};

#endif // __VISUAL_ODOMETRY_H