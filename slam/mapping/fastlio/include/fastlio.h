#ifndef _FASTLIO_H
#define _FASTLIO_H

#include <thread>
#include <memory>
#include <map>

#include "slam_base.h"
#include "mapping_types.h"

namespace Mapping {

class HDL_FastLIO : public SlamBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HDL_FastLIO();
  virtual ~HDL_FastLIO();
  bool init(InitParameter &param) override;
  bool originIsSet() override;
  RTKType& getOrigin() override;
  void setOrigin(RTKType rtk) override;
  std::vector<std::string> setSensors(std::vector<std::string> &sensors) override;
  void feedInsData(std::shared_ptr<RTKType> ins) override;
  void feedImuData(ImuType &imu) override;
  void feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) override;
  void feedImageData(const uint64_t &timestamp,
                     std::map<std::string, ImageType> &images,
                     std::map<std::string, cv::Mat> &images_stream) override;
  Eigen::Matrix4d getPose(PointCloudAttrImagePose &frame) override;

 private:
  void runLio();
  void runFloor();
  void runGraph();

 private:
  InitParameter mConfig;
  RTKType mOrigin;
  bool mOriginIsSet;
  PointCloud::Ptr mFrame;
  PointCloudAttrPtr mFrameAttr;
  std::map<std::string, ImageType> mImages;
  std::map<std::string, cv::Mat> mImagesStream;
  std::string mLidarName;

  std::mutex mImuDataMutex;
  std::vector<ImuType> mImuData;
  Eigen::Matrix4d mImuInsStaticTrans;

  RWQueue<PointCloudType*> mFloorQueue;
  RWQueue<Eigen::Isometry3d> mOdomQueue;
  Eigen::Matrix4d mLastOdom;

  bool mThreadStart;
  std::unique_ptr<std::thread> mLioThread;
  std::unique_ptr<std::thread> mFloorThread;
  std::unique_ptr<std::thread> mGraphThread;
};

}

#endif