#ifndef __GICPM_H
#define __GICPM_H

#include <mutex>
#include <vector>
#include <thread>
#include <memory>

#include "slam_base.h"
#include "mapping_types.h"

namespace Mapping {

class HDL_GICP : public SlamBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HDL_GICP();
  virtual ~HDL_GICP();
  bool init(InitParameter &param) override;
  bool originIsSet() override;
  RTKType& getOrigin() override;
  void setOrigin(RTKType rtk) override;
  std::vector<std::string> setSensors(std::vector<std::string> &sensors) override;
  void feedImuData(ImuType &imu) override;
  void feedInsData(std::shared_ptr<RTKType> ins) override;
  void feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) override;
  void feedImageData(const uint64_t &timestamp,
                     std::map<std::string, ImageType> &images,
                     std::map<std::string, cv::Mat> &images_stream) override;
  Eigen::Matrix4d getPose(PointCloudAttrImagePose &frame) override;

 private:
  void runScanMatch();
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
  Eigen::Matrix4d mInitGuess;
  Eigen::Matrix4d mImuInsStaticTrans;

  RWQueue<PointCloudType*> mScanMatchQueue;
  RWQueue<PointCloudType*> mFloorQueue;
  RWQueue<Eigen::Isometry3d> mOdomQueue;
  Eigen::Matrix4d mLastOdom;

  bool mThreadStart;
  std::unique_ptr<std::thread> mScanMatchThread;
  std::unique_ptr<std::thread> mFloorThread;
  std::unique_ptr<std::thread> mGraphThread;
};

}

#endif