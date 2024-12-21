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
  void feedInsData(bool rtk_valid, std::shared_ptr<RTKType> ins) override;
  void feedImuData(ImuType &imu) override;
  void feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) override;
  void feedImageData(const uint64_t &timestamp,
                     std::map<std::string, ImageType> &images,
                     std::map<std::string, cv::Mat> &images_stream) override;
  Eigen::Matrix4d getPose(PointCloudAttrImagePose &frame) override;
  std::vector<PoseType> getOdometrys() override;

 private:
  void runLio();
  void runFloor();
  void runGraph();

 private:
  InitParameter mConfig;
  RTKType mOrigin;
  bool mOriginIsSet;

  std::string mLidarName;
  PointCloudAttrPtr mFrameAttr;
  std::map<std::string, ImageType> mImages;
  std::map<std::string, cv::Mat> mImagesStream;

  std::mutex mImuDataMutex;
  std::vector<ImuType> mImuData;
  std::vector<PoseType> mOdometrys;
  Eigen::Matrix4d mImuInsStaticTrans;

  RWQueue<PointCloudType*> mFloorQueue;
  RWQueue<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> mOdomQueue;

  bool mThreadStart;
  std::unique_ptr<std::thread> mLioThread;
  std::unique_ptr<std::thread> mFloorThread;
  std::unique_ptr<std::thread> mGraphThread;
};

}

#endif