#ifndef __FLOAM_H
#define __FLOAM_H

#include <mutex>
#include <vector>
#include <thread>
#include <memory>
#include <map>

#include "slam_base.h"
#include "mapping_types.h"

namespace Mapping {

class HDL_FLOAM : public SlamBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HDL_FLOAM();
  virtual ~HDL_FLOAM();
  bool init(InitParameter &param) override;
  bool originIsSet() override;
  RTKType& getOrigin() override;
  void setOrigin(RTKType rtk) override;
  std::vector<std::string> setSensors(std::vector<std::string> &sensors) override;
  void feedImuData(ImuType &imu) override;
  void feedInsData(std::shared_ptr<RTKType> ins) override;
  void feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) override;
  void feedImageData(const uint64_t &timestamp,
                     std::map<std::string, cv::Mat> &images,
                     std::map<std::string, cv::Mat> &images_stream) override;
  Eigen::Matrix4d getPose(PointCloudAttrImagePose &frame) override;
  void getGraphEdges(std::vector<EdgeType> &edges) override;

 private:
  void runFloor();
  void runGraph();

 private:
  InitParameter mConfig;
  RTKType mOrigin;
  bool mOriginIsSet;
  PointCloud::Ptr mFrame;
  PointCloudAttrPtr mFrameAttr;
  std::map<std::string, cv::Mat> mImages;
  std::map<std::string, cv::Mat> mImagesStream;
  std::string mLidarName;

  std::mutex mImuDataMutex;
  std::vector<ImuType> mImuData;
  Eigen::Matrix4d mInitGuess;
  Eigen::Matrix4d mImuInsStaticTrans;

  RWQueue<PointCloudType*> mFloorQueue;
  Eigen::Matrix4d mLastOdom;

  bool mThreadStart;
  std::unique_ptr<std::thread> mFloorThread;
  std::unique_ptr<std::thread> mGraphThread;

  int mLidarLines;
  std::map<std::string, int> mSupportLidars;
};

}

#endif