#ifndef __RTK_MAPPING_H
#define __RTK_MAPPING_H

#include <map>
#include <memory>
#include "slam_base.h"
#include "UTMProjector.h"

namespace Mapping {

class RTKM : public SlamBase{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RTKM();
  virtual ~RTKM();
  bool init(InitParameter &param) override;
  bool originIsSet() override;
  RTKType &getOrigin() override;
  void setOrigin(RTKType rtk) override;
  std::vector<std::string> setSensors(std::vector<std::string> &sensors) override;
  void feedInsData(bool rtk_valid, std::shared_ptr<RTKType> data) override;
  void feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) override;
  void feedImageData(const uint64_t &timestamp,
                     std::map<std::string, ImageType> &images,
                     std::map<std::string, cv::Mat> &images_stream) override;
  Eigen::Matrix4d getPose(PointCloudAttrImagePose &frame) override;

 private:
  bool getInterpolatedTransform(uint64_t timeStamp, Eigen::Matrix4d& trans);
  void computeRTKTransform(const RTKType &origin, std::shared_ptr<RTKType> &data);

 private:
  void runGraph();

 protected:
  InitParameter mConfig;
  bool mOriginIsSet;
  RTKType mOrigin;
  PointCloudAttrPtr mFrameAttr;
  std::map<std::string, ImageType> mImages;
  std::map<std::string, cv::Mat> mImagesStream;
  std::string mLidarName;

  std::map<uint64_t, std::shared_ptr<RTKType>> mTimedInsData;
  Eigen::Matrix4d mLastOdom;

 private:
  std::unique_ptr<UTMProjector> mProjector;

  bool mThreadStart;
  std::unique_ptr<std::thread> mGraphThread;
};

}

#endif