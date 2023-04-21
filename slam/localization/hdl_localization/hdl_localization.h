#ifndef __HDL_LOCALIZATION__H
#define __HDL_LOCALIZATION__H

#include "localization_base.h"

namespace Locate {

class HdlLocalization : public LocalizationBase {
  public:
    HdlLocalization(const std::string &imageName);
    virtual ~HdlLocalization();
    bool init(InitParameter &param) override;
    void setInitPose(uint64_t stamp, const Eigen::Matrix4d& pose) override;
    void updateLocalMap(PointCloud::Ptr& cloud) override;
    void feedInsData(std::shared_ptr<RTKType> ins) override;
    void feedImuData(ImuType &imu) override;
    LocType localize(PointCloudAttrPtr& cloud, cv::Mat& image, Eigen::Isometry3d& pose) override;
    bool getTimedPose(uint64_t timestamp, Eigen::Matrix4d &pose) override;
    bool getTimedPose(RTKType &ins, Eigen::Matrix4d &pose) override;
  protected:
    std::string mImageName;
};

}
#endif  //__HDL_LOCALIZATION__H