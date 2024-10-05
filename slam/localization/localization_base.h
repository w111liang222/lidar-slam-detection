#ifndef __LOCALIZATION_BASE__H
#define __LOCALIZATION_BASE__H

#include "slam_base.h"
#include "mapping_types.h"

namespace Locate {

enum LocType {
  OK,
  ERROR,
  OTHER,
};

class LocalizationBase {
  public:
    LocalizationBase() {
    };
    virtual ~LocalizationBase() {
    };
    virtual bool init(InitParameter &param) = 0;
    virtual bool isInited() {
      return true;
    }
    virtual void setStaticTransform(Eigen::Matrix4d &t) {
      mStaticTrans = t;
    }
    virtual void setImuStaticTransform(Eigen::Matrix4d &t) {
      mImuStaticTrans = t;
    }
    virtual void setCameraParams(std::map<std::string, CamParamType> &camParam) {
      mCameraParam = camParam;
    }
    virtual void setInitPose(uint64_t stamp, const Eigen::Matrix4d& pose) = 0;
    virtual void updateLocalMap(PointCloud::Ptr& cloud) = 0;
    virtual void feedInsData(std::shared_ptr<RTKType> ins) = 0;
    virtual void feedImuData(ImuType &imu) {
      return;
    }
    virtual LocType localize(PointCloudAttrPtr& cloud, ImageType& image, Eigen::Isometry3d& pose) = 0;
    virtual bool getTimedPose(uint64_t timestamp, Eigen::Matrix4d &pose) = 0;
    virtual bool getTimedPose(RTKType &ins, Eigen::Matrix4d &pose) = 0;
  protected:
    Eigen::Matrix4d mStaticTrans;
    Eigen::Matrix4d mImuStaticTrans;
    std::map<std::string, CamParamType> mCameraParam;
};

}

#endif  //__LOCALIZATION_BASE__H