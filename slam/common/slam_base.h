#ifndef __SLAM_BASE__H
#define __SLAM_BASE__H

#include <string>
#include <pcl/common/transforms.h>

#include "mapping_types.h"
#include "slam_utils.h"
#include "pcd_writer.h"
#include "keyframe.h"

struct InitParameter {
  std::string map_path;
  double      resolution;
  double      key_frame_distance;
  double      key_frame_degree;
  double      key_frame_range;
  double      scan_period;
};

struct PoseRange {
  PoseRange() {
    x_min = 0;
    x_max = 0;
    y_min = 0;
    y_max = 0;
  }
  PoseRange(double xmin, double xmax, double ymin, double ymax) {
    x_min = xmin;
    x_max = xmax;
    y_min = ymin;
    y_max = ymax;
  }
  double      x_min;
  double      x_max;
  double      y_min;
  double      y_max;
};

enum MapCoordinateType {
  WGS84 = 0,
  GCJ02,
};

class SlamBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SlamBase() {
    };
    virtual ~SlamBase() {
    };
    virtual bool init(InitParameter &param) = 0;
    virtual bool isInited() {
      return true;
    }
    virtual bool originIsSet() = 0;
    virtual RTKType& getOrigin() = 0;
    virtual void setOrigin(RTKType rtk) = 0;
    virtual MapCoordinateType getMapCoordinate() {
      return MapCoordinateType::WGS84;
    }
    virtual std::vector<std::string> setSensors(std::vector<std::string> &sensors) {
      return sensors;
    }
    virtual void setCameraParameter(const std::map<std::string, CamParamType> &camParam) {
      mCameraParams = camParam;
    }
    virtual void setStaticTransform(const Eigen::Matrix4d &t) {
      mStaticTrans = t;
    }
    virtual void setImuStaticTransform(Eigen::Matrix4d &t) {
      mImuStaticTrans = t;
    }
    virtual void setInitPoseRange(PoseRange &r) {
      return;
    }
    virtual void setInitPose(const Eigen::Matrix4d &t) {
      return;
    }
    virtual int getEstimatePose(Eigen::Matrix4d &t) {
      return 0;
    }
    virtual void preprocessPoints(PointCloud::Ptr &points_in, PointCloud::Ptr &points_out) {
      pcl::transformPointCloud(*points_in, *points_out, mStaticTrans);
    }
    virtual void feedInsData(bool rtk_valid, std::shared_ptr<RTKType> ins) = 0;
    virtual void feedImuData(ImuType &imu) {
      return;
    }
    virtual void feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) = 0;
    virtual void feedImageData(const uint64_t &timestamp,
                               std::map<std::string, ImageType> &images,
                               std::map<std::string, cv::Mat> &images_stream) {
      return;
    }
    virtual Eigen::Matrix4d getPose(PointCloudAttrImagePose &frame) = 0;
    virtual bool getTimedPose(uint64_t timestamp, Eigen::Matrix4d &pose) {
      return false;
    }
    virtual bool getTimedPose(RTKType &ins, Eigen::Matrix4d &pose) {
      return false;
    }
    virtual std::vector<PoseType> getOdometrys() {
      return std::vector<PoseType>();
    }
    virtual void getGraphMap(std::vector<std::shared_ptr<KeyFrame>> &frames) {
    }
    virtual void getColorMap(PointCloudRGB::Ptr &points) {
    }
  protected:
    std::map<std::string, CamParamType> mCameraParams;
    Eigen::Matrix4d mStaticTrans;
    Eigen::Matrix4d mImuStaticTrans;
};

#endif  //__SLAM_BASE__H