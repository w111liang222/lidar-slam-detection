#ifndef HDL_LOCALIZATION_DELTA_ESTIMATER_HPP
#define HDL_LOCALIZATION_DELTA_ESTIMATER_HPP

#include <thread>

#include "mapping_types.h"
#include "slam_utils.h"

namespace hdl_localization {

class DeltaEstimater {
public:
  DeltaEstimater();
  ~DeltaEstimater();
  void init(Eigen::Matrix4d &extrinic);
  void feedImuData(ImuType &imu);
  void feedPointData(PointCloudAttrPtr &points);
  bool getDeltaOdom(Eigen::Matrix4d &delta);

private:
  void runLio();

private:
  Eigen::Matrix4d mLastodom;
  Eigen::Matrix4d mImuInsStaticTrans;
  RWQueue<Eigen::Matrix4d> mOdomQueue;

  bool mThreadStart;
  std::unique_ptr<std::thread> mLioThread;
};

} // namespace hdl_localization


#endif