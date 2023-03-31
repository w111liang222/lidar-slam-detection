#ifndef __LIDAR_SCAN__H
#define __LIDAR_SCAN__H

#include <stdint.h>

#include <memory>
#include <vector>
#include <Eigen/Geometry>

class LidarScan {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  LidarScan(const std::string &name, uint64_t timeStamp, int attrDim,
            const std::shared_ptr<std::vector<float>> &pointCloud,
            const std::shared_ptr<std::vector<float>> &pointCloudAttr);
  virtual ~LidarScan();
  const std::shared_ptr<std::vector<float>> &getPointCloudPtr();
  const std::shared_ptr<std::vector<float>> &getPointCloudAttr();
  const std::string &getName();
  const uint64_t &getTimeStamp();
  const int &getAttrDim();
  void setTimeStamp(uint64_t timestamp);

 protected:
  std::string mName;
  uint64_t mTimeStamp;
  int mAttrDim;
  std::shared_ptr<std::vector<float>> mPointCloudPtr;
  std::shared_ptr<std::vector<float>> mPointCloudAttr;
};

#endif  //__LIDAR_SCAN__H
