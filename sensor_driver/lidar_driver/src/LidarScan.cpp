#include "LidarScan.h"

#include <iostream>
#include "Logger.h"

const double PI = 3.1415926535897932384626433832795;

LidarScan::LidarScan(const std::string &name, uint64_t timeStamp, int attrDim,
                     const std::shared_ptr<std::vector<float>> &pointCloud,
                     const std::shared_ptr<std::vector<float>> &pointCloudAttr) {
  mName = name;
  mTimeStamp = timeStamp;
  mAttrDim = attrDim;
  mPointCloudPtr = pointCloud;
  mPointCloudAttr = pointCloudAttr;
}

LidarScan::~LidarScan() {}

const std::shared_ptr<std::vector<float>> &LidarScan::getPointCloudPtr() {
  return mPointCloudPtr;
}

const std::shared_ptr<std::vector<float>> &LidarScan::getPointCloudAttr() {
  return mPointCloudAttr;
}

const std::string &LidarScan::getName() { return mName; }

const uint64_t &LidarScan::getTimeStamp() { return mTimeStamp; }

const int &LidarScan::getAttrDim() { return mAttrDim; }

void LidarScan::setTimeStamp(uint64_t timestamp) { mTimeStamp = timestamp; }
