
#ifndef __ROSBAG_READER__H
#define __ROSBAG_READER__H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>

#include "Types.h"

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 80

void printProgress(double percentage)
{
  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);
}

class RosbagReader
{
public:
  RosbagReader(std::string file, std::string config_path);
  virtual ~RosbagReader();
  uint32_t getFrameSize(std::string topic);
  std::vector<Imu_t> readImu(std::string topic);
  std::vector<Ins_t> readGps(std::string topic);
  void startScanIter(std::string topic);
  void StopScanIter();
  uint64_t readScan(PointCloud::Ptr &scan);

private:
  cv::FileStorage mConfig;
  std::string mDataSet;
  void *mBag;
  void *mScanView;
};

#endif //__ROSBAG_READER__H