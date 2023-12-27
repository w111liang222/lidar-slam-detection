
#ifndef __ROSBAG_WRITTER__H
#define __ROSBAG_WRITTER__H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>

#include "Types.h"

class RosbagWritter
{
public:
  RosbagWritter(std::string file);
  virtual ~RosbagWritter();
  void writeScan(std::string topic, const std::string frame, uint64_t timestamp,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr input);
  void writeImage(std::string topic, const std::string frame, uint64_t timestamp,
                  cv::Mat input);
  void writeCompressedImage(std::string topic, const std::string frame, uint64_t timestamp,
                            cv::Mat input);
  void writeIns(std::string topic, const std::string frame, Ins_t &ins);
  void writeImu(std::string topic, const std::string frame, Imu_t &imu);
  void writeTimeStamp(std::string topic, uint64_t timestamp, uint64_t data);

private:
  void *mBag;
};

#endif //__ROSBAG_WRITTER__H