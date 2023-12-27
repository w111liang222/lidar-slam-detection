#ifndef __ODOM_LIDAR_H_
#define __ODOM_LIDAR_H_

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "transform.h"

#include <random>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Pointcloud;

typedef long long int Timestamp;

class OdomTformData {
 public:
  OdomTformData(Timestamp timestamp_us, Transform T_o0_ot);

  const Transform& getTransform() const;
  const Timestamp& getTimestamp() const;

 private:
  Transform T_o0_ot_;
  Timestamp timestamp_us_;
};

class Odom {
 public:
  void addTransformData(const Timestamp& timestamp_us,
                        const Transform& transform);

  Transform getOdomTransform(const Timestamp timestamp_us,
                             const size_t start_idx = 0,
                             size_t* match_idx = nullptr) const;

  const size_t getNumberOfOdoms() const;

 private:
  std::vector<OdomTformData> data_;
};

class Scan {
 public:
  struct Config {
  };

  Scan(const Pointcloud& pointcloud, Timestamp &time, const Config& config);

  static Config getConfig();

  void setOdomTransform(const Odom& odom, const double time_offset,
                        const size_t start_idx, size_t* match_idx);

  void getTimeAlignedPointcloud(const Transform& T_o_l,
                                Pointcloud* pointcloud) const;

 private:
  Timestamp timestamp_us_;  // signed to allow simpler comparisons
  Pointcloud raw_points_;
  Transform T_o0_ot_;  // absolute odom transform to each point in pointcloud

  bool odom_transform_set_;
};

class Lidar {
 public:
  Lidar() {};

  const size_t getNumberOfScans() const;

  // note points are appended so any points in *pointcloud are preserved
  void getCombinedPointcloud(Pointcloud* pointcloud) const;

  void addPointcloud(const std::vector<float> &pointcloud, Timestamp time,
                     const Scan::Config& config = Scan::Config());

  void setOdomOdomTransforms(const Odom& odom, const double time_offset = 0.0);

  void setOdomLidarTransform(const Transform& T_o_l);
  void setBestOdomLidarTransform(const Transform& T_o_l);

  const Transform& getOdomLidarTransform() const;
  const Transform& getBestOdomLidarTransform() const;

 private:
  Transform T_o_l_;  // transform from lidar to odometry
  Transform bestT_o_l_;

  std::vector<Scan> scans_;
};

#endif //__ODOM_LIDAR_H_