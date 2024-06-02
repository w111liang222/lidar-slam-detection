#ifndef __TYPES__H
#define __TYPES__H

#include <stdlib.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp> 
#include <opencv2/core/eigen.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    float time = 0;
    float height = 0;

    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, radius, radius)(
                                  std::uint8_t, intensity, intensity)(std::uint16_t, angle, angle)(
                                  std::uint8_t, ring, ring)(double, time, time)(float, height, height))

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (float, time, time)(std::uint16_t, ring, ring))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (std::uint32_t, range, range))

typedef FullPointType Point;
typedef pcl::PointCloud<Point> PointCloud;

struct Imu_t {
  Imu_t() {
    gyro_x = 0;
    gyro_y = 0;
    gyro_z = 0;
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
    timestamp = 0;
  }
  double gyro_x;           // rad / s
  double gyro_y;           // rad / s
  double gyro_z;           // rad / s
  double acc_x;            // m / s^2
  double acc_y;            // m / s^2
  double acc_z;            // m / s^2
  uint64_t timestamp;      // us
};

struct Ins_t {
  Ins_t() {
    latitude = 0;
    longitude = 0;
    altitude = 0;
    status = 0;
    timestamp = 0;
  }
  double latitude;         // degrees
  double longitude;        // degrees
  double altitude;         // m
  int status;
  uint64_t timestamp;      // us
};

#endif //__TYPES__H