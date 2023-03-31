#ifndef __TYPES__H
#define __TYPES__H

#include <stdint.h>
#include <vector>
#include "Transform.h"

struct RangeFilter {
  RangeFilter() {
    xmin = 1e4;
    xmax = 1e4;
    ymin = 1e4;
    ymax = 1e4;
    zmin = 1e4;
    zmax = 1e4;
  }
  RangeFilter(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) {
    xmin = xMin;
    xmax = xMax;
    ymin = yMin;
    ymax = yMax;
    zmin = zMin;
    zmax = zMax;
  }
  double xmin;
  double xmax;
  double ymin;
  double ymax;
  double zmin;
  double zmax;
};

#pragma pack(1)
struct CustomLidarPackage {
  char head[2];
  char version[2];
  uint32_t frame_id;
  uint64_t timestamp;
  uint32_t point_num;
  float points_buf[4 * 74];
  char tail[2];
};
#pragma pack()

inline void pointTransform(float &x, float &y, float &z, Transform &trans) {
    Eigen::Vector3d t(x, y, z);
    t = trans.translation() + trans.rotation() * t;
    x = t(0);
    y = t(1);
    z = t(2);
}

#endif  //__TYPES__H
