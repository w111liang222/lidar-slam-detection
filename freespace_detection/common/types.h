#pragma once
#include <vector>
#include <unordered_map>
#include <memory>
#include <float.h>

#include <pcl/common/common.h>

struct AttitudeAngles {
  // 单位弧度
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
};

struct Translation {
  // 单位m
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_z = 0.0;
};

struct PointPose {
  // 激光雷达坐标系或者车辆坐标系
  float x = 0;
  float y = 0;
  float z = 0;
};

struct Grid {
  PointPose center;
  std::vector<int> points;
  float max_z = FLT_MIN;
  float min_z = FLT_MAX;
  double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0;
  int count = 0;
  bool is_occupied = false;
};

struct Polar {
  PointPose farthest_point;
  PointPose nearest_point;
  float max_radius = FLT_MIN;
  float min_radius = FLT_MAX;
  std::vector<Grid> grids;
  bool is_empty = true;
};

typedef std::shared_ptr<Grid> GridPtr;
typedef std::vector<std::vector<Grid>> MyGridMap;
typedef std::vector<Polar> PolarVector;
typedef std::vector<std::vector<std::vector<int>>> RegionPoint;

typedef pcl::PointXYZI Point; 
