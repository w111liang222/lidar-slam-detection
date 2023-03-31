#pragma once
#include <memory>

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "../common/parameters.h"
#include "../common/types.h"
#include "../common/common.h"

class Preprocess
{
 public:
  Preprocess(const std::shared_ptr<Parameters>& params_ptr);
  ~Preprocess();
  void Run(PointCloudPtr origin_pd, const AttitudeAngles& attitude_angles);
  // PointCloudPtr GetTransform() { return transform_pd_; }
  PointCloudPtr GetResult() { return processd_pd_; }
  RegionPoint GetPointsIndex() { return points_index_; }

 private:
  void SelectRoi();
  void SelectRoitemp();
  void SelectRoiByradius();
  void Filter() {}
  void Rotate(const AttitudeAngles& attitude_angles);
  bool IsIn(const float x, const float x_min, const float x_max) {
    return (x < x_max) && (x > x_min);
  }
  bool IsInRoi(const Point& p) {
    return IsIn(p.x, param_ptr_->range_rear_x, param_ptr_->range_front_x) && 
           IsIn(p.y, param_ptr_->range_right_y, param_ptr_->range_left_y) &&
           IsIn(p.z, param_ptr_->range_down_z, param_ptr_->range_up_z);
  }
  bool IsInVehicle(const Point& p) {
    return IsIn(p.x, param_ptr_->vehicle_rear_x, param_ptr_->vehicle_front_x) && 
           IsIn(p.y, param_ptr_->vehicle_right_y, param_ptr_->vehicle_left_y) &&
           IsIn(p.z, param_ptr_->vehicle_down_z, param_ptr_->vehicle_up_z);
  }  
  bool IsInRoiByRadius(const Point& p) {
    return std::sqrt(p.x * p.x + p.y * p.y) < param_ptr_->range_radius
           && IsIn(p.z, param_ptr_->range_down_z, param_ptr_->range_up_z);
  }  

 private:
  std::shared_ptr<PreprocessParam> param_ptr_;
  float region_width_x_;
  float region_width_y_;
  RegionPoint points_index_;
  PointCloudPtr origin_pd_;
  PointCloudPtr roi_pd_;
  PointCloudPtr transform_pd_;
  PointCloudPtr processd_pd_;
  AttitudeAngles pre_attitude_angles_;
  Eigen::Affine3f transform_matrix_ = Eigen::Affine3f::Identity();
};


