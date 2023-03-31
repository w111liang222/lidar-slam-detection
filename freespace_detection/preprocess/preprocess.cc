#include "preprocess.h"

Preprocess::Preprocess(const std::shared_ptr<Parameters>& params_ptr) {
  param_ptr_ = params_ptr->preprocess_param_;

  origin_pd_ = PointCloudPtr(new PointCloud());
  roi_pd_ = PointCloudPtr(new PointCloud());
  transform_pd_ = PointCloudPtr(new PointCloud());
  processd_pd_ = PointCloudPtr(new PointCloud()); 
  region_width_x_ = (param_ptr_->range_front_x - param_ptr_->range_rear_x) / 
                     param_ptr_->region_num_x;
  region_width_y_ = (param_ptr_->range_left_y - param_ptr_->range_right_y) / 
                     param_ptr_->region_num_y;

  points_index_ = RegionPoint(param_ptr_->region_num_x, 
                    std::vector<std::vector<int>>(param_ptr_->region_num_y));                   
}

Preprocess::~Preprocess() {}

void Preprocess::Run(PointCloudPtr origin_pd, 
                     const AttitudeAngles& attitude_angles) { 
  // 清除上一帧数据
  // origin_pd_.reset(new  PointCloud());
  origin_pd_ = origin_pd; 
  roi_pd_.reset(new  PointCloud());                      
  transform_pd_.reset(new  PointCloud());                      
  processd_pd_.reset(new  PointCloud());    
  RegionPoint(param_ptr_->region_num_x, std::vector<std::vector<int>>(
              param_ptr_->region_num_y)).swap(points_index_); 

  // 筛选ROI内的点云  
  SelectRoitemp(); 
  // SelectRoi();
  if (param_ptr_->downsample_option) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(roi_pd_);
    voxel_filter.setLeafSize(param_ptr_->filter_resol, param_ptr_->filter_resol, 
                            param_ptr_->filter_resol);
    voxel_filter.filter(*roi_pd_);  
  }
  // SelectRoiByradius();
  // Rotate(attitude_angles);
  processd_pd_ = roi_pd_;
}

void Preprocess::SelectRoitemp() {
  if (origin_pd_->points.size() > 0) {    
    // int count = 0;
    for (const Point& p : origin_pd_->points) {
      if (IsInRoi(p) && !IsInVehicle(p)) {
      // if (!IsInVehicle(p)) {
        roi_pd_->push_back(p);
      }
    }
  } else {
    std::cerr << "origin pointcloud is empty" << std::endl;
  }
}

void Preprocess::SelectRoi() {
  if (origin_pd_->points.size() > 0) {    
    int count = 0;
    for (const Point& p : origin_pd_->points) {
      if (IsInRoi(p) && !IsInVehicle(p)) {
        roi_pd_->push_back(p);
        // double dist = std::sqrt(p.x * p.x + p.y * p.y);
        double region_index_x = p.x / region_width_x_;
        double region_index_y = p.y / region_width_y_;

        int index_x = region_index_x + (param_ptr_->region_num_x / 2); // 考虑坐标为负值时的偏移 
        int index_y = region_index_y + (param_ptr_->region_num_y / 2); // 考虑坐标为负值时的偏移
        if (index_x < param_ptr_->region_num_x && 
            index_y < param_ptr_->region_num_y) {
          points_index_.at(index_x).at(index_y).push_back(count);
        }
        count++;
      }
    }
  } else {
    std::cerr << "origin pointcloud is empty" << std::endl;
  }
}

void Preprocess::SelectRoiByradius() {
  if (origin_pd_->points.size() > 0) {    
    // int count = 0;
    for (const Point& p : origin_pd_->points) {
      if (IsInRoiByRadius(p) && !IsInVehicle(p)) {
        roi_pd_->push_back(p);
        // double dist = std::sqrt(p.x * p.x + p.y * p.y);
        // double region_index_x = p.x / region_width_x_;
        // double region_index_y = p.y / region_width_y_;

        // int index_x = region_index_x + (param_ptr_->region_num_x / 2); // 考虑坐标为负值时的偏移 
        // int index_y = region_index_y + (param_ptr_->region_num_y / 2); // 考虑坐标为负值时的偏移
        // if (index_x < param_ptr_->region_num_x && 
        //     index_y < param_ptr_->region_num_y) {
        //   points_index_.at(index_x).at(index_y).push_back(count);
        // }
        // count++;
      }
    }
  } else {
    std::cerr << "origin pointcloud is empty" << std::endl;
  }
}


void Preprocess::Rotate(const AttitudeAngles& attitude_angles) {
  double delt_x = 0.0;
  double delt_y = 0.0;
  double delt_z = 0.0;
  transform_matrix_.translation() << delt_x, delt_y, delt_z;
  transform_matrix_.rotate(Eigen::AngleAxisf(attitude_angles.yaw, 
                                             Eigen::Vector3f::UnitZ()));
  transform_matrix_.rotate(Eigen::AngleAxisf(attitude_angles.pitch, 
                                             Eigen::Vector3f::UnitY()));
  transform_matrix_.rotate(Eigen::AngleAxisf(attitude_angles.roll, 
                                             Eigen::Vector3f::UnitX())); 
  pcl::transformPointCloud(*roi_pd_, *transform_pd_, transform_matrix_);
}
