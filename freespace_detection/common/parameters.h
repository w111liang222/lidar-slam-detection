#pragma once
// #ifndef PARAMETERS_H
// #define PARAMETERS_H
#include <memory>
#include <vector>

#include <pcl/common/common.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud; 
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr; 
// typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr PointCloudConstPtr;

struct PreprocessParam {
  float range_front_x = 20;  // m
  float range_rear_x = -20;    // m
  float range_left_y = 10;   // m
  float range_right_y = -10; // m
  float range_up_z = 2.6;   // m
  float range_down_z = -0.2; // m  
  float range_radius = 40; // m  
  // area of the vehicle
  float vehicle_front_x = 3.1;  // m
  float vehicle_rear_x = -0.5;    // m
  float vehicle_left_y = 1.2;   // m
  float vehicle_right_y = -1.2; // m  
  float vehicle_up_z = 2.6;   // m
  float vehicle_down_z = 0; // m    
  int region_num_x = 16;  
  int region_num_y = 8;
  bool downsample_option = true;
  float filter_resol = 0.1;
};

struct RemoveGroundParam {
  float range_front_x = 50;  // m
  float range_rear_x = 0;    // m
  float range_left_y = 12;   // m
  float range_right_y = -12; // m
  int region_num_x = 25;  
  int region_num_y = 25;  
  // gpf parameter
  unsigned int num_iter = 5; // 迭代次数
  unsigned int num_LPR = 50; // 用于计算最低点高度的点云数量
  float Th_seed = 0.2; // 用于选取种子点集的高度阈值
  float Th_dist = 0.25; // 用于确定地面点集的高度阈值    
};

struct CoarseSegmentationParam {
  // gridmap parameters
  int grid_max_x = 40;
  int grid_min_x = 0;
  int grid_max_y = 10;
  int grid_min_y = -10;
  float grid_max_z = 2.6;
  float grid_min_z = 0;  
  float grid_size_x = 0.1;
  float grid_size_y = 0.1;
  int num_thres = 2;
  float min_z_thres = 2.5;

  // pola parameters
  float theta_divider = 2;
  int lidar_fov = 360;
};

struct PatchParam {
    bool ATAT_ON = false;
    double noise_bound = 0.2;
    double max_r_for_ATAT = 5;
    int num_sectors_for_ATAT = 20;

    int num_iter = 5;
    int num_lpr = 10;
    int num_min_pts = 5;
    int num_rings = 30;
    int num_sectors = 100;
    int num_zones = 4;
    int num_rings_of_interest;

    double sensor_height = 0;
    double th_seeds = 0.2;
    double th_dist = 0.2;
    double max_range = 80;
    double min_range = 0.9;
    double uprightness_thr = 0.707;
    double adaptive_seed_selection_margin = -1.2;
    double min_range_z2; // 12.3625
    double min_range_z3; // 22.025
    double min_range_z4; // 41.35

    bool verbose = false;
    bool  using_global_thr = false;
    double global_elevation_thr = -0.5;    
    std::vector<int> num_sectors_each_zone = {1, 1, 1, 1};
    std::vector<int> num_rings_each_zone = {4, 4, 4, 2};

    std::vector<double> sector_sizes;
    std::vector<double> ring_sizes;
    std::vector<double> min_ranges = {0.9, 8.3625, 19.025, 30.35};
    std::vector<double> elevation_thr = {0.423, 0.646, 0.679, 0.825};
    std::vector<double> flatness_thr = {0.0005, 0.000725, 0.001, 0.001};    

    bool visualize = false;
};

class Parameters
{
 public:
  Parameters() {
    preprocess_param_ = std::make_shared<PreprocessParam>();
    remove_ground_param_ = std::make_shared<RemoveGroundParam>();
    segmentation_param_ = std::make_shared<CoarseSegmentationParam>();
    patch_param_ = std::make_shared<PatchParam>();
  }
  ~Parameters() {}
  Parameters operator=(const Parameters& params) {
    this->preprocess_param_ = params.preprocess_param_;
    this->remove_ground_param_ = params.remove_ground_param_;
    this->segmentation_param_ = params.segmentation_param_;
    this->patch_param_ = params.patch_param_;
    return *this;
  }

 public:
  std::shared_ptr<PreprocessParam> preprocess_param_;
  std::shared_ptr<RemoveGroundParam> remove_ground_param_;
  std::shared_ptr<CoarseSegmentationParam> segmentation_param_;
  std::shared_ptr<PatchParam> patch_param_;
};


// #endif // !PARAMETERS_H


