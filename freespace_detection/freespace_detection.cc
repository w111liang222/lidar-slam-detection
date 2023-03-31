#include "freespace_detection.h"

FreespaceDetection::FreespaceDetection(std::shared_ptr<Parameters> params_ptr) 
                                       : params_ptr_(params_ptr) {
  preprocess_ptr_.reset(new Preprocess(params_ptr_));
  // rm_ground_ptr_.reset(new GroundPlaneFiting(params_ptr_));
  coarse_segmentation_ptr_.reset(new CoarseSegmentation(params_ptr_));
  patch_ptr_.reset(new PatchGroundSegmentation<Point>(params_ptr_));
  ground_pd_.reset(new pcl::PointCloud<Point>);
  n_ground_pd_.reset(new pcl::PointCloud<Point>);
  transform_pd_.reset(new pcl::PointCloud<Point>);
  preprocess_pd_.reset(new pcl::PointCloud<Point>);
}

FreespaceDetection::~FreespaceDetection() {}

void FreespaceDetection::Run(PointCloudPtr origin_pd, const AttitudeAngles& attitude_angles) {
  origin_pd_ = origin_pd;
  preprocess_ptr_->Run(origin_pd_, attitude_angles);
  preprocess_pd_ = preprocess_ptr_->GetResult();

  patch_ptr_->EstimateGround(*preprocess_pd_, *ground_pd_, *n_ground_pd_);

  coarse_segmentation_ptr_->Run(n_ground_pd_);
  grid_map_ = coarse_segmentation_ptr_->GetGridMap();   
  polar_vector_ = coarse_segmentation_ptr_->GetPolarVector(); 
  grid_info_ = coarse_segmentation_ptr_->GetGridInfo();
}

