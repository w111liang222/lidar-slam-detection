#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <memory>

#include <pcl/common/common.h>
// #include <jsoncpp/json/json.h>

#include "common/parameters.h"
#include "common/types.h"
#include "preprocess/preprocess.h"
#include "coarse_segmentation/coarse_segmentation.h"
// #include "remove_ground/ground_plane_fiting.h"
#include "remove_ground/patch_ground_plane_fiting.h"

using PointType = PointXYZILID;

class FreespaceDetection {
 public:
  FreespaceDetection(std::shared_ptr<Parameters> params_ptr);
  ~FreespaceDetection();
  void Run(PointCloudPtr origin_pd, const AttitudeAngles& attitude_angles);
  PointCloudPtr GetOriginPoints() {return origin_pd_;}
  PointCloudPtr GetTransformPoints() {return transform_pd_;}
  PointCloudPtr GetPreprocessPoints() {return preprocess_pd_;}
  PointCloudPtr GetGroundPoints() {return ground_pd_;}
  PointCloudPtr GetNGroundPoints() {return n_ground_pd_;}
  MyGridMap GetGridMap() {return grid_map_;}
  PolarVector GetPolarVector() {return polar_vector_;}
  GridInfo GetGridInfo() { return grid_info_; }
  
 private:
  void ReadJsonFile();
 public:
  std::shared_ptr<Parameters> params_ptr_;

 private:
  std::string file_name_;

  PointCloudPtr origin_pd_;
  PointCloudPtr transform_pd_;
  PointCloudPtr preprocess_pd_;
  RegionPoint points_index_;
  PointCloudPtr ground_pd_;
  PointCloudPtr n_ground_pd_;
  MyGridMap grid_map_;
  PolarVector polar_vector_;
  GridInfo grid_info_;

  std::unique_ptr<Preprocess> preprocess_ptr_;
  // std::unique_ptr<GroundPlaneFiting> rm_ground_ptr_;
  std::unique_ptr<CoarseSegmentation> coarse_segmentation_ptr_;
  std::unique_ptr<PatchGroundSegmentation<Point>> patch_ptr_;
};


