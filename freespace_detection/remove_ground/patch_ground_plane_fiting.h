#ifndef PATCH_GROUND_PLANE_FITING_H
#define PATCH_GROUND_PLANE_FITING_H

#include <iostream>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

#include "../common/types.h"
#include "../common/parameters.h"
#include "../common/common.h"

#define MARKER_Z_VALUE -2.2
#define UPRIGHT_ENOUGH 0.55 // cyan
#define FLAT_ENOUGH 0.2 // green
#define TOO_HIGH_ELEVATION 0.0 // blue
#define TOO_TILTED 1.0 // red
#define GLOBALLLY_TOO_HIGH_ELEVATION_THR 0.8

#define NUM_HEURISTIC_MAX_PTS_IN_PATCH 3000

template<typename PointT>
bool CompareZ(const PointT& p1, const PointT& p2) {
  return p1.z < p2.z;
}

template<typename PointT>
class PatchGroundSegmentation {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::vector<pcl::PointCloud<PointT> > Ring;
  typedef std::vector<Ring>                     Zone; 

  PatchGroundSegmentation(std::shared_ptr<Parameters> params_ptr);
  ~PatchGroundSegmentation();
  void EstimateGround(const pcl::PointCloud<PointT> &cloud_in,
                      pcl::PointCloud<PointT> &cloud_out,
                      pcl::PointCloud<PointT> &cloud_nonground);
  PointCloudPtr GetGround() { return all_ground_; }
  PointCloudPtr GetNGround() { return all_n_ground_; }  

 private:
  void CheckParam();
  void InitializeZone(Zone &z, int num_sectors, int num_rings); 
  void EstimateSensorHeight(pcl::PointCloud<PointT> cloud_in);
  void ExtractPiecewiseGround(const int zone_idx, 
                              const pcl::PointCloud<PointT> &src,
                              pcl::PointCloud<PointT> &dst,
                              pcl::PointCloud<PointT> &non_ground_dst,
                              bool is_h_available=true);
  double ConsensusSetBasedHeightEstimation(const Eigen::RowVectorXd& X,
                                           const Eigen::RowVectorXd& ranges,
                                           const Eigen::RowVectorXd& weights);                              
  void ExtractInitialSeeds(const int zone_idx, 
                            const pcl::PointCloud<PointT> &p_sorted,
                            pcl::PointCloud<PointT> &init_seeds, 
                            bool is_h_available);                   
  void EstimatePlane(const pcl::PointCloud<PointT> &ground); 
  void FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings);
  void pc2czm(const pcl::PointCloud<PointT> &src, std::vector<Zone> &czm);
  void SetGroundLikelihoodEstimationStatus(const int k, const int ring_idx,
                                           const int concentric_idx,
                                           const double z_vec,
                                           const double z_elevation,
                                           const double surface_variable);

 private:
  PointCloudPtr all_ground_; // 所有地面点集
  PointCloudPtr all_n_ground_; // 所有非地面点集    

  bool initialized_ = true;  

  double min_range_z2_; // 12.3625
  double min_range_z3_; // 22.025
  double min_range_z4_; // 41.35  
  std::vector<Zone> ConcentricZoneModel_;

  float d_;
  Eigen::MatrixXf normal_;
  Eigen::VectorXf singular_values_;
  float th_dist_d_;
  Eigen::Matrix3f cov_;
  Eigen::Vector4f pc_mean_;
  double ring_size;
  double sector_size;

  pcl::PointCloud<PointT> revert_pc, reject_pc;
  pcl::PointCloud<PointT> ground_pc_;
  pcl::PointCloud<PointT> non_ground_pc_;
  pcl::PointCloud<PointT> regionwise_ground_;
  pcl::PointCloud<PointT> regionwise_nonground_;  

  std::shared_ptr<PatchParam> patch_param_;  
};

#include "patch_ground_plane_fiting.cc"

#endif