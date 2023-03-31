#ifndef _CPP_PATCH_GROUND_PLANE_FITING
#define _CPP_PATCH_GROUND_PLANE_FITING  

#include "patch_ground_plane_fiting.h"

template<typename PointT>
PatchGroundSegmentation<PointT>::PatchGroundSegmentation(
                          std::shared_ptr<Parameters> params_ptr) :
                          patch_param_(params_ptr->patch_param_) {
  CheckParam();
  patch_param_->num_rings_of_interest = patch_param_->elevation_thr.size();

  revert_pc.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
  ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
  non_ground_pc_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
  regionwise_ground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);
  regionwise_nonground_.reserve(NUM_HEURISTIC_MAX_PTS_IN_PATCH);

  min_range_z2_ = patch_param_->min_ranges[1];
  min_range_z3_ = patch_param_->min_ranges[2];
  min_range_z4_ = patch_param_->min_ranges[3];  

  patch_param_->min_ranges = {patch_param_->min_range, min_range_z2_, 
                              min_range_z3_, min_range_z4_};

  patch_param_->ring_sizes = {(min_range_z2_ - patch_param_->min_range) / 
                               patch_param_->num_rings_each_zone.at(0),
    (min_range_z3_ - min_range_z2_) / patch_param_->num_rings_each_zone.at(1),
    (min_range_z4_ - min_range_z3_) / patch_param_->num_rings_each_zone.at(2),
    (patch_param_->max_range - min_range_z4_) / 
     patch_param_->num_rings_each_zone.at(3)};
  patch_param_->sector_sizes = {2 * M_PI / patch_param_->num_sectors_each_zone.at(0), 
                   2 * M_PI / patch_param_->num_sectors_each_zone.at(1),
                   2 * M_PI / patch_param_->num_sectors_each_zone.at(2),
                   2 * M_PI / patch_param_->num_sectors_each_zone.at(3)};  
  std::cout << "PATCH INITIALIZATION COMPLETE" << std::endl;   

  for (int iter = 0; iter < patch_param_->num_zones; ++iter) {
    Zone z;
    InitializeZone(z, patch_param_->num_sectors_each_zone.at(iter), 
                    patch_param_->num_rings_each_zone.at(iter));
    ConcentricZoneModel_.push_back(z);
  }                     
}

template<typename PointT>
PatchGroundSegmentation<PointT>::~PatchGroundSegmentation() {}

template<typename PointT>
void PatchGroundSegmentation<PointT>::CheckParam() {
  std::string SET_SAME_SIZES_OF_PARAMETERS = "Some parameters are wrong! the size of parameters should be same";
  
  int n_z = patch_param_->num_zones;
  int n_r = patch_param_->num_rings_each_zone.size();
  int n_s = patch_param_->num_sectors_each_zone.size();
  int n_m = patch_param_->min_ranges.size();
  
  if ((n_z != n_r) || (n_z != n_s) || (n_z != n_m)) {
    throw std::invalid_argument(SET_SAME_SIZES_OF_PARAMETERS);
  }
  
  if ((n_r != n_s) || (n_r != n_m) || (n_s != n_m)) {
    throw std::invalid_argument(SET_SAME_SIZES_OF_PARAMETERS);
  }
  
  if (patch_param_->min_range != patch_param_->min_ranges[0]) {
    throw std::invalid_argument("Setting min. ranges are weired! The first term should be eqaul to min_range_");
  }
  
  if (patch_param_->elevation_thr.size() != patch_param_->flatness_thr.size()) {
    throw std::invalid_argument("Some parameters are wrong! Check the elevation/flatness_thresholds");
  }
}

template<typename PointT>
inline
void PatchGroundSegmentation<PointT>::InitializeZone(Zone &z, 
                                      int num_sectors, int num_rings) {
  z.clear();
  pcl::PointCloud<PointT> cloud;
  cloud.reserve(1000);
  Ring     ring;
  for (int i = 0; i < num_sectors; i++) {
    ring.emplace_back(cloud);
  }
  for (int j = 0; j < num_rings; j++) {
    z.emplace_back(ring);
  }
}

template<typename PointT>
void PatchGroundSegmentation<PointT>::EstimateGround(
                                      const pcl::PointCloud<PointT> &cloud_in,
                                      pcl::PointCloud<PointT> &cloud_out,
                                      pcl::PointCloud<PointT> &cloud_nonground) {                                     
  if (initialized_ && patch_param_->ATAT_ON) {
    EstimateSensorHeight(cloud_in);
    initialized_ = false;
  }
  // 1.Msg to pointcloud
  pcl::PointCloud<PointT> laserCloudIn;
  laserCloudIn = cloud_in;
  
  // 22.05.02 Update
  // Global sorting is deprecated
  //    sort(laserCloudIn.points.begin(), laserCloudIn.end(), point_z_cmp<PointT>);
  // 2.Error point removal
  // As there are some error mirror reflection under the ground,
  // Sort point according to height, here uses z-axis in default
  // -2.0 is a rough criteria
  // int i = 0;
  // while (i < laserCloudIn.points.size()) {
  //   if (laserCloudIn.points[i].z < -patch_param_->sensor_height - 2.0) {
  //     std::iter_swap(laserCloudIn.points.begin() + i, 
  //                    laserCloudIn.points.end() - 1);
  //     laserCloudIn.points.pop_back();
  //   } else {
  //     ++i;
  //   }
  // }

  // 4. pointcloud -> regionwise setting
  for (int k = 0; k < patch_param_->num_zones; ++k) {
    FlushPatchesInZone(ConcentricZoneModel_[k], 
                       patch_param_->num_sectors_each_zone[k], 
                       patch_param_->num_rings_each_zone[k]);
  }
  pc2czm(laserCloudIn, ConcentricZoneModel_);

  cloud_out.clear();
  cloud_nonground.clear();
  revert_pc.clear();
  reject_pc.clear();

  int concentric_idx = 0;
  for (int k = 0; k < patch_param_->num_zones; ++k) {
    auto zone = ConcentricZoneModel_[k];
    for (uint16_t ring_idx = 0; 
         ring_idx < patch_param_->num_rings_each_zone[k]; ++ring_idx) {
      for (uint16_t sector_idx = 0; 
           sector_idx < patch_param_->num_sectors_each_zone[k]; ++sector_idx) {
        if (zone[ring_idx][sector_idx].points.size() > patch_param_->num_min_pts) {
          // 22.05.02 update
          // Region-wise sorting is adopted
          sort(zone[ring_idx][sector_idx].points.begin(), 
               zone[ring_idx][sector_idx].end(), CompareZ<PointT>);
          ExtractPiecewiseGround(k, zone[ring_idx][sector_idx], 
                                 regionwise_ground_, regionwise_nonground_);

          // Status of each patch
          // used in checking uprightness, elevation, and flatness, respectively
          const double ground_z_vec = abs(normal_(2, 0));
          const double ground_z_elevation = pc_mean_(2, 0);
          const double surface_variable = singular_values_.minCoeff() /
              (singular_values_(0) + singular_values_(1) + singular_values_(2));

          // if (patch_param_->visualize) {
          //   auto polygons = set_polygons(k, ring_idx, sector_idx, 3);
          //   polygons.header = poly_list_.header;
          //   poly_list_.polygons.push_back(polygons);
          //   set_ground_likelihood_estimation_status(k, ring_idx, concentric_idx, ground_z_vec,
          //                                           ground_z_elevation, surface_variable);
          // }

          if (ground_z_vec < patch_param_->uprightness_thr) {
            // All points are rejected
            cloud_nonground += regionwise_ground_;
            cloud_nonground += regionwise_nonground_;
          } else { // satisfy uprightness
            if (concentric_idx < patch_param_->num_rings_of_interest) {
              if (ground_z_elevation > -patch_param_->sensor_height + 
                  patch_param_->elevation_thr[ring_idx + 2 * k]) {
                if (patch_param_->flatness_thr[ring_idx + 2 * k] > 
                    surface_variable) {
                  if (patch_param_->verbose) {
                    // std::cout << "\033[1;36m[Flatness] Recovery operated. Check "
                    //           << ring_idx + 2 * k
                    //           << "th param. flatness_thr_: " 
                    //           << patch_param_->flatness_thr[ring_idx + 2 * k]
                    //           << " > "
                    //           << surface_variable << "\033[0m" << std::endl;
                    revert_pc += regionwise_ground_;
                  }
                  cloud_out += regionwise_ground_;
                  cloud_nonground += regionwise_nonground_;
                } else {
                  if (patch_param_->verbose) {
                      // std::cout << "\033[1;34m[Elevation] Rejection operated. Check "
                      //           << ring_idx + 2 * k
                      //           << "th param. of elevation_thr_: " 
                      //           << -patch_param_->sensor_height + 
                      //              patch_param_->elevation_thr[ring_idx + 2 * k]
                      //           << " < "
                      //           << ground_z_elevation << "\033[0m" << std::endl;
                      reject_pc += regionwise_ground_;
                  }
                  cloud_nonground += regionwise_ground_;
                  cloud_nonground += regionwise_nonground_;
                }
              } else {
                cloud_out += regionwise_ground_;
                cloud_nonground += regionwise_nonground_;
              }
            } else {
              if (patch_param_->using_global_thr && 
                  (ground_z_elevation > patch_param_->global_elevation_thr)) {
                // std::cout << "\033[1;33m[Global elevation] " 
                //           << ground_z_elevation << " > " 
                //           << patch_param_->global_elevation_thr
                //           << "\033[0m" << std::endl;
                cloud_nonground += regionwise_ground_;
                cloud_nonground += regionwise_nonground_;
              } else {
                cloud_out += regionwise_ground_;
                cloud_nonground += regionwise_nonground_;
              }
            }
          }
        }
      }
      ++concentric_idx;
    }
  }
//    ofstream time_txt("/home/shapelim/patchwork_time_anal.txt", std::ios::app);
//    time_txt<<t0 - start<<" "<<t1 - t0 <<" "<<t2-t1<<" "<<t_total_ground<< " "<<t_total_estimate<<"\n";
//    time_txt.close();

  // if (patch_param_->verbose) {
  //   sensor_msgs::PointCloud2 cloud_ROS;
  //   pcl::toROSMsg(revert_pc, cloud_ROS);
  //   cloud_ROS.header.stamp    = ros::Time::now();
  //   cloud_ROS.header.frame_id = "/map";
  //   revert_pc_pub.publish(cloud_ROS);
  //   pcl::toROSMsg(reject_pc, cloud_ROS);
  //   cloud_ROS.header.stamp    = ros::Time::now();
  //   cloud_ROS.header.frame_id = "/map";
  //   reject_pc_pub.publish(cloud_ROS);
  // }
  // PlaneViz.publish(poly_list_);

}

template<typename PointT>
void PatchGroundSegmentation<PointT>::EstimateSensorHeight(
                                      pcl::PointCloud<PointT> cloud_in) {
  // ATAT: All-Terrain Automatic HeighT estimator
  Ring ring_for_ATAT(patch_param_->num_sectors_for_ATAT);
  for (auto const &pt : cloud_in.points) {
    int ring_idx, sector_idx;
    double r = xy2radius(pt.x, pt.y);

    float sector_size_for_ATAT = 2 * M_PI / patch_param_->num_sectors_for_ATAT;

    if ((r <= patch_param_->max_r_for_ATAT) && (r > patch_param_->min_range)) {
      double theta = xy2theta(pt.x, pt.y);
      sector_idx = std::min(static_cast<int>((theta / sector_size_for_ATAT)), 
                       patch_param_->num_sectors_for_ATAT);
      ring_for_ATAT[sector_idx].points.emplace_back(pt);
    }
  }

  // Assign valid measurements and corresponding linearities/planarities
  std::vector<double> ground_elevations_wrt_the_origin;
  std::vector<double> linearities;
  std::vector<double> planarities;
  for (int i = 0; i < patch_param_->num_sectors_for_ATAT; ++i) {
    pcl::PointCloud<PointT> dummy_est_ground;
    pcl::PointCloud<PointT> dummy_est_non_ground;
    ExtractPiecewiseGround(0, ring_for_ATAT[i], dummy_est_ground, 
                            dummy_est_non_ground, false);

    const double ground_z_vec = std::abs(normal_(2, 0));
    const double ground_z_elevation = pc_mean_(2, 0);
    const double linearity   =
            (singular_values_(0) - singular_values_(1)) / singular_values_(0);
    const double planarity   =
            (singular_values_(1) - singular_values_(2)) / singular_values_(0);

    // Check whether the vector is sufficiently upright and flat
    if (ground_z_vec > patch_param_->uprightness_thr && linearity < 0.9) {
      ground_elevations_wrt_the_origin.push_back(ground_z_elevation);
      linearities.push_back(linearity);
      planarities.push_back(planarity);
    }
  }

  // Setting for consensus set-based height estimation
  int N = ground_elevations_wrt_the_origin.size();
  Eigen::Matrix<double, 1, Eigen::Dynamic> values = Eigen::MatrixXd::Ones(1, N);
  Eigen::Matrix<double, 1, Eigen::Dynamic> ranges = 
                      patch_param_->noise_bound * Eigen::MatrixXd::Ones(1, N);
  Eigen::Matrix<double, 1, Eigen::Dynamic> weights = 1.0 / 
        std::pow(patch_param_->noise_bound , 2) * Eigen::MatrixXd::Ones(1, N);
  for (int i = 0; i < N; ++i) {
    values(0, i) = ground_elevations_wrt_the_origin[i];
    ranges(0, i) = ranges(0, i) * linearities[i];
    weights(0, i) = weights(0, i) * planarities[i] * planarities[i];
  }

  double estimated_h = ConsensusSetBasedHeightEstimation(values, ranges, weights);
  // std::cout << "\033[1;33m[ATAT] The sensor height is auto-calibrated via the ground points in the vicinity of the vehicle\033[0m" << std::endl;
  // std::cout << "\033[1;33m[ATAT] Elevation of the ground w.r.t. the origin is " << estimated_h << " m\033[0m" << std::endl;

  // Note that these are opposites
  patch_param_->sensor_height = -estimated_h;
}

template<typename PointT>
double PatchGroundSegmentation<PointT>::ConsensusSetBasedHeightEstimation(
                                        const Eigen::RowVectorXd& X,
                                        const Eigen::RowVectorXd& ranges,
                                        const Eigen::RowVectorXd& weights) {
  // check input parameters
  bool dimension_inconsistent = (X.rows() != ranges.rows()) || (X.cols() != ranges.cols());

  bool only_one_element = (X.rows() == 1) && (X.cols() == 1);
  assert(!dimension_inconsistent);
  assert(!only_one_element); // TODO: admit a trivial solution

  int N = X.cols();
  std::vector<std::pair<double, int>> h;
  for (size_t i= 0 ;i < N ;++i){
      h.push_back(std::make_pair(X(i) - ranges(i), i+1));
      h.push_back(std::make_pair(X(i) + ranges(i), -i-1));
  }

  // ascending order
  std::sort(h.begin(), h.end(), [](std::pair<double, int> a, std::pair<double, int> b) 
            { return a.first < b.first; });

  int nr_centers = 2 * N;
  Eigen::RowVectorXd x_hat = Eigen::MatrixXd::Zero(1, nr_centers);
  Eigen::RowVectorXd x_cost = Eigen::MatrixXd::Zero(1, nr_centers);

  double ranges_inverse_sum = ranges.sum();
  double dot_X_weights = 0;
  double dot_weights_consensus = 0;
  int consensus_set_cardinal = 0;
  double sum_xi = 0;
  double sum_xi_square = 0;

  for (size_t i = 0 ; i < nr_centers ; ++i){

    int idx = int(std::abs(h.at(i).second)) - 1; // Indices starting at 1
    int epsilon = (h.at(i).second > 0) ? 1 : -1;

    consensus_set_cardinal += epsilon;
    dot_weights_consensus += epsilon * weights(idx);
    dot_X_weights += epsilon * weights(idx) * X(idx);
    ranges_inverse_sum -= epsilon * ranges(idx);
    sum_xi += epsilon * X(idx);
    sum_xi_square += epsilon * X(idx) * X(idx);

    x_hat(i) = dot_X_weights / dot_weights_consensus;

    double residual = consensus_set_cardinal * x_hat(i) * x_hat(i) + sum_xi_square  - 2 * sum_xi * x_hat(i);
    x_cost(i) = residual + ranges_inverse_sum;

  }

  size_t min_idx;
  x_cost.minCoeff(&min_idx);
  double estimate_temp = x_hat(min_idx);
  return estimate_temp;
}

template<typename PointT>
inline void PatchGroundSegmentation<PointT>::ExtractPiecewiseGround(
                      const int zone_idx, const pcl::PointCloud<PointT> &src,
                      pcl::PointCloud<PointT> &dst,
                      pcl::PointCloud<PointT> &non_ground_dst,
                      bool is_h_available) {
  // 0. Initialization
  if (!ground_pc_.empty()) ground_pc_.clear();
  if (!dst.empty()) dst.clear();
  if (!non_ground_dst.empty()) non_ground_dst.clear();

  // 1. set seeds!
  ExtractInitialSeeds(zone_idx, src, ground_pc_, is_h_available);

  // 2. Extract ground
  for (int i = 0; i < patch_param_->num_iter; i++) {
    EstimatePlane(ground_pc_);
    ground_pc_.clear();

    //pointcloud to matrix
    Eigen::MatrixXf points(src.points.size(), 3);
    int j = 0;
    for (auto &p : src.points) {
      points.row(j++) << p.x, p.y, p.z;
    }
    // ground plane models
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (int r = 0; r < result.rows(); r++) {
      if (i < patch_param_->num_iter - 1) {
        if (result[r] < th_dist_d_) {
            ground_pc_.points.push_back(src[r]);
        }
      } else { // Final stage
        if (result[r] < th_dist_d_) {
          dst.points.push_back(src[r]);
        } else {
          if (i == patch_param_->num_iter - 1) {
            non_ground_dst.push_back(src[r]);
          }
        }
      }
    }
  }
}

template<typename PointT>
inline void PatchGroundSegmentation<PointT>::ExtractInitialSeeds(
        const int zone_idx, const pcl::PointCloud<PointT> &p_sorted,
        pcl::PointCloud<PointT> &init_seeds, bool is_h_available) {
  init_seeds.points.clear();

  // LPR is the mean of low point representative
  double sum = 0;
  int cnt = 0;

  int init_idx = 0;
  // Empirically, adaptive seed selection applying to Z1 is fine
  if (is_h_available) {
    static double lowest_h_margin_in_close_zone = 
                  (patch_param_->sensor_height == 0.0) ? -0.2 : 
                  patch_param_->adaptive_seed_selection_margin * 
                  patch_param_->sensor_height;
    if (zone_idx == 0) {
      for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lowest_h_margin_in_close_zone) {
          ++init_idx;
        } else {
          break;
        }
      }
    }
  }

  // Calculate the mean height value.
  const int num = p_sorted.points.size();
  for (int i = init_idx; i < num && cnt < patch_param_->num_lpr; i++) {
    if (p_sorted.points[i].z < -patch_param_->sensor_height + 0.3) {
      sum += p_sorted.points[i].z;
      cnt++;
    } else {
      break;
    }
  }
  double lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

  // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
  for (int i = 0; i < p_sorted.points.size(); i++) {
    if (p_sorted.points[i].z < lpr_height + patch_param_->th_seeds) {
      init_seeds.points.push_back(p_sorted.points[i]);
    }
  }
}

template<typename PointT>
inline void PatchGroundSegmentation<PointT>::EstimatePlane(
                                      const pcl::PointCloud<PointT> &ground) {
  pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);
  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, 
                                    Eigen::DecompositionOptions::ComputeFullU);
  singular_values_ = svd.singularValues();

  // use the least singular vector as normal
  normal_ = (svd.matrixU().col(2));
  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean_.head<3>();

  // according to normal.T*[x,y,z] = -d
  d_ = -(normal_.transpose() * seeds_mean)(0, 0);
  // set distance threhold to `th_dist - d`
  th_dist_d_ = patch_param_->th_dist - d_;
}

template<typename PointT>
inline void PatchGroundSegmentation<PointT>::FlushPatchesInZone(Zone &patches, 
                                              int num_sectors, int num_rings) {
  for (int i = 0; i < num_sectors; i++) {
    for (int j = 0; j < num_rings; j++) {
      if (!patches[j][i].points.empty()) patches[j][i].points.clear();
    }
  }
} 

template<typename PointT>
inline
void PatchGroundSegmentation<PointT>::pc2czm(const pcl::PointCloud<PointT> &src, 
                                             std::vector<Zone> &czm) {
  for (auto const &pt : src.points) {
    int ring_idx, sector_idx;
    double r = xy2radius(pt.x, pt.y);
    if ((r <= patch_param_->max_range) && (r > patch_param_->min_range)) {
      double theta = xy2theta(pt.x, pt.y);

      if (r < min_range_z2_) { // In First rings
        ring_idx = std::min(static_cast<int>(
          ((r - patch_param_->min_range) / patch_param_->ring_sizes[0])), 
          patch_param_->num_rings_each_zone[0] - 1);
        sector_idx = std::min(static_cast<int>(
          (theta / patch_param_->sector_sizes[0])), 
          patch_param_->num_sectors_each_zone[0] - 1);
        czm[0][ring_idx][sector_idx].points.emplace_back(pt);
      } else if (r < min_range_z3_) {
        ring_idx = std::min(static_cast<int>(
          ((r - min_range_z2_) / patch_param_->ring_sizes[1])), 
          patch_param_->num_rings_each_zone[1] - 1);
        sector_idx = std::min(static_cast<int>(
          (theta / patch_param_->sector_sizes[1])), 
          patch_param_->num_sectors_each_zone[1] - 1);
        czm[1][ring_idx][sector_idx].points.emplace_back(pt);
      } else if (r < min_range_z4_) {
        ring_idx = std::min(static_cast<int>(
          ((r - min_range_z3_) / patch_param_->ring_sizes[2])), 
          patch_param_->num_rings_each_zone[2] - 1);
        sector_idx = std::min(static_cast<int>(
          (theta / patch_param_->sector_sizes[2])), 
          patch_param_->num_sectors_each_zone[2] - 1);
        czm[2][ring_idx][sector_idx].points.emplace_back(pt);
      } else { // Far!
        ring_idx = std::min(static_cast<int>((
          (r - min_range_z4_) / patch_param_->ring_sizes[3])), 
          patch_param_->num_rings_each_zone[3] - 1);
        sector_idx = std::min(static_cast<int>(
          (theta / patch_param_->sector_sizes[3])), 
          patch_param_->num_sectors_each_zone[3] - 1);
        czm[3][ring_idx][sector_idx].points.emplace_back(pt);
      }
    }
  }
}

// template<typename PointT>
// inline
// void PatchGroundSegmentation<PointT>::SetGroundLikelihoodEstimationStatus(
//                                          const int k, const int ring_idx,
//                                          const int concentric_idx,
//                                          const double z_vec,
//                                          const double z_elevation,
//                                          const double surface_variable) {
//   if (z_vec > uprightness_thr_) { //orthogonal
//     if (concentrix_idx < num_rings_of_interest_) {
//       if (z_elevation > -sensor_height_ + elevation_thr_[ring_idx + 2 * k]) {
//         if (flatness_thr_[ring_idx + 2 * k] > surface_variable) {
//           poly_list_.likelihood.push_back(FLAT_ENOUGH);
//         } else {
//           poly_list_.likelihood.push_back(TOO_HIGH_ELEVATION);
//         }
//       } else {
//         poly_list_.likelihood.push_back(UPRIGHT_ENOUGH);
//       }
//     } else {
//       if (using_global_thr_ && (z_elevation > global_elevation_thr_)) {
//         poly_list_.likelihood.push_back(GLOBALLLY_TOO_HIGH_ELEVATION_THR);
//       } else {
//         poly_list_.likelihood.push_back(UPRIGHT_ENOUGH);
//       }
//     }
//   } else { // tilted
//     poly_list_.likelihood.push_back(TOO_TILTED);
//   }
// }

#endif
