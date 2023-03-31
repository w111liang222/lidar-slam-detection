#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <future>
#include <limits>
#include <nlopt.hpp>

#include "sensor.h"

class Aligner {
 public:
  struct Config {
    bool local = false;
    double max_time_offset = 0.2;

    double max_evals = 500;
    double xtol = 0.0001;

    int knn_batch_size = 1000;
    int local_knn_k = 1;
    int global_knn_k = 3;
    float local_knn_max_dist = 1.0;
    float global_knn_max_dist = 10.0;
    bool time_cal = true;
  };

  struct OptData {
    Lidar* lidar;
    Odom* odom;
    Aligner* aligner;
    bool time_cal;
  };

  Aligner(const Config& config);

  static Config getConfig();

  void lidarOdomTransform(Lidar* lidar, Odom* odom);
  bool getOptimizationStatus(float &percent);

 public:
  int iter_;
  double error_;
  bool finish_;

 private:
  void optimize(const std::vector<double>& lb, const std::vector<double>& ub,
                OptData* opt_data, std::vector<double>* x);

  static float kNNError(
      const pcl::KdTreeFLANN<Point>& kdtree, const Pointcloud& pointcloud,
      const size_t k, const float max_dist, const size_t start_idx = 0,
      const size_t end_idx = std::numeric_limits<size_t>::max());

  float lidarOdomKNNError(const Pointcloud& base_pointcloud,
                          const Pointcloud& combined_pointcloud) const;

  float lidarOdomKNNError(const Lidar& lidar) const;

  static double LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data);

  Config config_;
};

#endif  // LIDAR_ALIGN_ALIGNER_H_
