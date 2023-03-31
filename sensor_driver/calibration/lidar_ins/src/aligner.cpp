#include "aligner.h"

#include "Logger.h"

Aligner::Aligner(const Config& config)
 : config_(config), iter_(0), error_(1e8), finish_(false)
{
};

Aligner::Config Aligner::getConfig() {
  Aligner::Config config;
  return config;
}

float Aligner::kNNError(const pcl::KdTreeFLANN<Point>& kdtree,
                        const Pointcloud& pointcloud, const size_t k,
                        const float max_dist, const size_t start_idx,
                        const size_t end_idx) {
  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);

  float error = 0;
  for (size_t idx = start_idx; idx < std::min(pointcloud.size(), end_idx);
       ++idx) {
    kdtree.nearestKSearch(pointcloud[idx], k, kdtree_idx, kdtree_dist);
    for (const float& x : kdtree_dist) {
      error += std::min(x, max_dist);
    }
  }
  return error;
}

float Aligner::lidarOdomKNNError(const Pointcloud& base_pointcloud,
                                 const Pointcloud& combined_pointcloud) const {
  // shared_pointer needed by kdtree, no-op destructor to prevent it trying to
  // clean it up after use
  Pointcloud::ConstPtr combined_pointcloud_ptr(&combined_pointcloud,
                                               [](const Pointcloud*) {});

  pcl::KdTreeFLANN<Point> kdtree;

  kdtree.setInputCloud(combined_pointcloud_ptr);

  float max_dist =
      config_.local ? config_.local_knn_max_dist : config_.global_knn_max_dist;

  size_t k = config_.local ? config_.local_knn_k : config_.global_knn_k;
  // if searching own cloud add one to k as a point will always match to itself
  if (&base_pointcloud == &combined_pointcloud) {
    ++k;
  }

  // small amount of threading here to take edge off of bottleneck
  // break knn lookup up into several smaller problems each running in their own
  // thread
  std::vector<std::future<float>> errors;
  for (size_t start_idx = 0; start_idx < base_pointcloud.size();
       start_idx += config_.knn_batch_size) {
    size_t end_idx =
        start_idx + std::min(base_pointcloud.size() - start_idx,
                             static_cast<size_t>(config_.knn_batch_size));
    errors.emplace_back(std::async(std::launch::async, Aligner::kNNError,
                                   std::ref(kdtree), std::ref(base_pointcloud), k, max_dist,
                                   start_idx, end_idx));
  }

  // wait for threads to finish and grab results
  float total_error = 0.0f;
  for (std::future<float>& error : errors) {
    total_error += error.get();
  }

  return total_error;
}

float Aligner::lidarOdomKNNError(const Lidar& lidar) const {
  Pointcloud pointcloud;
  lidar.getCombinedPointcloud(&pointcloud);
  return lidarOdomKNNError(pointcloud, pointcloud);
}

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data) {
  OptData* d = static_cast<OptData*>(f_data);

  if (x.size() > 6) {
    d->lidar->setOdomOdomTransforms(*(d->odom), x[6]);
  }

  Eigen::Matrix<double, 6, 1> vec;
  vec.setZero();

  for (size_t i = 0; i < 6; ++i) {
    vec[i] = x[i];
  }

  d->lidar->setOdomLidarTransform(Transform::exp(vec.cast<float>()));

  double error = d->aligner->lidarOdomKNNError(*(d->lidar));
  if (d->aligner->error_ > error) {
    d->aligner->error_ = error;
    d->lidar->setBestOdomLidarTransform(Transform::exp(vec.cast<float>()));
  }

  d->aligner->iter_++;
  return error;
}

void Aligner::optimize(const std::vector<double>& lb,
                       const std::vector<double>& ub, OptData* opt_data,
                       std::vector<double>* x) {
  nlopt::opt opt;
  if (config_.local) {
    opt = nlopt::opt(nlopt::LN_BOBYQA, x->size());
  } else {
    opt = nlopt::opt(nlopt::GN_DIRECT_L, x->size());
  }

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  opt.set_maxeval(config_.max_evals);
  opt.set_xtol_abs(config_.xtol);

  opt.set_min_objective(LidarOdomMinimizer, opt_data);

  double minf;
  std::vector<double> grad;
  nlopt::result result = opt.optimize(*x, minf);
  LidarOdomMinimizer(*x, grad, opt_data);
  Transform::Vector6 bestX = opt_data->lidar->getBestOdomLidarTransform().log();
  for(int i = 0; i < 6; i++) {
    (*x)[i] = bestX(i);
  }
}

void Aligner::lidarOdomTransform(Lidar* lidar, Odom* odom) {
  OptData opt_data;
  opt_data.lidar = lidar;
  opt_data.odom = odom;
  opt_data.aligner = this;
  opt_data.time_cal = config_.time_cal;

  size_t num_params = 6;
  if (config_.time_cal) {
    ++num_params;
  }

  std::vector<double> x(num_params, 0.0);

  if (!config_.local) {
    LOG_INFO("Performing Global Optimization...");

    std::vector<double> lb = {-100.0, -100.0, -10.0, -M_PI, -M_PI, -M_PI};
    std::vector<double> ub = { 100.0,  100.0,  10.0,  M_PI,  M_PI,  M_PI};

    Transform::Vector6 initX = lidar->getBestOdomLidarTransform().log();
    error_ = lidarOdomKNNError(*(lidar));
    std::vector<double> global_x(6, 0.0);
    for(int i = 0; i < 6; i++) {
      global_x[i] = initX(i);
    }
    optimize(lb, ub, &opt_data, &global_x);
    config_.local = true;

    x[0] = global_x[0];
    x[1] = global_x[1];
    x[2] = global_x[2];
    x[3] = global_x[3];
    x[4] = global_x[4];
    x[5] = global_x[5];
  }

  LOG_INFO("Performing Local Optimization...");

  std::vector<double> lb = {-10.0, -10.0, -1.0, -1.0, -1.0, -1.0};
  std::vector<double> ub = { 10.0,  10.0,  1.0,  1.0,  1.0,  1.0};
  for (size_t i = 0; i < 6; ++i) {
    lb[i] += x[i];
    ub[i] += x[i];
  }
  if (config_.time_cal) {
    ub.push_back(config_.max_time_offset);
    lb.push_back(-config_.max_time_offset);
  }

  optimize(lb, ub, &opt_data, &x);
  finish_ = true;
  LOG_INFO("Optimization done");
}

bool Aligner::getOptimizationStatus(float &percent) {
  if (finish_) {
    percent = 100.0f;
  } else {
    percent = float(iter_) / (config_.max_evals * 2) * 100;
  }
  return finish_;
}
