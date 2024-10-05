#include <cmath>
#include <hdl_localization/pose_estimator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <hdl_localization/pose_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

#include "Logger.h"
#include "slam_base.h"

namespace hdl_localization {

const double fitness_score_max_range = 25; // 5m

/**
 * @brief constructor
 * @param stamp               timestamp
 * @param pos                 initial position
 * @param quat                initial orientation
 * @param cool_time_duration  during "cool time", prediction is not performed
 */
PoseEstimator::PoseEstimator(const Eigen::Matrix4f imu_ext, const uint64_t& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration)
    : init_stamp(stamp), cool_time_duration(cool_time_duration) {
  prev_stamp = 0;
  last_correction_stamp = 0;
  imu_extrinic = imu_ext;
  registration = nullptr;
  is_set_registration = false;

  process_noise = Eigen::MatrixXf::Identity(23, 23);
  process_noise.middleRows(0, 3) *= 2.0;
  process_noise.middleRows(3, 3) *= 5.0;
  process_noise.middleRows(6, 4) *= 2.0;
  process_noise.middleRows(10, 3) *= 1e-4;
  process_noise.middleRows(13, 3) *= 1e-4;
  process_noise.middleRows(16, 3) *= 5.0;
  process_noise.middleRows(19, 4) *= 1e-4;

  measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= 0.2;
  measurement_noise.middleRows(3, 4) *= 0.1;

  visual_noise = Eigen::MatrixXf::Identity(7, 7);
  visual_noise.middleRows(0, 3) *= 1.0;
  visual_noise.middleRows(3, 4) *= 0.5;

  gps_noise = Eigen::MatrixXf::Identity(7, 7);
  gps_noise.middleRows(0, 3) *= 0.1;
  gps_noise.middleRows(3, 4) *= 0.05;

  Eigen::VectorXf mean(23);
  mean.middleRows(0, 3) = pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3).setZero();
  mean.middleRows(16, 3).setZero();
  mean.middleRows(19, 4) = Eigen::Vector4f(1, 0, 0, 0);

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(23, 23) * 0.1;
  cov.middleRows(19, 4) *= 1e-2; // extrinic rotation imu

  PoseSystem system;
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 23, 6, 7, process_noise, measurement_noise, mean, cov));
  ukf->system.imu_extrinic = Eigen::Quaternionf(imu_extrinic.topLeftCorner<3, 3>()).normalized();
}

PoseEstimator::~PoseEstimator() {}

Eigen::Matrix4d PoseEstimator::predict_nostate(const uint64_t& stamp) {
  std::lock_guard<std::mutex> lock(data_mutex);
  double dt = (double(stamp) - prev_stamp) / 1000000.0;
  if (dt <= 0) {
    return matrix().cast<double>();
  }

  ukf->system.dt = dt;
  Eigen::VectorXf new_state = ukf->system.f(ukf->mean);
  Eigen::Vector3f new_pos(new_state[0], new_state[1], new_state[2]);
  Eigen::Quaternionf new_quat(new_state[6], new_state[7], new_state[8], new_state[9]);
  new_quat.normalized();
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = new_quat.toRotationMatrix();
  m.block<3, 1>(0, 3) = new_pos;
  return m.cast<double>();
}

void PoseEstimator::predict_imu(const uint64_t& pre_stamp, const uint64_t& next_stamp,
                                Eigen::VectorXf &pre_state, Eigen::VectorXf &next_state,
                                const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
  static double dt_smooth = 0;
  double dt = (double(next_stamp) - pre_stamp) / 1000000.0;
  dt = std::max(0.0, std::min(1.0, dt));
  dt_smooth = dt_smooth * 0.95 + dt * 0.05;

  ukf->system.dt = dt_smooth;
  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;

  next_state = ukf->system.f(pre_state, control);
}

bool PoseEstimator::get_timed_pose(RTKType &ins, Eigen::Matrix4d &pose) {
  std::lock_guard<std::mutex> lock(data_mutex);
  if (ins.timestamp <= prev_stamp) {
    return false;
  }

  ins.mean = Eigen::VectorXf(23);
  Eigen::Vector3f acc(ins.acc_x * 9.81, ins.acc_y * 9.81, ins.acc_z * 9.81);
  Eigen::Vector3f gyr(ins.gyro_x / 180.0 * M_PI,
                      ins.gyro_y / 180.0 * M_PI,
                      ins.gyro_z / 180.0 * M_PI);
  if (state_queue.empty()) {
    predict_imu(prev_stamp, ins.timestamp, ukf->mean, ins.mean, acc, gyr);
  } else {
    if (ins.timestamp <= state_queue.back().timestamp) {
      return false;
    }
    auto &last_imu = state_queue.back();
    acc = (acc + Eigen::Vector3f(last_imu.acc_x * 9.81, last_imu.acc_y * 9.81, last_imu.acc_z * 9.81)) / 2.0;
    gyr = (gyr + Eigen::Vector3f(last_imu.gyro_x / 180.0 * M_PI, last_imu.gyro_y / 180.0 * M_PI, last_imu.gyro_z / 180.0 * M_PI)) / 2.0;
    predict_imu(state_queue.back().timestamp, ins.timestamp, state_queue.back().mean, ins.mean, acc, gyr);
  }
  state_queue.push_back(ins);

  Eigen::Vector3f ins_pos(ins.mean[0], ins.mean[1], ins.mean[2]);
  Eigen::Quaternionf ins_quat(ins.mean[6], ins.mean[7], ins.mean[8], ins.mean[9]);
  ins_quat.normalized();
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = ins_quat.toRotationMatrix();
  m.block<3, 1>(0, 3) = ins_pos;
  pose = m.cast<double>();
  return true;
}
/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const uint64_t& stamp) {
  std::lock_guard<std::mutex> lock(data_mutex);
  if ((stamp - init_stamp) / 1000000.0 < cool_time_duration || prev_stamp == 0 || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (double(stamp) - prev_stamp) / 1000000.0;
  prev_stamp = stamp;
  if (dt <= 0 || dt > 1.0) {
    return;
  }

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  ukf->predict();
}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const uint64_t& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
  std::lock_guard<std::mutex> lock(data_mutex);
  if ((stamp - init_stamp) / 1000000.0 < cool_time_duration || prev_stamp == 0 || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (double(stamp) - prev_stamp) / 1000000.0;
  prev_stamp = stamp;
  if (dt <= 0 || dt > 1.0) {
    return;
  }

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;

  ukf->predict(control);
}

bool PoseEstimator::match(Eigen::VectorXf &observation, Eigen::MatrixXf &observation_cov,
                          const uint64_t& stamp, pcl::PointCloud<PointT>::Ptr& cloud,
                          boost::optional<std::shared_ptr<RTKType>> &gps_observation,
                          double &fitness_score) {
  bool result = true;
  Eigen::Matrix4f imu_guess;
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

  Eigen::MatrixXf fused_cov;
  Eigen::VectorXf fused_mean;
  // imu(system) conv, mean
  Eigen::VectorXf imu_mean(7);
  Eigen::MatrixXf imu_cov = Eigen::MatrixXf::Identity(7, 7);
  imu_mean.block<3, 1>(0, 0) = ukf->mean.block<3, 1>(0, 0);
  imu_mean.block<4, 1>(3, 0) = ukf->mean.block<4, 1>(6, 0);

  imu_cov.block<3, 3>(0, 0) = ukf->cov.block<3, 3>(0, 0);
  imu_cov.block<3, 4>(0, 3) = ukf->cov.block<3, 4>(0, 6);
  imu_cov.block<4, 3>(3, 0) = ukf->cov.block<4, 3>(6, 0);
  imu_cov.block<4, 4>(3, 3) = ukf->cov.block<4, 4>(6, 6);

  imu_guess = matrix();
  fused_cov = imu_cov;
  fused_mean = imu_mean;

  // gps fusion
  Eigen::VectorXf gps_mean(7);
  Eigen::MatrixXf gps_cov = Eigen::MatrixXf::Identity(7, 7);
  if (gps_observation) {
    std::shared_ptr<RTKType> gps_obs_ptr = (*gps_observation);
    Eigen::Matrix4f gps_pose = gps_obs_ptr->T.cast<float>();
    gps_cov = gps_noise * gps_obs_ptr->precision;
    Eigen::Quaternionf gps_quat(gps_pose.block<3, 3>(0, 0));
    gps_quat.normalized();
    gps_mean.middleRows(0, 3) = gps_pose.block<3, 1>(0, 3);
    gps_mean.middleRows(3, 4) = Eigen::Vector4f(gps_quat.w(), gps_quat.x(), gps_quat.y(), gps_quat.z());
    gps_mean(2) = fused_mean(2);
    if (gps_obs_ptr->dimension == 6) {
      fusion_pose(fused_cov, gps_cov, fused_mean, gps_mean, fused_cov, fused_mean);
    } else if (gps_obs_ptr->dimension == 2 || gps_obs_ptr->dimension == 3) {
      Eigen::MatrixXf fused_cov_2d = Eigen::MatrixXf::Identity(2, 2);
      Eigen::VectorXf fused_mean_2d(2);
      fusion_pose(fused_cov.block<2, 2>(0, 0), gps_cov.block<2, 2>(0, 0),
                  fused_mean.block<2, 1>(0, 0), gps_mean.block<2, 1>(0, 0),
                  fused_cov_2d, fused_mean_2d, 2);
      fused_cov.block<2, 2>(0, 0) = fused_cov_2d;
      fused_mean.block<2, 1>(0, 0) = fused_mean_2d;
    }
  }

  // align to map
  init_guess.block<3, 1>(0, 3) = Eigen::Vector3f(fused_mean[0], fused_mean[1], fused_mean[2]);
  init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(fused_mean[3], fused_mean[4], fused_mean[5], fused_mean[6]).normalized().toRotationMatrix();

  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  registration->setInputSource(cloud);
  registration->align(*aligned, init_guess);

  if(!registration->hasConverged()) {
    LOG_WARN("localization matching has not converged!!");
    result = false;
  }

#ifdef INTELIPC
  const uint64_t WARM_UP_TIME = std::numeric_limits<uint64_t>::max();
#else
  const uint64_t WARM_UP_TIME = 10000000;
#endif

  // check if wrong global localization result, then fastly fall back to initializing
  if ((stamp - init_stamp) < WARM_UP_TIME) {
    fitness_score = registration->getFitnessScore(fitness_score_max_range);
    LOG_DEBUG("localization fitness score {}", fitness_score);
  }

  Eigen::Matrix4f map_match_observation = registration->getFinalTransformation();
  if(!gps_observation) {
    Eigen::Isometry3f delta = Eigen::Isometry3f(init_guess.inverse() * map_match_observation);
    float dx = delta.translation().norm();
    float da = Eigen::AngleAxisf(delta.linear()).angle() / M_PI * 180;
    if(dx > 5.0 || da > 10.0) {
      LOG_WARN("too large transform!!  {} [m], {} [deg]", dx, da);
      result = false;
    }
  }

  Eigen::Vector3f p = map_match_observation.block<3, 1>(0, 3);
  Eigen::Quaternionf q(map_match_observation.block<3, 3>(0, 0));

  if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
    q.coeffs() *= -1.0f;
  }

  observation.middleRows(0, 3) = p;
  observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
  if (gps_observation) {
    std::shared_ptr<RTKType> gps_obs_ptr = (*gps_observation);
    if (gps_obs_ptr->dimension == 6) {
      fusion_pose(measurement_noise, gps_cov, observation, gps_mean, fused_cov, observation);
    } else if (gps_obs_ptr->dimension == 2 || gps_obs_ptr->dimension == 3) {
      Eigen::MatrixXf fused_cov_2d = Eigen::MatrixXf::Identity(2, 2);
      Eigen::VectorXf fused_mean_2d(2);
      fusion_pose(measurement_noise.block<2, 2>(0, 0), gps_cov.block<2, 2>(0, 0),
                  observation.block<2, 1>(0, 0), gps_mean.block<2, 1>(0, 0),
                  fused_cov_2d, fused_mean_2d, 2);
      observation.block<2, 1>(0, 0) = fused_mean_2d;
      fused_cov.block<2, 2>(0, 0) = fused_cov_2d;
    }
  }
  observation_cov = fused_cov;
  return result;
}

bool PoseEstimator::match(Eigen::VectorXf &observation, Eigen::MatrixXf &observation_cov,
                          boost::optional<std::shared_ptr<RTKType>> &gps_observation) {
  if (!gps_observation || (*gps_observation)->dimension != 6) {
    observation.block<3, 1>(0, 0) = ukf->mean.block<3, 1>(0, 0);
    observation.block<4, 1>(3, 0) = ukf->mean.block<4, 1>(6, 0);
    LOG_WARN("GPS observation is invalid");
    usleep(100000);
    return false;
  }

  Eigen::MatrixXf fused_cov;
  Eigen::VectorXf fused_mean;
  // imu(system) conv, mean
  Eigen::VectorXf imu_mean(7);
  Eigen::MatrixXf imu_cov = Eigen::MatrixXf::Identity(7, 7);
  imu_mean.block<3, 1>(0, 0) = ukf->mean.block<3, 1>(0, 0);
  imu_mean.block<4, 1>(3, 0) = ukf->mean.block<4, 1>(6, 0);

  imu_cov.block<3, 3>(0, 0) = ukf->cov.block<3, 3>(0, 0);
  imu_cov.block<3, 4>(0, 3) = ukf->cov.block<3, 4>(0, 6);
  imu_cov.block<4, 3>(3, 0) = ukf->cov.block<4, 3>(6, 0);
  imu_cov.block<4, 4>(3, 3) = ukf->cov.block<4, 4>(6, 6);

  fused_cov = imu_cov;
  fused_mean = imu_mean;

  Eigen::VectorXf gps_mean(7);
  Eigen::MatrixXf gps_cov = Eigen::MatrixXf::Identity(7, 7);
  std::shared_ptr<RTKType> gps_obs_ptr = (*gps_observation);
  Eigen::Matrix4f gps_pose = gps_obs_ptr->T.cast<float>();
  gps_cov = gps_noise * (10 * gps_obs_ptr->precision);
  Eigen::Quaternionf gps_quat(gps_pose.block<3, 3>(0, 0));
  gps_quat.normalized();
  gps_mean.middleRows(0, 3) = gps_pose.block<3, 1>(0, 3);
  gps_mean.middleRows(3, 4) = Eigen::Vector4f(gps_quat.w(), gps_quat.x(), gps_quat.y(), gps_quat.z());

  fusion_pose(fused_cov, gps_cov, fused_mean, gps_mean, observation_cov, observation);
  return true;
}

/**
 * @brief correct
 * @return cloud aligned to the globalmap
 */
void PoseEstimator::correct(const uint64_t& stamp,
                            Eigen::VectorXf &observation, Eigen::MatrixXf &observation_cov) {
  std::lock_guard<std::mutex> lock(data_mutex);
  last_correction_stamp = stamp;
  prev_stamp = stamp;

  // ukf->setMeasurementNoiseCov(observation_cov);
  ukf->correct(observation);

  // delete out of date states
  auto it = state_queue.begin();
  while (it != state_queue.end()) {
    if (it->timestamp <= stamp) {
      it = state_queue.erase(it);
    } else {
      break;
    }
  }

  // re-compute the predict states
  for (size_t i = 0; i < state_queue.size(); i++) {
    Eigen::Vector3f acc(state_queue[i].acc_x * 9.81, state_queue[i].acc_y * 9.81, state_queue[i].acc_z * 9.81);
    Eigen::Vector3f gyr(state_queue[i].gyro_x / 180.0 * M_PI,
                        state_queue[i].gyro_y / 180.0 * M_PI,
                        state_queue[i].gyro_z / 180.0 * M_PI);
    if (i == 0) {
      predict_imu(stamp, state_queue[i].timestamp, ukf->mean, state_queue[i].mean, acc, gyr);
    } else {
      auto &last_imu = state_queue[i - 1];
      acc = (acc + Eigen::Vector3f(last_imu.acc_x * 9.81, last_imu.acc_y * 9.81, last_imu.acc_z * 9.81)) / 2.0;
      gyr = (gyr + Eigen::Vector3f(last_imu.gyro_x / 180.0 * M_PI, last_imu.gyro_y / 180.0 * M_PI, last_imu.gyro_z / 180.0 * M_PI)) / 2.0;
      predict_imu(state_queue[i - 1].timestamp, state_queue[i].timestamp, state_queue[i - 1].mean, state_queue[i].mean, acc, gyr);
    }
  }
}

/* getters */
uint64_t PoseEstimator::last_correction_time() const {
  return last_correction_stamp;
}

uint64_t PoseEstimator::get_dt() const {
  return uint64_t(ukf->system.dt * 1000000);
}

Eigen::Vector3f PoseEstimator::pos() const {
  return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
}

Eigen::Vector3f PoseEstimator::vel() const {
  return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
}

Eigen::Quaternionf PoseEstimator::quat() const {
  return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
}

Eigen::Matrix4f PoseEstimator::matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = pos();
  return m;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error() const {
  return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error() const {
  return imu_pred_error;
}

void PoseEstimator::fusion_pose(Eigen::MatrixXf cov1, Eigen::MatrixXf cov2,
                                Eigen::VectorXf mean1, Eigen::VectorXf mean2,
                                Eigen::MatrixXf &fused_cov, Eigen::VectorXf &fused_mean, int dim) {
  if (dim == 6) {
    if (mean1.tail<4>().dot(mean2.tail<4>()) < 0.0) {
      mean2.tail<4>() *= -1.0;
    }
  }
  Eigen::MatrixXf inv_cov1 = cov1.inverse();
  Eigen::MatrixXf inv_cov2 = cov2.inverse();

  fused_cov = (inv_cov1 + inv_cov2).inverse();
  fused_mean = fused_cov * inv_cov1 * mean1 + fused_cov * inv_cov2 * mean2;
}
}