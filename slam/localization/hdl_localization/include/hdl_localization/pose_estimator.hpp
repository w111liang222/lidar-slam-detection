#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <mutex>
#include <memory>
#include <boost/optional.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

#include "mapping_types.h"

namespace kkl {
  namespace alg {
    template<typename T, class System> class UnscentedKalmanFilterX;
  }
}

namespace hdl_localization {

class PoseSystem;

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief constructor
   * @param registration        registration method
   * @param stamp               timestamp
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  PoseEstimator(const Eigen::Matrix4f imu_ext, const uint64_t& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0);
  ~PoseEstimator();

  void set_registration(pcl::Registration<PointT, PointT>::Ptr& in) {
    registration = in;
    is_set_registration = true;
  }

  void reset_registration() {
    is_set_registration = false;
    registration = nullptr;
  }

  bool is_registration() {
    return (is_set_registration && registration != nullptr);
  }

  Eigen::Matrix4d predict_nostate(const uint64_t& stamp);

  void predict_imu(const uint64_t& pre_stamp, const uint64_t& next_stamp,
                   Eigen::VectorXf &pre_state, Eigen::VectorXf &next_state,
                   const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

  bool get_timed_pose(RTKType &ins, Eigen::Matrix4d &pose);
  /**
   * @brief predict
   * @param stamp    timestamp
   */
  void predict(const uint64_t& stamp);

  /**
   * @brief predict
   * @param stamp    timestamp
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(const uint64_t& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

  bool match(Eigen::VectorXf &observation, Eigen::MatrixXf &observation_cov,
             const uint64_t& stamp, pcl::PointCloud<PointT>::Ptr& cloud,
             boost::optional<std::shared_ptr<RTKType>> &gps_observation,
             double &fitness_score);
  bool match(Eigen::VectorXf &observation, Eigen::MatrixXf &observation_cov,
             boost::optional<std::shared_ptr<RTKType>> &gps_observation);
  /**
   * @brief correct
   * @param cloud   input cloud
   */
  void correct(const uint64_t& stamp,
               Eigen::VectorXf &observation, Eigen::MatrixXf &observation_cov);

  /* getters */
  uint64_t last_correction_time() const;
  uint64_t get_dt() const;

  Eigen::Vector3f pos() const;
  Eigen::Vector3f vel() const;
  Eigen::Quaternionf quat() const;
  Eigen::Matrix4f matrix() const;

  const boost::optional<Eigen::Matrix4f>& wo_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& imu_prediction_error() const;

  void fusion_pose(Eigen::MatrixXf cov1, Eigen::MatrixXf cov2,
                   Eigen::VectorXf mean1, Eigen::VectorXf mean2,
                   Eigen::MatrixXf &fused_cov, Eigen::VectorXf &fused_mean, int dim = 6);

private:
  uint64_t init_stamp;             // when the estimator was initialized
  uint64_t prev_stamp;             // when the estimator was updated last time
  uint64_t last_correction_stamp;  // when the estimator performed the correction step
  double cool_time_duration;        //

  Eigen::Matrix4f imu_extrinic;

  Eigen::MatrixXf process_noise;
  Eigen::MatrixXf measurement_noise;
  Eigen::MatrixXf visual_noise;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;

  Eigen::MatrixXf gps_noise;

  boost::optional<Eigen::Matrix4f> wo_pred_error;
  boost::optional<Eigen::Matrix4f> imu_pred_error;

  std::mutex data_mutex;
  pcl::Registration<PointT, PointT>::Ptr registration;
  bool is_set_registration;

  std::vector<RTKType> state_queue;
  };

}  // namespace hdl_localization

#endif  // POSE_ESTIMATOR_HPP
