#include <mutex>
#include <memory>
#include <atomic>
#include <iostream>
#include <sys/time.h>

#include <pcl/filters/voxel_grid.h>
#include <boost/format.hpp>

#include <hdl_graph_slam/registrations.hpp>
#include <hdl_localization/pose_estimator.hpp>

#include "slam_base.h"
#include "mapping_types.h"
#include "localization_base.h"

using namespace hdl_graph_slam;
using namespace Locate;
using PointT = pcl::PointXYZI;

struct RegistrationType {
  RegistrationType(const pcl::Registration<PointT, PointT>::Ptr& in) {
    registration = in;
  }
  pcl::Registration<PointT, PointT>::Ptr registration;
};

static RWQueue<RegistrationType*> hdlRegistrationQueue;
static pcl::Registration<PointT, PointT>::Ptr registration[2];

namespace hdl_localization {

class HdlLocalizationNodelet {
public:
  HdlLocalizationNodelet() {
  }
  virtual ~HdlLocalizationNodelet() {
  }

  void onInit(InitParameter &param) {
    initialize_params(param);
    pingpong_idx = 0;
    last_pingpong_idx = 1;
    for (int i = 0; i < 2; i++) {
      if (registration[i] == nullptr) {
#ifdef HAVE_CUDA_ENABLE
        registration[i] = select_registration_method("NDT_CUDA", 50000);
#else
        registration[i] = select_registration_method("FAST_VGICP", 50000);
#endif
      }
    }
  }

  void onDeinit() {
    pose_data.clear();
    imu_data.clear();
    ins_data.clear();

    downsample_filter = nullptr;
    pose_estimator.reset(nullptr);
    RegistrationType *registration_ptr = nullptr;
    while (hdlRegistrationQueue.try_dequeue(registration_ptr)) {
        delete registration_ptr;
    }
  }

  void initialize_params(InitParameter &param) {
    mConfig = param;
    if (param.resolution >= 0.1) {
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(param.resolution, param.resolution, param.resolution);
      downsample_filter.reset(voxelgrid);
    }
  }

  void setImuExtrinic(Eigen::Matrix4d &extrinic) {
    imu_extrinic = extrinic;
  }

  void ins_callback(std::shared_ptr<RTKType> &ins) {
    std::lock_guard<std::mutex> lock(ins_data_mutex);
    if (ins_data.size() >= 10) {
      ins_data.erase(ins_data.begin());
    }
    ins_data.push_back(ins);
  }

  /**
   * @brief callback for imu data
   * @param imu
   */
  void imu_callback(ImuType& imu) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu);
  }

  void update_pose(uint64_t stamp, const Eigen::Matrix4d& pose) {
    std::lock_guard<std::mutex> lock(pose_data_mutex);
    while (pose_data.size() >= 10) {
      pose_data.erase(pose_data.begin());
    }

    std::shared_ptr<PoseType> pose_ptr(new PoseType());
    pose_ptr->timestamp = stamp;
    pose_ptr->T = pose;
    pose_data.push_back(pose_ptr);
  }

  bool get_timed_pose(uint64_t timestamp, Eigen::Matrix4d &pose) {
    pose_data_mutex.lock();
    std::vector<std::shared_ptr<PoseType>> pose_data_snapshot = pose_data;
    pose_data_mutex.unlock();
    if (pose_data_snapshot.empty()) {
      LOG_WARN("pose data is empty");
      return false;
    }

    if (timestamp < pose_data_snapshot.front()->timestamp) {
      LOG_WARN("query timestamp is out of date, {}, {}", timestamp, pose_data_snapshot.front()->timestamp);
      return false;
    }

    if (timestamp > pose_data_snapshot.back()->timestamp) {
      if ((timestamp - pose_data_snapshot.back()->timestamp) > 1000000) {
        // LOG_WARN("query timestamp is in the future (>1s)");
        return false;
      }
      // predict
      if (pose_estimator == nullptr) {
        return false;
      }
      pose = pose_estimator->predict_nostate(timestamp);
    } else {
      if (pose_data_snapshot.size() < 2) {
        LOG_WARN("need two pose data for interplate");
        return false;
      }
      // interplate
      auto first_pose  = pose_data_snapshot.begin();
      auto second_pose = pose_data_snapshot.begin();
      for(; second_pose != pose_data_snapshot.end(); second_pose++) {
        auto dt = ((*first_pose)->timestamp - double(timestamp)) / 1000000.0;
        auto dt2 = ((*second_pose)->timestamp - double(timestamp)) / 1000000.0;
        if(dt <= 0 && dt2 >= 0) {
          break;
        }
        first_pose = second_pose;
      }
      double t_diff_ratio = static_cast<double>(timestamp - (*first_pose)->timestamp) /
                            static_cast<double>((*second_pose)->timestamp - (*first_pose)->timestamp + 1);
      pose = interpolateTransform((*first_pose)->T, (*second_pose)->T, t_diff_ratio);
    }
    return true;
  }

  bool get_timed_pose(RTKType &ins, Eigen::Matrix4d &pose) {
    bool result = false;
    if (pose_estimator && pose_estimator->is_registration()) {
      result = pose_estimator->get_timed_pose(ins, pose);
    }

    return result;
  }

  /**
   * @brief callback for point cloud data
   * @param cloud
   */
  LocType frame_callback(PointCloudAttrPtr& cloud, ImageType& image, Eigen::Isometry3d& pose) {
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      LOG_WARN("waiting for initial pose input!!");
      return LocType::OTHER;
    }

    if(cloud->cloud->empty()) {
      LOG_WARN("cloud is empty!!");
      return LocType::OTHER;
    }

    const uint64_t& stamp = cloud->cloud->header.stamp;

    // update sub-map registration
    RegistrationType* registration_ptr;
    bool has_data = hdlRegistrationQueue.try_dequeue(registration_ptr);
    if (has_data) {
      pose_estimator->set_registration(registration_ptr->registration);
      delete registration_ptr;
      last_pingpong_idx.store((last_pingpong_idx + 1) % 2);
    }

    // imu process
    bool use_imu = false;
    imu_data_mutex.lock();
    auto it = imu_data.begin();
    int imu_data_num = 0;
    Eigen::Vector3f acc_mean(0, 0, 0);
    Eigen::Vector3f gyr_mean(0, 0, 0);
    for(it; it != imu_data.end(); it++) {
      if(stamp < uint64_t(it->stamp * 1000000.0)) {
        break;
      }
      acc_mean = acc_mean + it->acc.cast<float>();
      gyr_mean = gyr_mean + it->gyr.cast<float>();
      imu_data_num++;
    }
    if (imu_data_num != 0) {
      acc_mean = acc_mean / imu_data_num;
      gyr_mean = gyr_mean / imu_data_num;
      use_imu = true;
    }
    imu_data.erase(imu_data.begin(), it);
    imu_data_mutex.unlock();

    // predict
    if(!use_imu) {
      pose_estimator->predict(stamp);
    } else {
      pose_estimator->predict(stamp, acc_mean, gyr_mean);
    }

    // undistortion points
    Eigen::Matrix4d start_pose, stop_pose;
    if (get_timed_pose(stamp, start_pose) && get_timed_pose(stamp + pose_estimator->get_dt(), stop_pose)) {
      undistortPoints((start_pose.inverse() * stop_pose).cast<float>(), cloud, mConfig.scan_period);
    }

    auto filtered = downsample(cloud->cloud);

    // ins process
    boost::optional<std::shared_ptr<RTKType>> gps_observation;
    ins_data_mutex.lock();
    if (!ins_data.empty()) {
      if (stamp > ins_data.back()->timestamp) {
        ins_data.clear();
      } else if (stamp >= ins_data.front()->timestamp) {
        bool gps_data_valid = false;
        auto first_ins  = ins_data.begin();
        auto second_ins = ins_data.begin();
        for(; second_ins != ins_data.end(); second_ins++) {
          auto dt = ((*first_ins)->timestamp - double(stamp)) / 1000000.0;
          auto dt2 = ((*second_ins)->timestamp - double(stamp)) / 1000000.0;
          if(dt <= 0 && dt2 >= 0  && (dt2 - dt) < 0.2) {
            gps_data_valid = true;
            break;
          }
          first_ins = second_ins;
        }
        if (gps_data_valid) {
          std::shared_ptr<RTKType> gps_obs_ptr(new RTKType());
          double t_diff_ratio = static_cast<double>(stamp - (*first_ins)->timestamp) /
                                static_cast<double>((*second_ins)->timestamp - (*first_ins)->timestamp + 1);
          gps_obs_ptr->precision = (*first_ins)->precision;
          gps_obs_ptr->dimension = (*first_ins)->dimension;
          gps_obs_ptr->T = interpolateTransform((*first_ins)->T, (*second_ins)->T, t_diff_ratio);
          gps_observation = gps_obs_ptr;
        }
      }
    }
    ins_data_mutex.unlock();

    // correct
    double fitness_score = 0;
    Eigen::VectorXf observation(7);
    Eigen::MatrixXf observation_cov = Eigen::MatrixXf::Identity(7, 7);
    bool result = false;
    if(pose_estimator->is_registration()) {
      result = pose_estimator->match(observation, observation_cov, stamp, filtered, gps_observation, fitness_score);
    } else {
      result = pose_estimator->match(observation, observation_cov, gps_observation);
    }

    pose_estimator->correct(stamp, observation, observation_cov);
    pose = Eigen::Isometry3d(pose_estimator->matrix().cast<double>());
    update_pose(stamp, pose.matrix());

    // check registration fitness score
    if (fitness_score > 1.0) {
      LOG_WARN("localization fitness score is too large, score {}", fitness_score);
      result = false;
    }

    return (result ? LocType::OK : LocType::ERROR);
  }

  /**
   * @brief callback for globalmap input
   * @param cloud
   */
  void globalmap_callback(PointCloud::Ptr& cloud) {
    if (pingpong_idx == last_pingpong_idx) {
      LOG_WARN("localization previous sub-map has not been processed");
      return;
    }

    RegistrationType *registration_ptr = nullptr;
    if (cloud != nullptr && !cloud->empty()) {
      registration[pingpong_idx]->setInputTarget(cloud);
      registration_ptr = new RegistrationType(registration[pingpong_idx]);
    } else {
      registration_ptr = new RegistrationType(nullptr);
    }

    hdlRegistrationQueue.enqueue(registration_ptr);
    pingpong_idx.store(last_pingpong_idx);
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose
   */
  void initialpose_callback(uint64_t stamp, const Eigen::Matrix4d& pose) {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    Eigen::Vector3d p = pose.topRightCorner<3, 1>();
    Eigen::Quaterniond q = Eigen::Quaterniond(pose.topLeftCorner<3, 3>());
    q.normalize();
    pose_estimator.reset(
          new hdl_localization::PoseEstimator(
            imu_extrinic.cast<float>(),
            stamp,
            p.cast<float>(),
            q.cast<float>(),
            0.2)
    );
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::Ptr downsample(pcl::PointCloud<PointT>::Ptr& cloud) {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

private:
  InitParameter mConfig;
  std::mutex pose_data_mutex;
  std::vector<std::shared_ptr<PoseType>> pose_data;
  // ins input buffer
  std::mutex ins_data_mutex;
  std::vector<std::shared_ptr<RTKType>> ins_data;
  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<ImuType> imu_data;

  pcl::Filter<PointT>::Ptr downsample_filter;

  // pose estimator
  Eigen::Matrix4d imu_extrinic;
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  std::atomic<int> pingpong_idx;
  std::atomic<int> last_pingpong_idx;
};
}

hdl_localization::HdlLocalizationNodelet hdlLocalizationNode;

void init_hdl_localization_node(InitParameter &param) {
  hdlLocalizationNode.onInit(param);
}

void deinit_hdl_localization_node() {
  hdlLocalizationNode.onDeinit();
}

void set_imu_extrinic_hdl_localization(Eigen::Matrix4d extrinic) {
  hdlLocalizationNode.setImuExtrinic(extrinic);
}

void set_initpose_hdl_localization(uint64_t stamp, const Eigen::Matrix4d& pose) {
  hdlLocalizationNode.initialpose_callback(stamp, pose);
}

void set_map_hdl_localization(PointCloud::Ptr& cloud) {
  hdlLocalizationNode.globalmap_callback(cloud);
}

void enqueue_ins_hdl_localization(std::shared_ptr<RTKType> &ins) {
  hdlLocalizationNode.ins_callback(ins);
}

void enqueue_imu_hdl_localization(ImuType& imu) {
  hdlLocalizationNode.imu_callback(imu);
}

bool get_timed_pose_hdl_localization(uint64_t timestamp, Eigen::Matrix4d &pose) {
  return hdlLocalizationNode.get_timed_pose(timestamp, pose);
}

bool get_timed_pose_hdl_localization(RTKType &ins, Eigen::Matrix4d &pose) {
  return hdlLocalizationNode.get_timed_pose(ins, pose);
}

LocType enqueue_hdl_localization(PointCloudAttrPtr& cloud, ImageType& image, Eigen::Isometry3d& pose) {
  return hdlLocalizationNode.frame_callback(cloud, image, pose);
}
