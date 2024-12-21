#include "fastlio.h"

#include <sys/prctl.h>
#include "backend_api.h"

using namespace Mapping;

// FastLIO declaration
int  fastlio_init(std::vector<double> &extT, std::vector<double>& extR, int filter_num, int max_point_num, double scan_period, bool undistort);
bool fastlio_is_init();
void fastlio_imu_enqueue(ImuType imu);
void fastlio_ins_enqueue(bool rtk_valid, RTKType ins);
void fastlio_pcl_enqueue(PointCloudAttrPtr &points);
bool fastlio_main();
void fastlio_odometry(Eigen::Matrix4d &odom_s, Eigen::Matrix4d &odom_e);
std::vector<double> fastlio_state();

std::vector<PoseType> prediction(std::pair<Eigen::Isometry3d, Eigen::Isometry3d> &odomtry, PointCloudAttrPtr &frame, std::vector<ImuType> &imus, Eigen::Matrix4d &staticTrans) {
  // prediction from start frame stamp to end frame
  std::vector<double> state = fastlio_state();
  PoseType last_pose;
  last_pose.timestamp = frame->cloud->header.stamp;
  last_pose.T = Eigen::Matrix4d::Identity();
  last_pose.T.topRightCorner<3, 1>() = Eigen::Vector3d(state[0], state[1], state[2]);
  last_pose.T.topLeftCorner<3, 3>()  = Eigen::Quaterniond(state[6], state[3], state[4], state[5]).normalized().toRotationMatrix();

  std::vector<PoseType> imu_poses;
  imu_poses.push_back(last_pose);

  Eigen::Vector3d last_vec = Eigen::Vector3d(state[7], state[8], state[9]);
  Eigen::Vector3d last_acc = imus[0].acc;
  Eigen::Vector3d last_gyr = imus[0].gyr;
  for (int i = 0; i < imus.size(); i++) {
    Eigen::Vector3d acc_mean = 0.5 * (last_acc + imus[i].acc);
    Eigen::Vector3d gyr_mean = 0.5 * (last_gyr + imus[i].gyr);
    acc_mean    = acc_mean    / state[19];
    acc_mean[0] = acc_mean[0] - state[10];
    acc_mean[1] = acc_mean[1] - state[11];
    acc_mean[2] = acc_mean[2] - state[12];
    gyr_mean[0] = gyr_mean[0] - state[13];
    gyr_mean[1] = gyr_mean[1] - state[14];
    gyr_mean[2] = gyr_mean[2] - state[15];

    PoseType pose;
    pose.timestamp = static_cast<uint64_t>(imus[i].stamp * 1000000);
    double dt = (pose.timestamp - imu_poses[i].timestamp) / 1000000.0;

    Eigen::Quaterniond rot = Eigen::Quaterniond(imu_poses[i].T.topLeftCorner<3, 3>()).normalized();
    rot = rot * Eigen::Quaterniond(1, gyr_mean[0] * dt / 2, gyr_mean[1] * dt / 2, gyr_mean[2] * dt / 2).normalized();

    acc_mean    = rot * acc_mean;
    acc_mean[0] = acc_mean[0] + state[16];
    acc_mean[1] = acc_mean[1] + state[17];
    acc_mean[2] = acc_mean[2] + state[18];

    pose.T = Eigen::Matrix4d::Identity();
    pose.T.topRightCorner<3, 1>() = imu_poses[i].T.topRightCorner<3, 1>() + last_vec * dt + 0.5 * acc_mean * dt * dt;
    pose.T.topLeftCorner<3, 3>()  = rot.toRotationMatrix();
    last_vec = last_vec + acc_mean * dt;
    last_acc = imus[i].acc;
    last_gyr = imus[i].gyr;
    imu_poses.push_back(pose);
  }

  // transform from imu to lidar
  for (int i = 0; i < imu_poses.size(); i++) {
    imu_poses[i].T = staticTrans.inverse() * imu_poses[i].T * staticTrans;
  }
  for (int i = imu_poses.size() - 1; i >= 0; i--) {
    imu_poses[i].T = imu_poses[0].T.inverse() * imu_poses[i].T;
  }

  Eigen::Matrix4d delta_odom = (odomtry.first.inverse() * odomtry.second).matrix();
  Eigen::Matrix4d delta_delta_odom = imu_poses.back().T.inverse() * delta_odom;
  Eigen::Vector3d delta_translation = delta_delta_odom.block<3, 1>(0, 3);
  Eigen::Quaterniond delta_rotation(delta_delta_odom.block<3, 3>(0, 0));
  Eigen::AngleAxisd angle_axis(delta_rotation);

  double duration = (imu_poses.back().timestamp - imu_poses.front().timestamp) / 1000000.0;
  for (int i = 0; i < imu_poses.size(); i++) {
    double t_diff_ratio = (imu_poses[i].timestamp - imu_poses[0].timestamp) / 1000000.0 / duration;
    Eigen::Matrix<double, 6, 1> log_vector = t_diff_ratio * (Eigen::Matrix<double, 6, 1>() << delta_translation, angle_axis.angle() * angle_axis.axis()).finished();
    constexpr double kEpsilon = 1e-8;
    const float norm = log_vector.tail<3>().norm();
    Eigen::Matrix4d plus_odom = Eigen::Matrix4d::Identity();
    if (norm < kEpsilon) {
        Eigen::Vector3d new_translation = log_vector.head<3>();
        Eigen::Quaterniond new_rotation(Eigen::Quaterniond::Identity());
        plus_odom.block<3, 1>(0, 3) = new_translation;
        plus_odom.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
    } else {
        Eigen::Vector3d new_translation = log_vector.head<3>();
        Eigen::Quaterniond new_rotation(Eigen::AngleAxisd(norm,  log_vector.tail<3>() / norm));
        plus_odom.block<3, 1>(0, 3) = new_translation;
        plus_odom.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
    }
    imu_poses[i].T = imu_poses[i].T * plus_odom;
  }

  return imu_poses;
}

HDL_FastLIO::HDL_FastLIO() : SlamBase(), mOriginIsSet(false), mLidarName("") {
}

HDL_FastLIO::~HDL_FastLIO() {
  mThreadStart = false;
  mLioThread->join();
  mLioThread.reset(nullptr);
  mFloorThread->join();
  mFloorThread.reset(nullptr);
  mGraphThread->join();
  mGraphThread.reset(nullptr);
  LOG_INFO("FastLIO thread join success");
  deinit_backend();
}

std::vector<std::string> HDL_FastLIO::setSensors(std::vector<std::string> &sensors) {
  std::vector<std::string> sensor_list;

  bool is_imu_set = false;
  // check IMU is set for use
  for (auto sensor : sensors) {
    if (sensor.compare("RTK") == 0) {
      sensor_list.push_back(sensor);
    } else if (sensor.compare("IMU") == 0) {
      sensor_list.push_back(sensor);
      is_imu_set = true;
    }
  }

  if (!is_imu_set) {
    return sensor_list;
  }

  bool is_lidar_set = false;
  for (auto sensor : sensors) {
    if (sensor.compare("RTK") == 0 || sensor.compare("IMU") == 0) {
      continue;
    } else if (sensor.length() < 2 || sensor[1] != '-') { // not a "n-" pattern -> CameraName
      sensor_list.push_back(sensor);
    } else {
      sensor_list.push_back(sensor);
      if (!is_lidar_set) {
        is_lidar_set = true;
        mLidarName = sensor;
      }
    }
  }
  return sensor_list;
}

bool HDL_FastLIO::init(InitParameter &param) {
  mConfig = param;
  // fast lio init
  mImuInsStaticTrans = mImuStaticTrans * mStaticTrans.inverse();
  std::vector<double> extrinsicT = {mImuInsStaticTrans(0, 3), mImuInsStaticTrans(1, 3), mImuInsStaticTrans(2, 3)};
  std::vector<double> extrinsicR = {mImuInsStaticTrans(0, 0), mImuInsStaticTrans(0, 1), mImuInsStaticTrans(0, 2),
                                    mImuInsStaticTrans(1, 0), mImuInsStaticTrans(1, 1), mImuInsStaticTrans(1, 2),
                                    mImuInsStaticTrans(2, 0), mImuInsStaticTrans(2, 1), mImuInsStaticTrans(2, 2)};
  fastlio_init(extrinsicT, extrinsicR, 1, -1, mConfig.scan_period, true);

  // backend init
  init_backend(param);

  mThreadStart = true;
  mLioThread.reset(new std::thread(&HDL_FastLIO::runLio, this));
  mFloorThread.reset(new std::thread(&HDL_FastLIO::runFloor, this));
  mGraphThread.reset(new std::thread(&HDL_FastLIO::runGraph, this));
  return true;
}

bool HDL_FastLIO::originIsSet() { return mOriginIsSet; }

RTKType &HDL_FastLIO::getOrigin() { return mOrigin; }

void HDL_FastLIO::setOrigin(RTKType rtk) {
  if (!mOriginIsSet) {
    graph_set_origin(rtk);
    mOriginIsSet = true;
    mOrigin = rtk;
  }
}

void HDL_FastLIO::feedInsData(bool rtk_valid, std::shared_ptr<RTKType> ins) {
  enqueue_graph_gps(rtk_valid, ins);
  fastlio_ins_enqueue(rtk_valid, *ins);
}

void HDL_FastLIO::feedImuData(ImuType &imu) {
  std::lock_guard<std::mutex> lock(mImuDataMutex);
  mImuData.push_back(imu);
  fastlio_imu_enqueue(imu);
}

void HDL_FastLIO::feedImageData(const uint64_t &timestamp,
                                std::map<std::string, ImageType> &images,
                                std::map<std::string, cv::Mat> &images_stream) {
  mImages = images;
  mImagesStream = images_stream;
}

void HDL_FastLIO::feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) {
  // transform to INS coordinate
  mFrameAttr = points[mLidarName];
  preprocessPoints(mFrameAttr->cloud, mFrameAttr->cloud);

  // enqueue to FastLIO engine and floor detection
  fastlio_pcl_enqueue(mFrameAttr);
  mFloorQueue.enqueue(new PointCloudType(mFrameAttr->cloud));
}

Eigen::Matrix4d HDL_FastLIO::getPose(PointCloudAttrImagePose &frame) {
  // wait for odometry
  std::pair<Eigen::Isometry3d, Eigen::Isometry3d> odomtry(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
  mOdomQueue.wait_dequeue_timed(odomtry, 10000000); // 10s

  // process imu data to undistortion the cloud
  std::vector<ImuType> sync_imu;
  mImuDataMutex.lock();
  auto it = mImuData.begin();
  for(it; it != mImuData.end(); it++) {
    if ((mFrameAttr->cloud->header.stamp / 1000000.0 + mConfig.scan_period) <= it->stamp) {
      break;
    }
    if (mFrameAttr->cloud->header.stamp < uint64_t(it->stamp * 1000000.0)) {
      sync_imu.push_back(*it);
    }
  }
  mImuData.erase(mImuData.begin(), it);
  mImuDataMutex.unlock();

  // predict the imu pose
  std::vector<PoseType> imu_poses;
  if (sync_imu.size() > 0 && fastlio_is_init()) {
    ImuType last_imu = sync_imu.back();
    last_imu.stamp = mFrameAttr->cloud->header.stamp / 1000000.0 + mConfig.scan_period;
    sync_imu.push_back(last_imu);
    imu_poses = prediction(odomtry, mFrameAttr, sync_imu, mImuInsStaticTrans);
  }

  // push to odometry vector
  for (int i = 0; i < int(imu_poses.size()) - 1; i++) {
    PoseType odometry = imu_poses[i];
    odometry.T = odomtry.first * odometry.T;
    if (!mOdometrys.empty() && mOdometrys.back().timestamp >= odometry.timestamp) {
      continue;
    }
    mOdometrys.emplace_back(odometry);
  }

  // combine whole frame
  mFrameAttr->T = (odomtry.first.inverse() * odomtry.second).matrix(); // delta odometry
  frame = PointCloudAttrImagePose(mFrameAttr, mImages, mImagesStream, imu_poses, odomtry.first);

  return (get_odom2map() * odomtry.first).matrix();
}

std::vector<PoseType> HDL_FastLIO::getOdometrys() {
  return mOdometrys;
}

void HDL_FastLIO::runLio() {
  prctl(PR_SET_NAME, "FLIO Odom", 0, 0, 0);
  while (mThreadStart) {
    if (fastlio_main()) {
      Eigen::Matrix4d odom_s, odom_e;
      fastlio_odometry(odom_s, odom_e);
      odom_s = mImuInsStaticTrans.inverse() * odom_s * mImuInsStaticTrans;
      odom_e = mImuInsStaticTrans.inverse() * odom_e * mImuInsStaticTrans;

      mOdomQueue.enqueue(std::make_pair<Eigen::Isometry3d, Eigen::Isometry3d>(Eigen::Isometry3d(odom_s), Eigen::Isometry3d(odom_e)));
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void HDL_FastLIO::runFloor() {
  prctl(PR_SET_NAME, "FLIO Floor", 0, 0, 0);
  while (mThreadStart) {
    PointCloudType *points = nullptr;
    if (false == mFloorQueue.wait_dequeue_timed(points, 100000)) {
      continue;
    }
    std::shared_ptr<PointCloudType> pointPtr = std::shared_ptr<PointCloudType>(points);

    PointCloud::Ptr filtered = enqueue_filter(pointPtr->points);
    FloorCoeffs floor = enqueue_floor(filtered);
    enqueue_graph_floor(floor);
  }
}

void HDL_FastLIO::runGraph() {
  prctl(PR_SET_NAME, "FLIO Graph", 0, 0, 0);
  while (mThreadStart) {
    auto clock = std::chrono::steady_clock::now();
    graph_optimization(mThreadStart);
    auto elapseMs = since(clock).count();

    int sleep_count = 0;
    int time_to_sleep = std::max(int((3000 - elapseMs) / 100), 1);
    while (sleep_count < time_to_sleep && mThreadStart) {
      usleep(100000);
      sleep_count++;
    }
    if (!mThreadStart) {
      break;
    }
  }
}