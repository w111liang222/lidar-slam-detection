#include "fastlio.h"

#include <sys/prctl.h>
#include "backend_api.h"

using namespace Mapping;

// FastLIO declaration
int  fastlio_init(std::vector<double> &extT, std::vector<double>& extR, int filter_num, int max_point_num, bool undistort);
void fastlio_imu_enqueue(ImuType imu);
void fastlio_pcl_enqueue(PointCloudAttrPtr &points, bool sync);
bool fastlio_main();
std::vector<double> get_fastlio_odom();
std::vector<double> get_fastlio_state();

void undistortion(uint64_t last_stamp, PointCloudAttrPtr &frame, std::vector<ImuType> &imus, Eigen::Matrix4d &staticTrans) {
  // prediction from last frame start stamp
  std::vector<double> state = get_fastlio_state();
  PoseType last_pose;
  last_pose.timestamp = last_stamp;
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

  // erase the poses before current frame
  auto it = imu_poses.begin();
  while (it->timestamp < frame->cloud->header.stamp) {
    it = imu_poses.erase(it);
  }

  // transform from imu to lidar
  for (int i = 0; i < imu_poses.size(); i++) {
    imu_poses[i].T = staticTrans.inverse() * imu_poses[i].T * staticTrans;
  }
  for (int i = imu_poses.size() - 1; i >= 0; i--) {
    imu_poses[i].T = imu_poses[0].T.inverse() * imu_poses[i].T;
  }
  undistortPoints(imu_poses, frame);
}

HDL_FastLIO::HDL_FastLIO() : SlamBase(), mOriginIsSet(false) {
  mLidarName = "";
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
  mLastStamp = 0;
  mLastOdom.setIdentity();
  mLastFrame.T.setIdentity();
  // fast lio init
  mImuInsStaticTrans = mImuStaticTrans * mStaticTrans.inverse();
  std::vector<double> extrinsicT = {mImuInsStaticTrans(0, 3), mImuInsStaticTrans(1, 3), mImuInsStaticTrans(2, 3)};
  std::vector<double> extrinsicR = {mImuInsStaticTrans(0, 0), mImuInsStaticTrans(0, 1), mImuInsStaticTrans(0, 2),
                                    mImuInsStaticTrans(1, 0), mImuInsStaticTrans(1, 1), mImuInsStaticTrans(1, 2),
                                    mImuInsStaticTrans(2, 0), mImuInsStaticTrans(2, 1), mImuInsStaticTrans(2, 2)};
  fastlio_init(extrinsicT, extrinsicR, 1, -1, false);

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

void HDL_FastLIO::feedInsData(std::shared_ptr<RTKType> ins) {
  enqueue_graph_gps(ins);
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
  mFrameAttr = mergePoints(mLidarName, points, uint64_t(mConfig.scan_period * 1000000.0));
  preprocessPoints(mFrameAttr->cloud, mFrameAttr->cloud);

  // store raw pointcloud
  mFrame = mFrameAttr->cloud;
  mFrameAttr->cloud = PointCloud::Ptr(new PointCloud(*mFrame));

  // process imu data
  std::vector<ImuType> sync_imu;
  mImuDataMutex.lock();
  auto it_erase = mImuData.begin();
  uint64_t last_imu_stamp = 0;
  for(auto it = mImuData.begin(); it != mImuData.end(); it++) {
    if(mFrame->header.stamp < uint64_t((it->stamp - mConfig.scan_period) * 1000000.0)) {
      break;
    }
    if (mLastStamp < uint64_t(it->stamp * 1000000.0)) {
      // check and insert the stamp point of current frame
      if (last_imu_stamp < mFrame->header.stamp && mFrame->header.stamp < uint64_t(it->stamp * 1000000.0)) {
        ImuType imu = *it;
        if (sync_imu.size() > 0) {
          imu.acc = 0.5 * (imu.acc + sync_imu.back().acc);
          imu.gyr = 0.5 * (imu.gyr + sync_imu.back().gyr);
        }
        imu.stamp = mFrame->header.stamp / 1000000.0;
        sync_imu.push_back(imu);
      }
      sync_imu.push_back(*it);
    } else {
      it_erase = it;
    }
    last_imu_stamp = uint64_t(it->stamp * 1000000);
  }
  mImuData.erase(mImuData.begin(), it_erase);
  mImuDataMutex.unlock();

  if (sync_imu.size() > 0) {
    ImuType last_imu = sync_imu.back();
    last_imu.stamp = mFrame->header.stamp / 1000000.0 + mConfig.scan_period;
    sync_imu.push_back(last_imu);
    undistortion(mLastStamp, mFrameAttr, sync_imu, mImuInsStaticTrans);
  }

  mLastStamp = mFrame->header.stamp;
  // enqueue to FastLIO engine and floor detection
  fastlio_pcl_enqueue(mFrameAttr, false);
  mFloorQueue.enqueue(new PointCloudType(mFrameAttr->cloud));
}

Eigen::Matrix4d HDL_FastLIO::getPose(PointCloudAttrImagePose &frame) {
  // wait for odometry
  Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  mOdomQueue.wait_dequeue_timed(odom, 1000000);
  Eigen::Matrix4d delta_odom = mLastOdom.inverse() * odom.matrix();
  mLastOdom = odom.matrix();

  // output the last frame (delay 1 frame)
  frame = mLastFrame;
  frame.points->T = delta_odom;

  // combine whole frame
  mFrameAttr->cloud = mFrame; // restore raw pointcloud
  mLastFrame = PointCloudAttrImagePose(mFrameAttr, mImages, mImagesStream, odom);

  return (get_odom2map() * odom).matrix();
}

void HDL_FastLIO::runLio() {
  prctl(PR_SET_NAME, "FLIO Odom", 0, 0, 0);
  while (mThreadStart) {
    if (fastlio_main()) {
      std::vector<double> state = get_fastlio_odom();

      Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
      odom.topRightCorner<3, 1>() = Eigen::Vector3d(state[0], state[1], state[2]);
      odom.topLeftCorner<3, 3>()  = Eigen::Quaterniond(state[6], state[3], state[4], state[5]).normalized().toRotationMatrix();
      odom = mImuInsStaticTrans.inverse() * odom * mImuInsStaticTrans;

      mOdomQueue.enqueue(Eigen::Isometry3d(odom));
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