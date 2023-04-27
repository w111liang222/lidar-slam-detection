#include "fastlio.h"

#include <sys/prctl.h>
#include "backend_api.h"

using namespace Mapping;

// FastLIO declaration
int  fastlio_init(std::vector<double> &extT, std::vector<double>& extR, int filter_num, int max_point_num, bool undistort);
void fastlio_imu_enqueue(ImuType &imu);
void fastlio_pcl_enqueue(PointCloudAttrPtr &points, bool sync);
bool fastlio_main();
std::vector<double> get_fastlio_odom();

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
      if (!is_lidar_set) {
        is_lidar_set = true;
        mLidarName = sensor;
        sensor_list.push_back(sensor);
      }
    }
  }
  return sensor_list;
}

bool HDL_FastLIO::init(InitParameter &param) {
  mConfig = param;
  mLastOdom.setIdentity();
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
                                std::map<std::string, cv::Mat> &images,
                                std::map<std::string, cv::Mat> &images_stream) {
  mImages = images;
  mImagesStream = images_stream;
}

void HDL_FastLIO::feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) {
  // transform to INS coordinate
  mFrameAttr = points[mLidarName];
  preprocessPoints(mFrameAttr->cloud, mFrameAttr->cloud);

  // store raw pointcloud
  mFrame = mFrameAttr->cloud;
  mFrameAttr->cloud = PointCloud::Ptr(new PointCloud(*mFrame));

  // process imu data
  mImuDataMutex.lock();
  bool use_imu = false;
  auto it = mImuData.begin();
  int imu_data_num = 0;
  Eigen::Vector3d acc_mean(0, 0, 0);
  Eigen::Vector3d gyr_mean(0, 0, 0);
  for(it; it != mImuData.end(); it++) {
    if(mFrame->header.stamp < uint64_t((it->stamp - mConfig.scan_period) * 1000000.0)) {
      break;
    }
    acc_mean = acc_mean + it->acc;
    gyr_mean = gyr_mean + it->gyr;
    imu_data_num++;
  }
  if (imu_data_num != 0) {
    acc_mean = acc_mean / imu_data_num;
    gyr_mean = gyr_mean / imu_data_num;
    use_imu = true;
  }
  mImuData.erase(mImuData.begin(), it);
  mImuDataMutex.unlock();

  // undistortion pointcloud based on IMU rotation integration
  if (use_imu) {
    gyr_mean = Eigen::Quaterniond(mImuInsStaticTrans.topLeftCorner<3, 3>()).normalized().inverse() * gyr_mean;
    Eigen::Matrix4d delta_odom = Eigen::Matrix4d::Identity();
    delta_odom.block<3, 3>(0, 0) = Eigen::Quaterniond(1, gyr_mean[0] * mConfig.scan_period / 2, gyr_mean[1] * mConfig.scan_period / 2, gyr_mean[2] * mConfig.scan_period / 2).normalized().toRotationMatrix();
    undistortPoints(Eigen::Matrix4f::Identity(), delta_odom.cast<float>(), mFrameAttr, mConfig.scan_period);
  }

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

  // combine whole frame
  mFrameAttr->cloud = mFrame; // restore raw pointcloud
  mFrameAttr->T = delta_odom;
  frame = PointCloudAttrImagePose(mFrameAttr, mImages, mImagesStream, odom);

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