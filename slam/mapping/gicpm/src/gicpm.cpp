#include "gicpm.h"

#include <sys/prctl.h>
#include "backend_api.h"

using namespace Mapping;

HDL_GICP::HDL_GICP() : SlamBase(), mOriginIsSet(false), mFrame(nullptr), mFrameAttr(nullptr) {
  mLidarName = "";
}

HDL_GICP::~HDL_GICP() {
  mThreadStart = false;
  mScanMatchThread->join();
  mScanMatchThread.reset(nullptr);
  mFloorThread->join();
  mFloorThread.reset(nullptr);
  mGraphThread->join();
  mGraphThread.reset(nullptr);
  LOG_INFO("GICP thread join success");
  deinit_scan_match_node();
  deinit_backend();
}

bool HDL_GICP::init(InitParameter &param) {
  mConfig = param;
  mLastOdom.setIdentity();
  // backend init
  init_scan_match_node(param);
  init_backend(param);

  mImuInsStaticTrans = mImuStaticTrans * mStaticTrans.inverse();
  mThreadStart = true;
  mScanMatchThread.reset(new std::thread(&HDL_GICP::runScanMatch, this));
  mFloorThread.reset(new std::thread(&HDL_GICP::runFloor, this));
  mGraphThread.reset(new std::thread(&HDL_GICP::runGraph, this));
  return true;
}

bool HDL_GICP::originIsSet() { return mOriginIsSet; }

RTKType &HDL_GICP::getOrigin() { return mOrigin; }

void HDL_GICP::setOrigin(RTKType rtk) {
  if (!mOriginIsSet) {
    graph_set_origin(rtk);
    mOriginIsSet = true;
    mOrigin = rtk;
  }
}

std::vector<std::string> HDL_GICP::setSensors(std::vector<std::string> &sensors) {
  std::vector<std::string> sensor_list;

  bool is_lidar_set = false;
  for (auto sensor : sensors) {
    if (sensor.compare("RTK") == 0) {
      sensor_list.push_back(sensor);
    } else if (sensor.compare("IMU") == 0) {
      sensor_list.push_back(sensor);
    } else if (sensor.length() < 2 || sensor[1] != '-') { // not a "n-" pattern
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

void HDL_GICP::feedImuData(ImuType &imu) {
  std::lock_guard<std::mutex> lock(mImuDataMutex);
  mImuData.push_back(imu);
}

void HDL_GICP::feedInsData(std::shared_ptr<RTKType> ins) {
  enqueue_graph_gps(ins);
}

void HDL_GICP::feedImageData(const uint64_t &timestamp,
                             std::map<std::string, ImageType> &images,
                             std::map<std::string, cv::Mat> &images_stream) {
  mImages = images;
  mImagesStream = images_stream;
}

void HDL_GICP::feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) {
  // transform to INS coordinate
  mFrameAttr = mergePoints(mLidarName, points, uint64_t(mConfig.scan_period * 1000000.0));
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
    acc_mean = Eigen::Quaterniond(mImuInsStaticTrans.topLeftCorner<3, 3>()).normalized().inverse() * acc_mean;
    enqueue_graph_imu(mFrame->header.stamp, gyr_mean, acc_mean);
    Eigen::Matrix4d delta_odom = Eigen::Matrix4d::Identity();
    delta_odom.block<3, 3>(0, 0) = Eigen::Quaterniond(1, gyr_mean[0] * mConfig.scan_period / 2, gyr_mean[1] * mConfig.scan_period / 2, gyr_mean[2] * mConfig.scan_period / 2).normalized().toRotationMatrix();
    undistortPoints(Eigen::Matrix4f::Identity(), delta_odom.cast<float>(), mFrameAttr, mConfig.scan_period);
    mInitGuess = delta_odom;
  } else {
    mInitGuess = Eigen::Matrix4d::Identity();
  }

  // enqueue to GICP engine and backend
  PointCloud::Ptr filtered = enqueue_filter(mFrameAttr->cloud);
  mScanMatchQueue.enqueue(new PointCloudType(filtered, mInitGuess));
  mFloorQueue.enqueue(new PointCloudType(filtered));
}

Eigen::Matrix4d HDL_GICP::getPose(PointCloudAttrImagePose &frame) {
  // wait for odometry
  Eigen::Isometry3d odom;
  mOdomQueue.wait_dequeue(odom);
  Eigen::Matrix4d delta_odom = mLastOdom.inverse() * odom.matrix();
  mLastOdom = odom.matrix();

  // combine whole frame
  mFrameAttr->cloud = mFrame; // restore raw pointcloud
  mFrameAttr->T = delta_odom;
  frame = PointCloudAttrImagePose(mFrameAttr, mImages, mImagesStream, odom);

  return (get_odom2map() * odom).matrix();
}

void HDL_GICP::runScanMatch() {
  prctl(PR_SET_NAME, "GICP Odom", 0, 0, 0);
  while (mThreadStart) {
    PointCloudType *points = nullptr;
    if (false == mScanMatchQueue.wait_dequeue_timed(points, 100000)) {
      continue;
    }
    std::shared_ptr<PointCloudType> pointPtr = std::shared_ptr<PointCloudType>(points);

    Eigen::Matrix4d odom = enqueue_scan_match(pointPtr->points, pointPtr->T);
    mOdomQueue.enqueue(Eigen::Isometry3d(odom));
  }
}

void HDL_GICP::runFloor() {
  prctl(PR_SET_NAME, "GICP Floor", 0, 0, 0);
  while (mThreadStart) {
    PointCloudType *points = nullptr;
    if (false == mFloorQueue.wait_dequeue_timed(points, 100000)) {
      continue;
    }
    std::shared_ptr<PointCloudType> pointPtr = std::shared_ptr<PointCloudType>(points);

    FloorCoeffs floor = enqueue_floor(pointPtr->points);
    enqueue_graph_floor(floor);
  }
}

void HDL_GICP::runGraph() {
  prctl(PR_SET_NAME, "GICP Graph", 0, 0, 0);
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