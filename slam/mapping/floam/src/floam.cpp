#include "floam.h"

#include <sys/prctl.h>
#include "backend_api.h"

using namespace Mapping;

// FLOAM declarations
void setLaserProcessingNode(int scan_line, double vertical_angle, double scan_period, double max_dis, double min_dis);
void setOdomEstimationNode(int scan_line, double vertical_angle, double scan_period, double max_dis, double min_dis, double map_resolution);
int  initLaserProcessingNode();
int  initOdomEstimationNode();

void getFloamOdom(Eigen::Isometry3d &odom);
void floamEnqueuePoints(PointCloudAttrPtr &points, Eigen::Matrix4d &init_guess);

HDL_FLOAM::HDL_FLOAM() : SlamBase(), mOriginIsSet(false), mFrame(nullptr), mFrameAttr(nullptr) {
  mLidarName = "";
  mSupportLidars["Ouster-OS1-128"] = 128;
  mSupportLidars["Ouster-OS2-128"] = 128;
  mSupportLidars["Ouster-OS1-64"]  = 64;
  mSupportLidars["Ouster-OS1-32"]  = 32;
  mSupportLidars["VLP-16"]         = 16;
  mSupportLidars["LS-C-16"]        = 16;
  mSupportLidars["RS-LiDAR-16"]    = 16;
  mSupportLidars["RS-LiDAR-32"]    = 32;
  mSupportLidars["RS-Ruby-Lite"]   = 80;
  mSupportLidars["RS-Helios-16P"]  = 16;
  mSupportLidars["RS-Helios"]      = 32;
}

HDL_FLOAM::~HDL_FLOAM() {
  mThreadStart = false;
  mFloorThread->join();
  mFloorThread.reset(nullptr);
  mGraphThread->join();
  mGraphThread.reset(nullptr);
  LOG_INFO("FLOAM thread join success");
  deinit_backend();
}

std::vector<std::string> HDL_FLOAM::setSensors(std::vector<std::string> &sensors) {
  mLidarLines = 0;
  std::vector<std::string> sensor_list;

  // check RTK/IMU/Camera is set for use
  for (auto sensor : sensors) {
    if (sensor.compare("RTK") == 0) {
      sensor_list.push_back(sensor);
    } else if (sensor.compare("IMU") == 0) {
      sensor_list.push_back(sensor);
    } else if (sensor.length() < 2 || sensor[1] != '-') { // not a "n-" pattern
      sensor_list.push_back(sensor);
    }
  }

  // use the first Lidar to mapping
  for (auto sensor : sensors) {
    if (sensor.compare("RTK") == 0 || sensor.compare("IMU") == 0 || sensor.length() < 2) {
      continue;
    }
    std::string sensor_name = sensor.substr(2);
    if (mSupportLidars.find(sensor_name) == mSupportLidars.end()) {
      continue;
    }
    mLidarName = sensor;
    mLidarLines = mSupportLidars[sensor_name];
    sensor_list.push_back(sensor);
    LOG_INFO("FLOAM use sensor {} for mapping, lidar line is {}", sensor, mLidarLines);
    break;
  }
  return sensor_list;
}

bool HDL_FLOAM::init(InitParameter &param) {
  mConfig = param;
  mLastOdom.setIdentity();
  // floam init
  setLaserProcessingNode(mLidarLines, 2.0, mConfig.scan_period, 100.0, 0.1);
  setOdomEstimationNode (mLidarLines, 2.0, mConfig.scan_period, 100.0, 0.1, 0.2);
  initLaserProcessingNode();
  initOdomEstimationNode();

  // backend init
  init_backend(param);

  mImuInsStaticTrans = mImuStaticTrans * mStaticTrans.inverse();

  // start backend thread
  mThreadStart = true;
  mFloorThread.reset(new std::thread(&HDL_FLOAM::runFloor, this));
  mGraphThread.reset(new std::thread(&HDL_FLOAM::runGraph, this));
  return true;
}

bool HDL_FLOAM::originIsSet() { return mOriginIsSet; }

RTKType &HDL_FLOAM::getOrigin() { return mOrigin; }

void HDL_FLOAM::setOrigin(RTKType rtk) {
  if (!mOriginIsSet) {
    graph_set_origin(rtk);
    mOriginIsSet = true;
    mOrigin = rtk;
  }
}

void HDL_FLOAM::feedImuData(ImuType &imu) {
  std::lock_guard<std::mutex> lock(mImuDataMutex);
  mImuData.push_back(imu);
}

void HDL_FLOAM::feedInsData(std::shared_ptr<RTKType> ins) {
  enqueue_graph_gps(ins);
}

void HDL_FLOAM::feedImageData(const uint64_t &timestamp,
                              std::map<std::string, ImageType> &images,
                              std::map<std::string, cv::Mat> &images_stream) {
  mImages = images;
  mImagesStream = images_stream;
}

void HDL_FLOAM::feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) {
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
    mInitGuess = delta_odom;
  } else {
    mInitGuess = Eigen::Matrix4d::Identity();
  }

  // enqueue to FLOAM engine and floor detection
  floamEnqueuePoints(mFrameAttr, mInitGuess);
  mFloorQueue.enqueue(new PointCloudType(mFrameAttr->cloud));
}

Eigen::Matrix4d HDL_FLOAM::getPose(PointCloudAttrImagePose &frame) {
  // wait for odometry
  Eigen::Isometry3d odom;
  getFloamOdom(odom);
  Eigen::Matrix4d delta_odom = mLastOdom.inverse() * odom.matrix();
  mLastOdom = odom.matrix();

  // combine whole frame
  mFrameAttr->cloud = mFrame; // restore raw pointcloud
  mFrameAttr->T = delta_odom;
  frame = PointCloudAttrImagePose(mFrameAttr, mImages, mImagesStream, odom);

  return (get_odom2map() * odom).matrix();
}

void HDL_FLOAM::runFloor() {
  prctl(PR_SET_NAME, "FLOAM Floor", 0, 0, 0);
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

void HDL_FLOAM::runGraph() {
  prctl(PR_SET_NAME, "FLOAM Graph", 0, 0, 0);
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