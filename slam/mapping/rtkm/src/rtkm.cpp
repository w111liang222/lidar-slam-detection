#include "rtkm.h"

#include <sys/prctl.h>
#include "backend_api.h"

using namespace Mapping;

RTKM::RTKM() : SlamBase(), mOriginIsSet(false), mFrameAttr(nullptr) {
  mLidarName = "";
  mProjector.reset(new UTMProjector());
}

RTKM::~RTKM() {
  mThreadStart = false;
  mGraphThread->join();
  mGraphThread.reset(nullptr);

  mTimedInsData.clear();
  deinit_backend();
}

bool RTKM::init(InitParameter &param) {
  mConfig = param;
  mLastOdom.setIdentity();
  // backend init
  init_backend(param);

  mThreadStart = true;
  mGraphThread.reset(new std::thread(&RTKM::runGraph, this));
  return true;
}

bool RTKM::originIsSet() { return mOriginIsSet; }

RTKType &RTKM::getOrigin() { return mOrigin; }

void RTKM::setOrigin(RTKType rtk) {
  if (!mOriginIsSet) {
    mOriginIsSet = true;
    mOrigin = rtk;
    double easting, northing, altitude;
    mProjector->FromGlobalToLocal(rtk.latitude, rtk.longitude, easting, northing);
    altitude = rtk.altitude;
    LOG_INFO("RTKM: origin is set to  \n \
              lat: {}, lon: {}, alt: {}\n \
              x  : {}, y  : {}, z  : {}",
              rtk.latitude, rtk.longitude, rtk.altitude,
              easting, northing, altitude);

    std::shared_ptr<RTKType> rtk_ptr = std::make_shared<RTKType>(rtk);
    computeRTKTransform(mOrigin, rtk_ptr);
    mLastOdom = rtk_ptr->T;
  }
}

std::vector<std::string> RTKM::setSensors(std::vector<std::string> &sensors) {
  std::vector<std::string> sensor_list;

  bool is_rtk_set = false;
  // check RTK is set for use
  for (auto sensor : sensors) {
    if (sensor.compare("RTK") == 0) {
      sensor_list.push_back(sensor);
      is_rtk_set = true;
    } else if (sensor.compare("IMU") == 0) {
      sensor_list.push_back(sensor);
    }
  }

  if (!is_rtk_set) {
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

void RTKM::feedInsData(bool rtk_valid, std::shared_ptr<RTKType> data) {
  if (rtk_valid && mOriginIsSet) {
    computeRTKTransform(mOrigin, data);
    mTimedInsData[data->timestamp] = data;
  }
}

void RTKM::feedImageData(const uint64_t &timestamp,
                         std::map<std::string, ImageType> &images,
                         std::map<std::string, cv::Mat> &images_stream) {
  mImages = images;
  mImagesStream = images_stream;
}

void RTKM::feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) {
  // transform to INS coordinate
  mFrameAttr = mergePoints(mLidarName, points, uint64_t(mConfig.scan_period * 1000000.0));
  preprocessPoints(mFrameAttr->cloud, mFrameAttr->cloud);
}

Eigen::Matrix4d RTKM::getPose(PointCloudAttrImagePose &frame) {
  // wait for odometry
  Eigen::Matrix4d odom = mLastOdom;
  getInterpolatedTransform(mFrameAttr->cloud->header.stamp, odom);
  Eigen::Matrix4d delta_odom = mLastOdom.inverse() * odom.matrix();
  mLastOdom = odom;

  // combine whole frame
  mFrameAttr->T = delta_odom;
  frame = PointCloudAttrImagePose(mFrameAttr, mImages, mImagesStream, std::vector<PoseType>(), Eigen::Isometry3d(odom));

  return odom;
}

void RTKM::computeRTKTransform(const RTKType &origin, std::shared_ptr<RTKType> &data) {
  double originX, originY;
  double dataX, dataY, dataZ;
  mProjector->FromGlobalToLocal(origin.latitude, origin.longitude, originX, originY);
  mProjector->FromGlobalToLocal(data->latitude, data->longitude, dataX, dataY);
  dataX -= originX;
  dataY -= originY;
  dataZ = data->altitude - origin.altitude;

  double heading = data->heading - get_grid_convergence(mProjector->GetLongitude0(), data->latitude, data->longitude);
  double dataYaw = -heading;
  double dataPitch = data->pitch;
  double dataRoll = data->roll;
  data->T = getTransformFromRPYT(dataX, dataY, dataZ, dataYaw, dataPitch, dataRoll);
}

bool RTKM::getInterpolatedTransform(uint64_t timestamp, Eigen::Matrix4d &trans) {
  if (!mOriginIsSet) {
    LOG_WARN("RTKM: please set the coordinate origin first");
    return false;
  }
  if (mTimedInsData.find(timestamp) != mTimedInsData.end()) {
    Eigen::Matrix4d t = mTimedInsData[timestamp]->T;
    trans = t;
    return true;
  }

  if (mTimedInsData.size() <= 1) {
    LOG_WARN("RTKM: need at least two data for interpolation");
    return false;
  } else if (timestamp < mTimedInsData.begin()->second->timestamp) {
    LOG_WARN("RTKM: require ins timestamp is too small {}", timestamp);
    return false;
  } else if (timestamp > mTimedInsData.rbegin()->second->timestamp) {
    LOG_WARN("RTKM: lidar {}, ins {}", timestamp, mTimedInsData.rbegin()->second->timestamp);
    LOG_WARN("RTKM: require ins timestamp is too large {}", timestamp);
    return false;
  }

  // reverse search for the closet ins data
  auto it = mTimedInsData.rbegin();
  while (it != mTimedInsData.rend()) {
    if (it->first <= timestamp) {
      break;
    }
    it++;
  }
  RTKType &pre = *(it->second.get());
  it--;
  RTKType &next = *(it->second.get());
  LOG_DEBUG("RTKM: query timestamp %llu, within (%llu - %llu)", pre.timestamp, next.timestamp);
  // interpolate
  double t_diff_ratio = static_cast<double>(timestamp - pre.timestamp) /
                        static_cast<double>(next.timestamp - pre.timestamp);

  Eigen::Matrix3d rotation_matrix = pre.T.block<3, 3>(0, 0);
  Eigen::Quaterniond rotation(rotation_matrix);
  Eigen::Quaterniond rotation_inverted(rotation.w(), -rotation.x(), -rotation.y(), -rotation.z());
  Eigen::Vector3d translation = pre.T.block<3, 1>(0, 3);
  Eigen::Vector3d trans_inverted = -(rotation_inverted * translation);

  Eigen::Quaterniond next_rotation(next.T.block<3, 3>(0, 0));
  Eigen::Vector3d next_translation = next.T.block<3, 1>(0, 3);

  Eigen::Vector3d delta_translation = trans_inverted + rotation_inverted * next_translation;
  Eigen::Quaterniond delta_rotation = rotation_inverted * next_rotation;
  Eigen::AngleAxisd angle_axis(delta_rotation);
  Eigen::Matrix<double, 6, 1> log_vector = t_diff_ratio * (Eigen::Matrix<double, 6, 1>() << delta_translation, angle_axis.angle() * angle_axis.axis()).finished();

  constexpr double kEpsilon = 1e-8;
  const double norm = log_vector.tail<3>().norm();
  if (norm < kEpsilon) {
    Eigen::Vector3d tmp_translation = log_vector.head<3>();
    Eigen::Quaterniond tmp_rotation(Eigen::Quaterniond::Identity());
    Eigen::Vector3d new_translation = translation + rotation * tmp_translation;
    Eigen::Quaterniond new_rotation = rotation * tmp_rotation;
    trans.block<3, 1>(0, 3) = new_translation;
    trans.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
  } else {
    Eigen::Vector3d tmp_translation = log_vector.head<3>();
    Eigen::Quaterniond tmp_rotation(Eigen::AngleAxisd(norm,  log_vector.tail<3>() / norm));
    Eigen::Vector3d new_translation = translation + rotation * tmp_translation;
    Eigen::Quaterniond new_rotation = rotation * tmp_rotation;
    trans.block<3, 1>(0, 3) = new_translation;
    trans.block<3, 3>(0, 0) = new_rotation.toRotationMatrix();
  }

  return true;
}

void RTKM::runGraph() {
  prctl(PR_SET_NAME, "RTKM Graph", 0, 0, 0);
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