#include "localization.h"

#include <sys/prctl.h>
#include <pcl/filters/voxel_grid.h>
#include "hdl_localization/hdl_localization.h"
#include "backend_api.h"

using namespace Locate;

std::unique_ptr<MapLoader> Localization::mMap(new MapLoader());

void Localization::releaseStaticResources() {
  LOG_INFO("Localization: release static resources");
  mMap.reset(new MapLoader());
  deinit_graph_node();
}

void Localization::mergeMap(const std::string &directory, std::vector<std::shared_ptr<KeyFrame>> &frames) {
  std::unique_ptr<MapLoader> newMap(new MapLoader());
  InitParameter param = mConfig;
  param.map_path = directory;
  newMap->init(param, true);
  frames = newMap->mKeyFramesWhole;
  mMap->mergeMap(newMap.get());
  newMap.reset(nullptr);
}

Localization::Localization() : SlamBase(), mFrameAttr(nullptr), mGraphKDTree(nullptr) {
  mOriginIsSet = false;
  mFailureLocalizeCount = 0;
  mInitialized = false;
  mImageName = "";
  mLidarName = "";
  mImage = cv::Mat();
  mLastOdom.setIdentity();

  mLocalMapUpdateThread = nullptr;
  mProjector.reset(new UTMProjector());
}

Localization::~Localization() {
  mThreadStart = false;
  if (mLocalMapUpdateThread != nullptr) {
    mLocalMapUpdateThread->join();
    mLocalMapUpdateThread.reset(nullptr);
  }
  mGlobalLocator.reset(nullptr);
  mLocalizer.reset(nullptr);
  mKeyFrames.clear();
  mLocalMap = nullptr;
}

void Localization::initLocalizer(uint64_t stamp, const Eigen::Matrix4d& pose) {
  std::lock_guard<std::mutex> lock(mMutex);
  std::lock_guard<std::mutex> localizer_lock(mLocalizerMutex);
  mLocalMap = nullptr;
  mLocalizer.reset(new HdlLocalization(mImageName));
  mLocalizer->setStaticTransform(mStaticTrans);
  mLocalizer->setImuStaticTransform(mImuStaticTrans);
  mLocalizer->setCameraParams(mCameraParams);
  mLocalizer->init(mConfig);
  mLocalizer->setInitPose(stamp, pose);
  LOG_INFO("Localization: localizer is initialized success");
}

bool Localization::init(InitParameter &param) {
  mConfig = param;
  CamParamType cameraParam = CamParamType();
  if (mCameraParams.find(mImageName) != mCameraParams.end()) {
    cameraParam = mCameraParams[mImageName];
    cameraParam.staticTrans = cameraParam.staticTrans * mStaticTrans.inverse(); // localization are processed in INS coordinate system
  } else {
    mImageName = "";
    LOG_WARN("Localization: no parameter found for camera: {}", mImageName);
  }
  mGlobalLocator.reset(new GlobalLocalization(mConfig, mImageName, cameraParam));

  // load map meta data
  auto clock = std::chrono::steady_clock::now();

  bool result = false;
  if (mMap->getInitConfig().map_path.compare(mConfig.map_path) != 0) {
    mMap.reset(new MapLoader());
    result = mMap->init(param);
  } else {
    result = mMap->reloadGraph();
  }
  if (!result) {
    return false;
  }

  mOriginIsSet = mMap->originIsSet();
  mOrigin = mMap->getOrigin();
  if (mOriginIsSet) {
    mProjector->FromGlobalToLocal(mOrigin.latitude, mOrigin.longitude, mZeroUtm(0), mZeroUtm(1));
    mZeroUtm(2) = mOrigin.altitude;
  }
  // set data for sub modules
  mKeyFrames = mMap->mKeyFrames;
  mGraphKDTree = mMap->mGraphKDTree;

  std::vector<EdgeType> connections;
  mMap->getGraphEdges(connections);
  mGlobalLocator->setGraphMap(mMap->mKeyFrames, mMap->mGraphKDTree, connections);

  // start map update loop
  startMapUpdateThread();

  auto elapseMs = since(clock).count();
  LOG_INFO("Localization: cost {} ms for loading map", elapseMs);
  return true;
}

bool Localization::isInited() {
  return mInitialized;
}

std::vector<std::string> Localization::setSensors(std::vector<std::string> &sensors) {
  std::vector<std::string> sensor_list;

  // check Camera is set for use
  bool is_camera_set = false;
  bool is_lidar_set = false;
  for (auto sensor : sensors) {
    if (sensor.compare("RTK") == 0 || sensor.compare("IMU") == 0) {
      sensor_list.push_back(sensor);
    }  else if (sensor.length() < 2 || sensor[1] != '-') { // not a "n-" pattern -> CameraName
      if (!is_camera_set) {
        is_camera_set = true;
        mImageName = sensor;
        sensor_list.push_back(sensor);
        LOG_INFO("Localization: use Camera: {} to localize", sensor);
      }
    } else {
      if (!is_lidar_set) {
        is_lidar_set = true;
        mLidarName = sensor;
      }
      sensor_list.push_back(sensor);
    }
  }

  return sensor_list;
}

void Localization::setInitPoseRange(PoseRange &r) {
  std::lock_guard<std::mutex> lock(mMutex);
  mInitialized = false;
  mGlobalLocator->setInitPoseRange(r);
}

void Localization::setInitPose(const Eigen::Matrix4d &t) {
  std::lock_guard<std::mutex> lock(mMutex);
  mInitialized = false;
  mGlobalLocator->setInitPose(t);
}

int Localization::getEstimatePose(Eigen::Matrix4d &t) {
  std::lock_guard<std::mutex> lock(mMutex);
  mInitialized = false;
  return mGlobalLocator->getEstimatePose(t);
}

void Localization::feedInsData(std::shared_ptr<RTKType> ins) {
  if (!mOriginIsSet) {
    return;
  }
  ins->T = computeRTKTransform(*mProjector, ins, mZeroUtm);
  if (!mInitialized) {
    mGlobalLocator->feedInsData(ins);
  } else {
    mLocalizer->feedInsData(ins);
  }
};

void Localization::feedImuData(ImuType &imu) {
  if (!mInitialized) {
    return;
  }
  mLocalizer->feedImuData(imu);
};

void Localization::feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) {
  // transform to INS coordinate
  mFrameAttr = points[mLidarName];
  preprocessPoints(mFrameAttr->cloud, mFrameAttr->cloud);

  if (!mInitialized) {
    mInitialized = mGlobalLocator->initializePose(mFrameAttr->cloud, std::make_pair(mImageName, mImage), mLastOdom);
    if (!mInitialized) {
      return; // pose is not initialized, return
    } else {
      mFailureLocalizeCount = 0;
      initLocalizer(mFrameAttr->cloud->header.stamp, mLastOdom.matrix());
      mPoseQueue.enqueue(mLastOdom);
    }
  }

  // local localization
  auto result = mLocalizer->localize(mFrameAttr, mImage, mLastOdom);
  mPoseQueue.enqueue(mLastOdom); // push located pose and check if local map is needed to update

  if (result == LocType::OK) {
    mFailureLocalizeCount = 0;
  } else if (result == LocType::ERROR) {
    mFailureLocalizeCount++;
    if (mFailureLocalizeCount >= 5) {
      LOG_ERROR("Localization: failed to localize, fallback to initializing");
      PoseRange r(-10000, 10000, -10000, 10000);
      setInitPoseRange(r);
    }
  }
}

void Localization::feedImageData(const uint64_t &timestamp,
                                 std::map<std::string, cv::Mat> &images,
                                 std::map<std::string, cv::Mat> &images_stream) {
  if (images.find(mImageName) != images.end()) {
    mImage = images[mImageName];
  } else {
    mImage = cv::Mat();
  }
}

Eigen::Matrix4d Localization::getPose(PointCloudAttrImagePose &frame) {
  frame = PointCloudAttrImagePose(mFrameAttr);
  return mLastOdom.matrix();
}

bool Localization::getTimedPose(uint64_t timestamp, Eigen::Matrix4d &pose) {
  std::lock_guard<std::mutex> lock(mLocalizerMutex);
  if (mInitialized && mLocalizer != nullptr) {
    return mLocalizer->getTimedPose(timestamp, pose);
  } else {
    return false;
  }
}

bool Localization::getTimedPose(RTKType &ins, Eigen::Matrix4d &pose) {
  std::lock_guard<std::mutex> lock(mLocalizerMutex);
  if (mInitialized && mLocalizer != nullptr) {
    return mLocalizer->getTimedPose(ins, pose);
  } else {
    return false;
  }
}

void Localization::getGraphMap(std::vector<std::shared_ptr<KeyFrame>> &frames) {
  frames = mMap->mKeyFramesWhole;
}

void Localization::getColorMap(PointCloudRGB::Ptr &points) {
  std::lock_guard<std::mutex> lock(mLocalizerMutex);
  if (mInitialized && mLocalizer != nullptr) {
    mLocalizer->getColorMap(points);
  }
}

void Localization::startMapUpdateThread() {
  mThreadStart = true;
  mLocalMapUpdateThread.reset(new std::thread(&Localization::runUpdateLocalMap, this));
}

void Localization::runUpdateLocalMap() {
  prctl(PR_SET_NAME, "Local Map", 0, 0, 0);
  const double update_distance_threshold = 10.0; // distance between last updated position
  const double radius_distance_threshold = 30.0; // radius search keyframe distance
  const int    min_local_map_points_num = 1000;
  const int    max_local_map_points_num = 200000;

  pcl::VoxelGrid<Point> voxelgrid;
  double resolution = std::max(mConfig.resolution, 0.1); // < 0.1m will cause the filter index overflow
  voxelgrid.setLeafSize(resolution, resolution, resolution);

  Eigen::Isometry3d lastPose = Eigen::Isometry3d::Identity();
  while (mThreadStart) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (false == mPoseQueue.wait_dequeue_timed(pose, 10000)) {
      continue;
    }
    // flush pose queue
    while (mPoseQueue.try_dequeue(pose)) {
    }

    std::lock_guard<std::mutex> lock(mMutex);
    Eigen::Isometry3d delta = lastPose.inverse() * pose;
    double dx = delta.translation().norm();
    if (mLocalMap == nullptr || dx > update_distance_threshold) {
      pcl::PointXYZ searchPoint;
      searchPoint.x = pose(0, 3);
      searchPoint.y = pose(1, 3);
      searchPoint.z = pose(2, 3);
      std::vector<int> pointIdx;
      std::vector<float> pointDistance;
      if (mGraphKDTree->radiusSearch (searchPoint, radius_distance_threshold, pointIdx, pointDistance) > 0) {
        lastPose = pose;
        mLocalMap = PointCloud::Ptr(new PointCloud());
        for (size_t i = 0; i < pointIdx.size(); i++) {
          *mLocalMap += *(mKeyFrames[pointIdx[i]]->mTransfromPoints);
          if (mLocalMap->points.size() >= max_local_map_points_num) {
            break;
          }
        }
        // downsample the local map
        voxelgrid.setInputCloud(mLocalMap);
        voxelgrid.filter(*mLocalMap);
        if (pointDistance[0] >= 400) { // 20m
          mLocalMap = nullptr;
          LOG_WARN("Localization: nearest keyframe ({} m) is far away from current position", std::sqrt(pointDistance[0]));
        } else if (mLocalMap->points.size() < min_local_map_points_num) {
          LOG_WARN("Localization: local map points num: {} is less than {}", mLocalMap->points.size(), min_local_map_points_num);
        } else {
          LOG_INFO("Localization: local map is updated, contains {} keyframes, total points {}", pointIdx.size(), mLocalMap->points.size());
        }
      } else {
        mLocalMap = nullptr;
        LOG_ERROR("Localization: out-of map (graph kdtree radius search error)");
      }

      // notify localization module of updated local map
      if (mLocalizer) {
        mLocalizer->updateLocalMap(mLocalMap);
      }
      usleep(100000);
    }
  }
}