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

void Localization::setMapRender(bool enable) {
  mMap->setMapRender(enable);
}

void Localization::renderMap(Eigen::Matrix4d pose, PointCloudAttrPtr cloud, std::map<std::string, ImageType> images) {
  std::map<std::string, ImageType> render_images;
  for (auto &im : images) {
    if (getTimedPose(im.second.stamp, im.second.T)) {
      render_images[im.first] = im.second;
    }
  }
  mMap->render(pose, cloud, render_images);
}

void Localization::mergeMap(const std::string &directory, std::vector<std::shared_ptr<KeyFrame>> &frames) {
  std::unique_ptr<MapLoader> newMap(new MapLoader());
  InitParameter param = mConfig;
  param.map_path = directory;
  if (!newMap->init(param, true)) {
    LOG_ERROR("Map Merge: failed to load map: {}", param.map_path);
    return;
  }
  mMap->mergeMap(newMap.get());
  frames = newMap->mKeyFrames;
}

Localization::Localization() : SlamBase(), mFrameAttr(nullptr), mGraphKDTree(nullptr) {
  mFailureCounter  = 0;
  mLocalizationAge = 0;
  mInitialized = false;
  mImageName = "";
  mLidarName = "";
  mImage = std::map<std::string, ImageType>();
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
  mFailureCounter  = 0;
  mLocalizationAge = 0;

  mLocalMap = nullptr;
  mLocalizer.reset(new HdlLocalization());
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
    LOG_ERROR("Map Loader: error to load map: {}", mConfig.map_path);
    mMap.reset(new MapLoader());
    return false;
  }

  mMap->setParameter(mStaticTrans, mCameraParams);
  if (mMap->originIsSet()) {
    mProjector->FromGlobalToLocal(mMap->getOrigin().latitude, mMap->getOrigin().longitude, mZeroUtm(0), mZeroUtm(1));
    mZeroUtm(2) = mMap->getOrigin().altitude;
  }
  // set data for sub modules
  mKeyFrames = mMap->mKeyFrames;
  mGraphKDTree = mMap->mGraphKDTree;

  std::vector<EdgeType> connections;
  mMap->getGraphEdges(connections);
  mGlobalLocator->setGraphMap(mMap->mKeyFrames, connections);

  // start map update loop
  startMapUpdateThread();

  auto elapseMs = since(clock).count();
  LOG_INFO("Localization: cost {} ms for loading map", elapseMs);
  return true;
}

bool Localization::originIsSet() {
  return mMap->originIsSet();
}

RTKType& Localization::getOrigin() {
  return mMap->getOrigin();
}

MapCoordinateType Localization::getMapCoordinate() {
  return mMap->getMapCoordinate();
}

bool Localization::isInited() {
  return (mInitialized && isStable());
}

bool Localization::isStable() {
  return (mLocalizationAge >= 10);
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
      sensor_list.push_back(sensor);
      if (!is_camera_set) {
        is_camera_set = true;
        mImageName = sensor;
      }
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

void Localization::feedInsData(bool rtk_valid, std::shared_ptr<RTKType> ins) {
  if (!rtk_valid || !originIsSet()) {
    return;
  }
  ins->T = computeRTKTransform(*mProjector, ins, mZeroUtm);
  if (!mInitialized) {
    mGlobalLocator->feedInsData(ins);
  } else {
    mLocalizer->feedInsData(ins);
  }
}

void Localization::feedImuData(ImuType &imu) {
  if (!mInitialized) {
    return;
  }
  mLocalizer->feedImuData(imu);
}

void Localization::feedPointData(const uint64_t &timestamp, std::map<std::string, PointCloudAttrPtr> &points) {
  // transform to INS coordinate
  mFrameAttr = points[mLidarName];
  preprocessPoints(mFrameAttr->cloud, mFrameAttr->cloud);
  ImageType image = (mImage.find(mImageName) != mImage.end()) ? mImage[mImageName] : ImageType();

  if (!mInitialized) {
    mInitialized = mGlobalLocator->initializePose(mFrameAttr->cloud, std::make_pair(mImageName, image), mLastOdom);
    if (!mInitialized) {
      return; // pose is not initialized, return
    } else {
      initLocalizer(mFrameAttr->cloud->header.stamp, mLastOdom.matrix());
      mPoseQueue.enqueue(mLastOdom);
    }
  }

  // local localization
  auto result = mLocalizer->localize(mFrameAttr, image, mLastOdom);

  if (result == LocType::OK) {
    mLocalizationAge++;
    mFailureCounter = 0;

    mPoseQueue.enqueue(mLastOdom); // push located pose and check if local map is needed to update
    if (isStable()) {
      renderMap(mLastOdom.matrix(), mFrameAttr, mImage);
    }
  } else {
    mFailureCounter++;
  }

  // check if submap matching error
  if (mFailureCounter >= 5) {
    LOG_ERROR("Localization: failed to localize, fallback to initializing");
    PoseRange r(-10000, 10000, -10000, 10000);
    setInitPoseRange(r);
  }
}

void Localization::feedImageData(const uint64_t &timestamp,
                                 std::map<std::string, ImageType> &images,
                                 std::map<std::string, cv::Mat> &images_stream) {
  mImage = images;
}

Eigen::Matrix4d Localization::getPose(PointCloudAttrImagePose &frame) {
  frame = PointCloudAttrImagePose(mFrameAttr);
  return mLastOdom.matrix();
}

std::vector<PoseType> Localization::getOdometrys() {
  return mMap->mOdometrys;
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
  frames = mMap->mKeyFrames;
}

void Localization::getColorMap(PointCloudRGB::Ptr& points) {
  mMap->getColorMap(points);
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
      std::vector<int> pointIdx(1);
      std::vector<float> pointDistance(1);
      if (mGraphKDTree->radiusSearch (searchPoint, radius_distance_threshold, pointIdx, pointDistance) > 0) {
        lastPose = pose;
        float accumDistance = 0;
        mLocalMap = PointCloud::Ptr(new PointCloud());
        for (size_t i = 0; i < pointIdx.size(); i++) {
          float distance = std::sqrt(pointDistance[i]);
          if (!mLocalMap->points.empty() && (distance - accumDistance) < mConfig.key_frame_distance) {
            continue;
          }

          accumDistance = distance;
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
        mGraphKDTree->nearestKSearch(searchPoint, 1, pointIdx, pointDistance);
        LOG_ERROR("Localization: out of map, pose ({:.2f}, {:.2f}, {:.2f}), min dist to map {}", pose(0, 3), pose(1, 3), pose(2, 3), std::sqrt(pointDistance[0]));
      }

      // notify localization module of updated local map
      if (mLocalizer) {
        mLocalizer->updateLocalMap(mLocalMap);
      }
      usleep(100000);
    }
  }
}