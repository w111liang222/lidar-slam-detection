#include "map_loader.h"
#include <fstream>
#include <omp.h>

#include <pcl/filters/voxel_grid.h>

#include "overlap_removal.hpp"
#include "overlap_merge.hpp"
#include "overlap_update.hpp"
#include "global_alignment.hpp"
#include "backend_api.h"

using namespace Locate;

MapLoader::MapLoader() : mGraphKDTree(new pcl::KdTreeFLANN<pcl::PointXYZ>()), mPoseCloud(nullptr) {
  mOriginIsSet = false;
  mProjector.reset(new UTMProjector());
}

MapLoader::~MapLoader() {
  mKeyFrames.clear();
}

void MapLoader::getGraphEdges(std::vector<EdgeType> &edges) {
  graph_get_edges(edges);
}

bool MapLoader::loadSlamMap(bool is_merge) {
    // load map meta data
    bool is_load_ok = loadMapOrigin(mConfig.map_path);
    if (!is_load_ok) {
        return false;
    }

    // load odometry
    loadOdometry(mConfig.map_path);

    // get all keyframe files
    auto keyframe_files = getKeyframeFiles(mConfig.map_path);

    loadGraphMap(keyframe_files);
    LOG_INFO("Map: total {} key frames", mKeyFrames.size());
    if (mKeyFrames.size() <= 0) {
        return false;
    }

    if (!is_merge) {
      bool load_graph_ok = graph_load(mConfig.map_path + "/graph", mKeyFrames);
      if (!load_graph_ok) {
        return false;
      }

      // build kdtree for keyframe poses
      buildGraphKDTree();
    }

    mType = MapType::SLAM;
    return true;
}

bool MapLoader::init(InitParameter &param, bool is_merge) {
    mConfig = param;
    if (!is_merge) {
      init_graph_node(param);
    }

    LOG_INFO("Map: start to load map, path: {}", mConfig.map_path);
    if (isFileExist(mConfig.map_path + "/graph/map_info.txt")) {
      return loadSlamMap(is_merge);
    }

    LOG_ERROR("Map: not recognize the valid format, please check the map path");
    return false;
}

bool MapLoader::reloadGraph() {
  init_graph_node(mConfig);
  bool load_graph_ok = graph_load(mConfig.map_path + "/graph", mKeyFrames);
  return load_graph_ok;
}

void MapLoader::mergeMapSLAM(MapLoader *new_map) {
  if (!mOriginIsSet && new_map->mOriginIsSet) {
    LOG_ERROR("Map: can not merge, the reference map has no origin, but source has");
    goto err1;
  }

  if (mCoordinate != new_map->mCoordinate) {
    LOG_ERROR("Map: coordinate system is not consistent, {} / {}", mCoordinate, new_map->mCoordinate);
    goto err1;
  }

  // merge pose graph
  if (!graph_merge(new_map->mConfig.map_path + "/graph", new_map->mKeyFrames)) {
    LOG_ERROR("Map: can not merge the pose graph");
    goto err1;
  }

  if (!mOriginIsSet || !new_map->mOriginIsSet) {
    LOG_INFO("Map: run global search to align coordinate, source: {}, reference: {}", new_map->mOriginIsSet, mOriginIsSet);
    if (!global_alignment(mKeyFrames, new_map->mKeyFrames, mOriginIsSet)) {
      goto err1;
    }
  } else {
    if (std::fabs(mOrigin.latitude  - new_map->mOrigin.latitude)  > 1e-6 ||
        std::fabs(mOrigin.longitude - new_map->mOrigin.longitude) > 1e-6 ||
        std::fabs(mOrigin.altitude  - new_map->mOrigin.altitude)  > 1e-6) {
      LOG_INFO("Map: origin is not consistent, ({}, {}, {}) / ({}, {}, {})", mOrigin.latitude, mOrigin.longitude, mOrigin.altitude,
                                                                             new_map->mOrigin.latitude, new_map->mOrigin.longitude, new_map->mOrigin.altitude);
      graph_update_origin(new_map->mOrigin, mOrigin, new_map->mKeyFrames);
      graph_sync_pose(new_map->mKeyFrames, SyncDirection::FROM_GRAPH);
    }
  }

  // 1. resample single clip map
  // new_map->resampleMap();

  // 2. detect the overlap region based on distance and pointcloud registration
  {
    const int fragment = 10;
    int fragment_num = int(new_map->mKeyFrames.size() / fragment) + 1;

    OverlapDetector overlap_detector;
    for (int i = 0; i < fragment_num; i++) {
      int begin_offset = i * fragment;
      int end_offset = std::min((i + 1) * fragment, int(new_map->mKeyFrames.size()));
      std::vector<std::shared_ptr<KeyFrame>> fragment_frames(new_map->mKeyFrames.begin() + begin_offset, new_map->mKeyFrames.begin() + end_offset);

      std::vector<Overlap::Ptr> overlaps = overlap_detector.detect(mKeyFrames, fragment_frames);
      for (auto &overlap : overlaps) {
        graph_add_edge(overlap->key1->mPoints, overlap->key1->mId, overlap->key2->mPoints, overlap->key2->mId, overlap->relative_pose, overlap->score);
      }
      graph_optimize();
      graph_sync_pose(mKeyFrames, SyncDirection::FROM_GRAPH);
      graph_sync_pose(new_map->mKeyFrames, SyncDirection::FROM_GRAPH);
    }
  }

  // 3. merge keyframes
  mKeyFrames.insert(mKeyFrames.end(), new_map->mKeyFrames.begin(), new_map->mKeyFrames.end());
  mOdometrys.emplace_back(PoseType()); // push a zero pose as the splitter
  mOdometrys.insert(mOdometrys.end(), new_map->mOdometrys.begin(), new_map->mOdometrys.end());

  // 4. optimize
  // OptimizeMap();

  // 5. detect duplicate region and update
  // {
  //   OverlapUpdator overlap_updator;
  //   // std::vector<std::shared_ptr<KeyFrame>> duplicate_regions = overlap_updator.detect(mKeyFrames);
  //   // mKeyFrames = overlap_updator.update(mKeyFrames, duplicate_regions);
  //   // for(size_t i = 0; i < duplicate_regions.size(); i++) {
  //   //   graph_del_vertex(duplicate_regions[i]->mId);
  //   // }
  //   std::vector<Overlap::Ptr> overlaps = overlap_updator.post_process(mKeyFrames);
  //   for (auto &overlap : overlaps) {
  //     graph_add_edge(overlap->key1->mPoints, overlap->key1->mId, overlap->key2->mPoints, overlap->key2->mId, overlap->relative_pose, overlap->score);
  //   }
  // }

  // 6. optimize
  OptimizeMap();
  LOG_INFO("Map: merge success, total {} key frames", mKeyFrames.size());
  return;

err1:
  new_map->mKeyFrames.clear();
  LOG_ERROR("Map: fail to merge map: {}", new_map->mConfig.map_path);
}


void MapLoader::mergeMap(MapLoader *new_map) {
  if (mType == MapType::SLAM && new_map->mType == MapType::SLAM) {
    mergeMapSLAM(new_map);
  } else {
    LOG_ERROR("Map: map type is not support for merging: {} / {}", mType, new_map->mType);
  }
}

bool MapLoader::loadMapOrigin(const std::string &map_path) {
  std::string meta_file;
  {
    meta_file = map_path + "/graph/map_info.txt";
    std::ifstream fs(meta_file, std::ifstream::in);
    if (!fs) {
      LOG_ERROR("Map: failed to load map info data: {}", meta_file);
      return false;
    }
  }

  std::ifstream fs(meta_file, std::ifstream::in);
  fs >> mOrigin.latitude;
  fs >> mOrigin.longitude;
  fs >> mOrigin.altitude;
  fs >> mOrigin.heading;
  fs >> mOrigin.pitch;
  fs >> mOrigin.roll;

  // coordinate type, default: 0 = WGS84
  double coordinate = 0;
  fs >> coordinate;
  mCoordinate = static_cast<MapCoordinateType>(int(coordinate));

  // all zeros is invalid origin
  if (fabs(mOrigin.latitude) < 1e-4 && fabs(mOrigin.longitude) < 1e-4 && fabs(mOrigin.altitude) < 1e-4 &&
      fabs(mOrigin.heading)  < 1e-4 && fabs(mOrigin.pitch)     < 1e-4 && fabs(mOrigin.roll)     < 1e-4) {
    mOriginIsSet = false;
  } else {
    mOriginIsSet = true;
  }

  if (mOriginIsSet) {
    mProjector->FromGlobalToLocal(mOrigin.latitude, mOrigin.longitude, mZeroUtm(0), mZeroUtm(1));
    mZeroUtm(2) = mOrigin.altitude;
  }

  LOG_INFO("Map: load graph map origin: {}, {}, {}, {}, {}, {}",
                mOrigin.latitude, mOrigin.longitude, mOrigin.altitude,
                mOrigin.heading, mOrigin.pitch, mOrigin.roll);
  return true;
}

void MapLoader::loadOdometry(const std::string &map_path) {
  std::string odometry_file = map_path + "/graph/odometrys.txt";
  std::ifstream fs(odometry_file, std::ifstream::in);
  if (!fs) {
    LOG_WARN("Map: failed to load odometry, {}", odometry_file);
    return;
  }

  while(!fs.eof()) {
    double stamp;
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    fs >> stamp;
    fs >> t[0];
    fs >> t[1];
    fs >> t[2];
    fs >> q.x();
    fs >> q.y();
    fs >> q.z();
    fs >> q.w();
    if (fs.eof()) {
      break;
    }

    PoseType odom;
    odom.timestamp = uint64_t(stamp * 1000000);
    odom.T.block<3, 1>(0, 3) = t;
    odom.T.block<3, 3>(0, 0) = q.toRotationMatrix();
    mOdometrys.emplace_back(odom);
  }
}

std::vector<std::string> MapLoader::getKeyframeFiles(const std::string &map_path) {
  std::vector<std::string> keyframe_files = getDirDirs(map_path + "/graph");
  auto it = keyframe_files.begin();
  while (it != keyframe_files.end()) {
    if (isFileExist(*it + "/data") && isFileExist(*it + "/cloud.pcd")) {
      it++;
    } else {
      it = keyframe_files.erase(it);
    }
  }

  std::sort(keyframe_files.begin(), keyframe_files.end());
  return keyframe_files;
}

void MapLoader::loadGraphMap(const std::vector<std::string> &files) {
  mKeyFrames.clear();
  int keyframe_idx = 0;
  for (size_t i = 0; i < files.size(); i++) {
    std::shared_ptr<KeyFrame> frame(new KeyFrame(keyframe_idx, files[i], true));
    bool result = frame->loadOdom();
    if (!result) {
      LOG_ERROR("Map: failed to load {}", files[i]);
      mKeyFrames.clear();
      break;
    }
    mKeyFrames.push_back(frame);
    keyframe_idx++;
  }

  std::sort(mKeyFrames.begin(), mKeyFrames.end(),
    [](const std::shared_ptr<KeyFrame>& a, const std::shared_ptr<KeyFrame>& b) -> bool {
      return a->mId < b->mId;
    }
  );

  #pragma omp parallel for num_threads(4)
  for(size_t i = 0; i < mKeyFrames.size(); i++) {
    mKeyFrames[i]->loadMeta();
    mKeyFrames[i]->loadPcd();
    mKeyFrames[i]->loadImage();
    // mKeyFrames[i]->downsample(mConfig.resolution);
    mKeyFrames[i]->transformPoints(); // duplicate the points
    mKeyFrames[i]->computeDescriptor();
  }
}

void MapLoader::resampleMap() {
  // removal based on distance
  OverlapRemover overlap_remover;
  std::vector<std::shared_ptr<KeyFrame>> duplicate_frames = overlap_remover.detect(mKeyFrames);
  mKeyFrames = overlap_remover.remove(mKeyFrames, duplicate_frames);

  // sync to graph
  for(size_t i = 0; i < duplicate_frames.size(); i++) {
    graph_del_vertex(duplicate_frames[i]->mId);
  }
  LOG_INFO("Map: resample map to {} key frames", mKeyFrames.size());
}

void MapLoader::buildGraphKDTree() {
  mPoseCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  mPoseCloud->width = static_cast<int>(mKeyFrames.size());
  mPoseCloud->height = 1;
  mPoseCloud->points.resize(mPoseCloud->width * mPoseCloud->height);
  for (size_t i = 0; i < mKeyFrames.size(); i++) {
    mPoseCloud->points[i].x = mKeyFrames[i]->mOdom(0, 3);
    mPoseCloud->points[i].y = mKeyFrames[i]->mOdom(1, 3);
    mPoseCloud->points[i].z = mKeyFrames[i]->mOdom(2, 3);
  }
  mGraphKDTree->setInputCloud(mPoseCloud);
}

void MapLoader::OptimizeMap() {
  graph_optimize();
  graph_sync_pose(mKeyFrames, SyncDirection::FROM_GRAPH);
}

// Map Render
void MapLoader::setParameter(Eigen::Matrix4d staticTrans, std::map<std::string, CamParamType> cameraParam) {
  mRender.setParameter(staticTrans, cameraParam);
}

void MapLoader::setMapRender(bool enable) {
  mRender.setActive(enable);
}

void MapLoader::render(Eigen::Matrix4d pose, PointCloudAttrPtr cloud, std::map<std::string, ImageType> images) {
  mRender.render(pose, cloud, images);
}

void MapLoader::getColorMap(PointCloudRGB::Ptr& points) {
  mRender.getColorMap(points);
}
