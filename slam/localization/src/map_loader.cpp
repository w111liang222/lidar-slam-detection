#include "map_loader.h"
#include <omp.h>
#include <fstream>
#include "backend_api.h"

using namespace Locate;

MapLoader::MapLoader() : mGraphKDTree(new pcl::KdTreeFLANN<pcl::PointXYZ>()), mPoseCloud(nullptr) {
  mOriginIsSet = false;
}

MapLoader::~MapLoader() {
  mKeyFramesWhole.clear();
  mKeyFrames.clear();
}

void MapLoader::getGraphEdges(std::vector<EdgeType> &edges) {
  graph_get_edges(edges);
}

bool MapLoader::init(InitParameter &param, bool is_merge) {
    mConfig = param;
    if (!is_merge) {
      init_graph_node(param);
    }

    // load map meta data
    bool is_load_graph = true;
    bool is_load_ok = loadMapOrigin(mConfig.map_path, is_load_graph);
    if (!is_load_ok) {
        return false;
    }

    // get all keyframe files
    auto keyframe_files = getKeyframeFiles(mConfig.map_path, is_load_graph);

    loadGraphMap(keyframe_files, is_load_graph);
    LOG_INFO("Localization: total {} key frames", mKeyFrames.size());
    if (mKeyFrames.size() <= 0) {
        return false;
    }
    mKeyFramesWhole = mKeyFrames;
    if (is_load_graph) {
      bool load_graph_ok = false;
      if (!is_merge) {
        load_graph_ok = graph_load(mConfig.map_path + "/graph", mKeyFramesWhole);
      } else {
        load_graph_ok = graph_merge(mConfig.map_path + "/graph", mKeyFramesWhole);
      }
      if (!load_graph_ok) {
        return false;
      }
    }

    // map resample
    resampleMap(mConfig.key_frame_distance, mConfig.key_frame_degree);

    // map voxel grid filter
    voxelFilterMap(mConfig.key_frame_distance / 2.0);

    // build kdtree for keyframe poses
    buildGraphKDTree();
    return true;
}

bool MapLoader::reloadGraph() {
  init_graph_node(mConfig);
  bool load_graph_ok = graph_load(mConfig.map_path + "/graph", mKeyFramesWhole);
  return load_graph_ok;
}

void MapLoader::mergeMap(MapLoader *new_map) {
  mKeyFramesWhole.insert(mKeyFramesWhole.end(), new_map->mKeyFramesWhole.begin(), new_map->mKeyFramesWhole.end());
  mKeyFrames.insert(mKeyFrames.end(), new_map->mKeyFrames.begin(), new_map->mKeyFrames.end());
  LOG_INFO("Merged Map: total {} key frames", mKeyFramesWhole.size());
}

bool MapLoader::loadMapOrigin(const std::string &map_path, bool &is_load_graph) {
  std::string meta_file;
  {
    meta_file = map_path + "/graph/map_info.txt";
    std::ifstream fs(meta_file, std::ifstream::in);
    if (!fs) {
      is_load_graph = false;
    }
  }

  if (!is_load_graph) {
    meta_file = map_path + "/odometry/map_info.txt";
    std::ifstream fs(meta_file, std::ifstream::in);
    if (!fs) {
      LOG_ERROR("Localization: failed to load map meta data, {}", meta_file);
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

  // all zeros is invalid origin
  if (fabs(mOrigin.latitude) < 1e-4 && fabs(mOrigin.longitude) < 1e-4 && fabs(mOrigin.altitude) < 1e-4 &&
      fabs(mOrigin.heading)  < 1e-4 && fabs(mOrigin.pitch)     < 1e-4 && fabs(mOrigin.roll)     < 1e-4) {
    mOriginIsSet = false;
  } else {
    mOriginIsSet = true;
  }

  LOG_INFO("Localization: load {} map origin: {}, {}, {}, {}, {}, {}",
                (is_load_graph ? "graph" : "odometry"),
                mOrigin.latitude, mOrigin.longitude, mOrigin.altitude,
                mOrigin.heading, mOrigin.pitch, mOrigin.roll);
  return true;
}

std::vector<std::string> MapLoader::getKeyframeFiles(const std::string &map_path, bool is_load_graph) {
  std::vector<std::string> keyframe_files;
  if (is_load_graph) {
    keyframe_files = getDirDirs(map_path + "/graph");
    auto it = keyframe_files.begin();
    while (it != keyframe_files.end()) {
      if (isFileExist(*it + "/data") && isFileExist(*it + "/cloud.pcd")) {
        it++;
      } else {
        it = keyframe_files.erase(it);
      }
    }
  } else {
    keyframe_files = getDirFiles(map_path + "/odometry", ".odom");
    std::for_each(keyframe_files.begin(), keyframe_files.end(), [](std::string & f) {
      f.replace(f.end() - 5, f.end(), "");
    });
  }

  std::sort(keyframe_files.begin(), keyframe_files.end());
  return keyframe_files;
}

void MapLoader::loadGraphMap(const std::vector<std::string> &files, bool is_load_graph) {
  mKeyFrames.clear();
  int keyframe_idx = 0;
  for (size_t i = 0; i < files.size(); i++) {
    std::shared_ptr<KeyFrame> frame(new KeyFrame(keyframe_idx, files[i], is_load_graph));
    bool result = frame->loadOdom();
    if (!result) {
      LOG_ERROR("Localization: failed to load {}", files[i]);
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


void MapLoader::resampleMap(double key_frame_distance, double key_frame_degree) {
  bool is_first = true;
  Eigen::Isometry3d prev_keypose;
  auto it = mKeyFrames.begin();
  // re-calculate the keyframe based on localization settings
  while (it != mKeyFrames.end()) {
    if (is_first) {
      is_first = false;
      prev_keypose = (*it)->mOdom;
      it++;
      continue;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * (*it)->mOdom;
    double dx = delta.translation().norm();
    double da = Eigen::AngleAxisd(delta.linear()).angle() / M_PI * 180;

    // too close to the previous frame
    if(dx < key_frame_distance && da < key_frame_degree) {
      it = mKeyFrames.erase(it);
    } else {
      prev_keypose = (*it)->mOdom;
      it++;
    }
  }

  LOG_INFO("Localization: after resample to {} key frames", mKeyFrames.size());
}

void MapLoader::voxelFilterMap(double voxel_size) {
  std::map<uint64_t, bool> grid;
  auto it = mKeyFrames.begin();
  while (it != mKeyFrames.end()) {
    double grid_x = (*it)->mOdom(0, 3);
    double grid_y = (*it)->mOdom(1, 3);
    double grid_z = (*it)->mOdom(2, 3);
    grid_x = int(grid_x / voxel_size) * voxel_size;
    grid_y = int(grid_y / voxel_size) * voxel_size;
    grid_z = int(grid_z / voxel_size) * voxel_size;
    uint64_t hash_code = hashCoordinate(grid_x, grid_y, grid_z);
    if (grid.find(hash_code) != grid.end()) {
      it = mKeyFrames.erase(it);
    } else {
      grid[hash_code] = true;
      it++;
    }
  }
  LOG_INFO("Localization: after voxel filter to {} key frames", mKeyFrames.size());
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