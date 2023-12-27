#ifndef __MAP_LOADER_H
#define __MAP_LOADER_H

#include <memory>
#include <pcl/kdtree/kdtree_flann.h>

#include "slam_base.h"
#include "mapping_types.h"

namespace Locate {

class MapLoader {
 public:
  MapLoader();
  virtual ~MapLoader();
  bool init(InitParameter &param, bool is_merge = false);
  bool reloadGraph();
  bool originIsSet() {
    return mOriginIsSet;
  }
  RTKType& getOrigin() {
    return mOrigin;
  }
  const InitParameter &getInitConfig() {
    return mConfig;
  }
  void getGraphEdges(std::vector<EdgeType> &edges);
  void mergeMap(MapLoader *new_map);

 protected:
  bool loadMapOrigin(const std::string &map_path, bool &is_load_graph);
  std::vector<std::string> getKeyframeFiles(const std::string &map_path, bool is_load_graph);
  void loadGraphMap(const std::vector<std::string> &files, bool is_load_graph);
  void resampleMap(double key_frame_distance, double key_frame_degree);
  void voxelFilterMap(double voxel_size);
  void buildGraphKDTree();

 protected:
  bool mOriginIsSet;
  RTKType mOrigin;
  InitParameter mConfig;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mPoseCloud;

 public:
  std::vector<std::shared_ptr<KeyFrame>> mKeyFrames;
  std::vector<std::shared_ptr<KeyFrame>> mKeyFramesWhole;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr mGraphKDTree;
};

}
#endif