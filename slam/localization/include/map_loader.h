#ifndef __MAP_LOADER_H
#define __MAP_LOADER_H

#include <memory>
#include <pcl/kdtree/kdtree_flann.h>

#include "slam_base.h"
#include "mapping_types.h"
#include "map_render.h"
#include "UTMProjector.h"

namespace Locate {

class MapLoader {
 public:
  enum MapType {
    None,
    SLAM,
  };
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
  MapCoordinateType getMapCoordinate() {
    return mCoordinate;
  }
  const InitParameter &getInitConfig() {
    return mConfig;
  }
  void getGraphEdges(std::vector<EdgeType> &edges);
  void mergeMap(MapLoader *new_map);
  void resampleMap();

  // map render
  void setParameter(Eigen::Matrix4d staticTrans, std::map<std::string, CamParamType> cameraParam);
  void setMapRender(bool enable);
  void render(Eigen::Matrix4d pose, PointCloudAttrPtr cloud, std::map<std::string, ImageType> images);
  void getColorMap(PointCloudRGB::Ptr& points);

 protected:
  bool loadSlamMap(bool is_merge);
  void mergeMapSLAM(MapLoader *new_map);

  bool loadMapOrigin(const std::string &map_path);
  void loadOdometry(const std::string &map_path);
  std::vector<std::string> getKeyframeFiles(const std::string &map_path);
  void loadGraphMap(const std::vector<std::string> &files);
  void buildGraphKDTree();
  void OptimizeMap();

 protected:
  Eigen::Vector3d mZeroUtm;
  std::unique_ptr<UTMProjector> mProjector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mPoseCloud;

 public:
  enum MapType mType {MapType::None};
  enum MapCoordinateType mCoordinate {MapCoordinateType::WGS84};
  bool mOriginIsSet;
  RTKType mOrigin;
  InitParameter mConfig;
  std::vector<std::shared_ptr<KeyFrame>> mKeyFrames;
  std::vector<PoseType> mOdometrys;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr mGraphKDTree;
  MapRender mRender;
};

}
#endif