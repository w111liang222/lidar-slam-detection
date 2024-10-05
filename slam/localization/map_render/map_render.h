#ifndef __MAP_RENDER_H
#define __MAP_RENDER_H

#include "slam_base.h"
#include "render_common.h"
#include "camera_model.h"
#include "colmap_utils.h"
#include "image_render.hpp"

class MapRender {
  public:
    MapRender();
    virtual ~MapRender();
    void setActive(bool active);
    void setParameter(Eigen::Matrix4d staticTrans, std::map<std::string, CamParamType> cameraParam);
    void updateMap(PointCloud::Ptr cloud, std::unordered_set<Render::VoxelPtr> &voxel_hits);
    void render(Eigen::Matrix4d pose, PointCloudAttrPtr cloud, std::map<std::string, ImageType> images);
    void getColorMap(PointCloudRGB::Ptr& points);

  protected:
    void clear();

  protected:
    bool mActive;
    Eigen::Matrix4d mStaticTrans;
    Eigen::Matrix4d mLastPose;
    ColmapUtil      mColmap;
    std::map<std::string, CamParamType> mCameraParams;
    std::map<std::string, std::shared_ptr<CameraModel>> mCameras;
    std::map<std::string, std::shared_ptr<ImageRender>> mRenders;
    hash_map_3d<int64_t, Render::VoxelPtr> mHashMap;
};

#endif // __MAP_RENDER_H