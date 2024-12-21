#ifndef __COLMAP_UTILS_H
#define __COLMAP_UTILS_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "slam_base.h"
#include "render_common.h"
#include "camera_model.h"

class ColmapUtil {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ColmapUtil(std::string path);
    virtual ~ColmapUtil();
    void setup();
    void update(std::map<std::string, std::shared_ptr<CameraModel>> &cameras, std::map<std::string, ImageType> &images);
    void update(hash_map_3d<int64_t, Render::VoxelPtr> &map);
    void saveFiles();

  private:
    uint32_t mCameraIdx;
    uint64_t mImageIdx;
    uint64_t mPointIdx;
    PointCloudRGB::Ptr mRGBPoints;
    std::string mPath;
    std::unordered_map<std::string, ColmapCamera> mCamera;
    std::vector<ColmapImage> mImage;
    std::vector<ColmapPoint3d> mPoint3d;
};

#endif // __COLMAP_UTILS_H