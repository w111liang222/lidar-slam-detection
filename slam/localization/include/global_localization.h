#ifndef __GLOBAL_LOCALIZATION_H
#define __GLOBAL_LOCALIZATION_H

#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include <pcl/registration/registration.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "slam_base.h"
#include "mapping_types.h"

#include "Scancontext/Scancontext.h"
#include "ORBSLAM/include/Map.h"
#include "ORBSLAM/include/Initializer.h"
#include "ORBSLAM/include/Frame.h"
#include "ORBSLAM/include/KeyFrame.h"
#include "ORBSLAM/include/ORBVocabulary.h"
#include "ORBSLAM/include/KeyFrameDatabase.h"
#include "ORBSLAM/include/ORBextractor.h"
#include "ORBSLAM/include/ORBmatcher.h"
#include "ORBSLAM/include/Converter.h"

namespace Locate {

class GlobalLocalization {
 public:
  GlobalLocalization(InitParameter &param, std::string cameraName, CamParamType cameraParam);
  virtual ~GlobalLocalization();
  void clear();
  void feedInsData(std::shared_ptr<RTKType> &ins);
  bool initializePose(PointCloud::Ptr points, const std::pair<std::string, ImageType> &image, Eigen::Isometry3d &pose);

  void setInitPoseRange(PoseRange &r);
  void setInitPose(const Eigen::Matrix4d &t);
  int  getEstimatePose(Eigen::Matrix4d &t);
  void setGraphMap(std::vector<std::shared_ptr<KeyFrame>> &map, std::vector<EdgeType> &connections);

 private:
  void runSearchPose();
  void stopSearchPose();
  std::pair<int, float> localSearch(PointCloud::Ptr &cloud, std::shared_ptr<RTKType> &ins);
  std::pair<int, float> globalSearch(PointCloud::Ptr &cloud);
  void imageSearch(PointCloud::Ptr &cloud, std::pair<std::string, ImageType> &im, std::pair<int, float> &init_match, Eigen::Matrix4f &init_guess);
  bool registrationAlign(const std::string &sensor, std::pair<int, float> &init_match, Eigen::Matrix4f &init_guess, PointCloud::Ptr &cloud, Eigen::Isometry3d &pose);
  void buildGraphKDTree();
  PointCloud::Ptr downsample(PointCloud::Ptr& cloud);

 protected:
  std::atomic<bool> mInitialized;
  PoseRange mRange;
  PointCloud::Ptr mLastFrame;
  std::vector<std::shared_ptr<KeyFrame>> mKeyFrames;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr mGraphKDTree;
  std::vector<EdgeType> mMapConnections;
  std::vector<int> mSearchIdxMap;
  SCManager mScManager;
  std::vector<ORB_SLAM::Frame> mORBFrames;
  std::map<long unsigned int, std::vector<ORB_SLAM::KeyFrame*>> mORBConnections;
  std::unique_ptr<ORB_SLAM::KeyFrameDatabase> mORBKeyFrameDatabase;
  std::unique_ptr<ORB_SLAM::Map> mORBMap;
  std::unique_ptr<ORB_SLAM::ORBextractor> mORBextractor;

  bool mThreadStart;
  std::unique_ptr<std::thread> mSearchThread;
  std::mutex mMutex;
  RWQueue<PointCloudImageType*> mPointImageQueue;
  RWQueue<RTKType*> mInsQueue;
  RWQueue<Eigen::Isometry3d> mPoseQueue;

  pcl::VoxelGrid<Point>::Ptr mDownsampler;
  std::string mImageName;
  CamParamType mCameraParams;
};

}
#endif