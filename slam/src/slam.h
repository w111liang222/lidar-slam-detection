#ifndef __mapping_H
#define __mapping_H

#include <string>
#include <vector>
#include <thread>
#include <boost/format.hpp>
#include <pcl/filters/radius_outlier_removal.h>

#include "slam_base.h"
#include "UTMProjector.h"
#include "UnixSocket.h"

struct InsConfig {
  std::string status_name;
  int         status;
  int         priority;
  double      stable_time;
  double      precision;
};

class SLAM {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum runType {
    Online,
    Offline,
  };
  enum modeType {
    None,
    RTKM,
    FastLIO,
    Localization,
  };
  SLAM(enum runType run, enum modeType modeIn);
  virtual ~SLAM();
  static runType getRunModeType(std::string);
  static modeType getMappingTypeByName(std::string);
  static std::string getMappingNameByType(modeType);
  modeType getType() { return mMappingMode; }
  std::vector<std::string> setSensors(std::vector<std::string> sensors);
  void setParams(const std::string& map_path, const double resolution,
                 float dist_threshold, float degree_threshold, float frame_range);
  void setCameraParameter(const std::map<std::string, CamParamType>& camParam);
  void setStaticParameter(const Eigen::Matrix4d& trans);
  void setImuStaticParameter(const Eigen::Matrix4d& trans);
  void setInsConfig(const std::map<int, InsConfig> &config);
  void setInsDimension(int dim);
  void setInitPose(const Eigen::Matrix4d &t);
  int  getEstimatePose(double x0, double y0, double x1, double y1, Eigen::Matrix4d &init_guess);
  void setDestination(bool enable, std::string dest, int port);

 public:
  bool setup();
  bool preprocessInsData(uint64_t timestamp, std::shared_ptr<RTKType> &rtk);
  bool run(const uint64_t &timestamp,
           std::map<std::string, PointCloudAttrPtr>& points,
           std::map<std::string, ImageType> &images,
           std::map<std::string, cv::Mat> &images_stream,
           RTKType &rtk, std::vector<ImuType> &imu, PoseType &pose);
  bool getKeyframe(PointCloudAttrImagePose &keyframe);
  void getOdometrys(std::vector<PoseType> &odometrys);
  void getGraphMap(std::vector<std::shared_ptr<KeyFrame>> &frames);
  void getColorMap(PointCloudRGB::Ptr &points);
  RTKType &getOrigin();
  MapCoordinateType getMapCoordinate();
  void setOrigin(RTKType &rtk);
  void mergeMap(const std::string& directory, std::vector<std::shared_ptr<KeyFrame>> &frames);

  void runMappingThread();
  void runLocalizationThread();

 private:
  enum runType  mRunMode;
  enum modeType mMappingMode;
  int mInsDim;
  std::map<int, InsConfig> mInsConfig;
  bool mUseGPS;
  bool mUseIMU;

  pcl::Filter<Point>::Ptr mOutlierRemoval;
  std::unique_ptr<SlamBase> mSlam;
  std::unique_ptr<UTMProjector> mProjector;
  std::unique_ptr<UnixSocketServer> mUnixServer;
  InitParameter mInitParameter;

  bool mThreadStart;
  std::unique_ptr<std::thread> mMappingThread;
  std::unique_ptr<std::thread> mLocalizationThread;
  bool mOutput;
  std::string mDestinationIP;
  int mDestinationPort;

  int mLastInsPriority;
  double mLastInsTimestamp;
  boost::optional<Eigen::Vector3d> mZeroUtm;

  RWQueue<PointCloudAttrImagePose> mMapQueue;
  RWQueue<PointCloudAttrImagePose> mKeyframeQueue;

  Eigen::Matrix4d mStaticTrans;
  Eigen::Matrix4d mImuStaticTrans;
};

#endif //__mapping_H