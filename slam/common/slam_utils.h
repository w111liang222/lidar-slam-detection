#ifndef __SLAM_UTILS_H
#define __SLAM_UTILS_H

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "mapping_types.h"
#include "SystemUtils.h"
#include "UTMProjector.h"

uint64_t hashCoordinate(const double &px, const double &py, const double &pz);

std::vector<std::string> getDirFiles(std::string path, std::string extension);
std::vector<std::string> getDirDirs(std::string path);
bool isFileExist(const std::string& name);

Eigen::Matrix4d getTransformFromRPYT(double x, double y, double z,
                                     double yaw, double pitch, double roll);
void getRPYTfromTransformFrom(Eigen::Matrix4d tran, double &x, double &y, double &z,
                              double &yaw, double &pitch, double &roll);
Eigen::Matrix4d computeRTKTransform(UTMProjector &projector, std::shared_ptr<RTKType> &data, Eigen::Vector3d &zero_utm);
Eigen::Matrix4d interpolateTransform(Eigen::Matrix4d &preT, Eigen::Matrix4d &nextT, double &t_diff_ratio);
void undistortPoints(const Eigen::Matrix4f &delta_pose, PointCloudAttrPtr &points, double scan_period);
void undistortPoints(std::vector<PoseType> &poses, PointCloudAttrPtr &points);
void pointsDistanceFilter(const PointCloud::ConstPtr& cloud, PointCloud::Ptr& filtered, double min_range, double max_range);
PointCloudAttrPtr mergePoints(const std::string &base, std::map<std::string, PointCloudAttrPtr> &points, const uint64_t &scan_period);
void imageCvtColor(cv::Mat &image);

bool parseGPCHC(std::string message, RTKType &ins);
std::string formatGPCHC(RTKType &ins);

#endif  //__SLAM_UTILS_H