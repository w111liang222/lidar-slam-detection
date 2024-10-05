#ifndef __PY_UTILS_H
#define __PY_UTILS_H

#include "slam.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pcl/io/pcd_io.h>

namespace py=pybind11;

py::array_t<float> eigen_to_numpy(const Eigen::Matrix4d &e);
py::array_t<float> eigen_to_numpy(const PointCloud::Ptr &p);
py::array_t<float> pointcloud_to_numpy(const PointCloudRGB::Ptr &p);
py::array_t<char>  vector_to_numpy(const std::vector<char> &b);

std::vector<std::string> list_to_vector(py::list &l);
py::dict vector_to_pydict(const std::vector<EdgeType> &edges);
py::dict map_to_pydict(const std::map<int, Eigen::Matrix4d> &m);
py::dict map_to_pydict(const std::map<std::string, cv::Mat> &m);
py::dict map_to_pydict(const std::map<std::string, ImageType> &m);

Eigen::Matrix4d numpy_to_eigen(const py::array_t<float> &array);
Eigen::Matrix4d numpy_to_eigen(const py::array_t<double> &array);
PointCloud::Ptr numpy_to_pointcloud(py::array_t<float> &array, float multiply = 1.0);
PointCloudRGB::Ptr numpy_to_pointcloud_rgb(py::array_t<float> &array);
void numpy_to_pointcloud(const py::array_t<float> &points, const py::array_t<float> &points_attr, PointCloudAttrPtr &cloud);

void pydict_to_cloud(py::dict& points, py::dict& points_attr, std::map<std::string, PointCloudAttrPtr> &clouds);
void pydict_to_mat(py::dict& image_dict, std::map<std::string, cv::Mat> &images);
void pydict_to_image(py::dict& image_dict, py::dict& image_param, std::map<std::string, ImageType> &images);
void pydict_to_rtk(py::dict& rtk_dict, RTKType &rtk);
void numpy_to_imu(py::array_t<double> &imu_list, std::vector<ImuType> &imu);
void numpy_to_odometry(py::array_t<double> &poses, std::vector<PoseType> &odometrys);

py::dict keyframe_to_pydict(std::vector<std::shared_ptr<KeyFrame>> &frames);
std::map<int, InsConfig> pydict_to_ins_config(py::dict& dict);
int pydict_to_ins_dimension(py::dict& dict);
std::map<std::string, CamParamType> pylist_to_camera_config(py::list& cameras);

#endif //__PY_UTILS_H