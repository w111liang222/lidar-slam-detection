#ifndef __UTILS___H
#define __UTILS___H

#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/pytypes.h>
#include <string>
namespace py = pybind11;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <vector>

#include <dirent.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>

#include "Types.h"

template <typename T>
py::list to_list(T t)
{
  return py::cast(t);
}

template <typename T>
std::vector<T> sync_data(uint64_t stamp, std::vector<T>& all_data)
{
  std::vector<T> data;
  auto it = all_data.begin();
  while (it != all_data.end()) {
    if (it->timestamp > stamp) {
      break;
    }
    data.push_back(*it);
    it = all_data.erase(it);
  }

  return data;
}

void write_pickle(std::string filename, py::dict data);
void write_pickle(std::string directory, int idx, py::dict data);
void write_config(std::string filename, py::dict config);

py::dict generate_sensor_config(cv::FileStorage &config);
py::array_t<float> generate_identy_matrix();

void transformPointCloud(PointCloud::Ptr &points, Eigen::Matrix4d T);
py::array_t<float> scan_to_numpy_points(PointCloud::Ptr &p);
py::array_t<float> scan_to_numpy_stamp (PointCloud::Ptr &p);
py::array_t<double> imu_to_numpy(std::vector<Imu_t> imus);
py::dict ins_to_dict(Ins_t ins);

void create_directory(std::string path);
int getGPSweek(const uint64_t &stamp);
double getGPSsecond(const uint64_t &stamp);


#endif //__UTILS___H