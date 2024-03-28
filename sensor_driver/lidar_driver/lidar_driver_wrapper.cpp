#include "lidar_driver_wrapper.h"

#include <map>

#include <iostream>
#include <stdint.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "Transform.h"
#include "SystemUtils.h"
#include "lidar_driver.h"

namespace py=pybind11;

static std::map<std::string, std::shared_ptr<LIDAR::LidarDriver>> lidars;

void create_lidar(const std::string token_name, const std::string lidar_name) {
  if (lidars.find(token_name) != lidars.end()) {
    LOG_WARN("lidar {}:{} is already opened", token_name, lidar_name);
    return;
  }

  // new a lidar driver
  LIDAR::LidarDriver::lidarType type = LIDAR::LidarDriver::getLidarTypeByName(lidar_name);
  if (type == LIDAR::LidarDriver::lidarType::None) {
    LOG_ERROR("unsupport lidar: {}", lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = std::make_shared<LIDAR::LidarDriver>(LIDAR::LidarDriver::modeType::online, type, std::stoi(token_name));
  lidars[token_name] = lidar;
  LOG_INFO("lidar {}:{} open success", token_name, lidar_name);
}

void set_external_param(const std::string token_name, const std::string lidar_name, double x, double y, double z, double yaw, double pitch, double roll) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  Transform externTrans = getTransformFromRPYT(x, y, z, yaw, pitch, roll);
  lidar->setExternalParameter(externTrans);
  LOG_INFO("set lidar {}:{} external parameter [{}, {}, {}, {}, {}, {}]", token_name, lidar_name, x, y, z, yaw, pitch, roll);
}

void set_range_filter(const std::string token_name, const std::string lidar_name, double x_min, double y_min, double z_min, double x_max, double y_max, double z_max) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  RangeFilter filter = RangeFilter(x_min, x_max, y_min, y_max, z_min, z_max);
  lidar->setRangeFilter(filter);
  LOG_INFO("set lidar {}:{} range filter [{}, {}, {}, {}, {}, {}]", token_name, lidar_name, x_min, y_min, z_min, x_max, y_max, z_max);
}

void set_exclude(const std::string token_name, const std::string lidar_name, double x_min, double y_min, double z_min, double x_max, double y_max, double z_max) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  RangeFilter filter = RangeFilter(x_min, x_max, y_min, y_max, z_min, z_max);
  lidar->setExcludeFilter(filter);
  LOG_INFO("set lidar {}:{} exclude [{}, {}, {}, {}, {}, {}]", token_name, lidar_name, x_min, y_min, z_min, x_max, y_max, z_max);
}

void destory_lidar(const std::string token_name, const std::string lidar_name) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_WARN("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  auto it = lidars.find(token_name);
  lidars.erase(it);
  LOG_INFO("lidar {}:{} close success", token_name, lidar_name);
}

void start_capture(const std::string token_name, const std::string lidar_name, int port) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  lidar->startRun(port);
  LOG_INFO("lidar {}:{} start success", token_name, lidar_name);
}

void stop_capture(const std::string token_name, const std::string lidar_name) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  lidar->stopRun();
  LOG_INFO("lidar {}:{} stop success", token_name, lidar_name);
}

void start_package_transfer(const std::string token_name, const std::string lidar_name, const std::string dest) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  lidar->startPackageTransfer(dest);
  LOG_INFO("lidar {}:{} start package transfer {}", token_name, lidar_name, dest);
}

void stop_package_transfer(const std::string token_name, const std::string lidar_name) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  lidar->stopPackageTransfer();
  LOG_INFO("lidar {}:{} stop package transfer", token_name, lidar_name);
}

std::shared_ptr<LidarScan> get_points_online_impl(const std::string token_name, const std::string lidar_name, int timeout) {
  if (lidars.find(token_name) == lidars.end()) {
    LOG_ERROR("lidar {}:{} is not opened", token_name, lidar_name);
    return nullptr;
  }

  std::shared_ptr<LIDAR::LidarDriver> lidar = lidars[token_name];
  LidarScan *scan = nullptr;
  bool hasData = lidar->scanQueue.wait_dequeue_timed(scan, timeout);
  if (!hasData) {
    LOG_WARN("{}:{} No lidar data comes", token_name, lidar_name);
    return nullptr;
  }
  size_t queueSize = lidar->scanQueue.size_approx();
  if (queueSize >= 10) {
    LOG_WARN("lidar {}:{}'s queue has {} scans, consumer is too slow", token_name, lidar_name, queueSize);
  }
  return std::shared_ptr<LidarScan>(scan);
}

py::dict get_points_online(const std::string token_name, const std::string lidar_name, int timeout) {
  py::gil_scoped_release release;
  std::shared_ptr<LidarScan> scan = get_points_online_impl(token_name, lidar_name, timeout);
  py::gil_scoped_acquire acquire;

  py::dict data;
  if (scan == nullptr) {
    data["timestamp"] = 0;
    data["points"] = py::array_t<float>(py::array::ShapeContainer({0, 4}));
    data["points_attr"] = py::array_t<float>(py::array::ShapeContainer({0, 0}));
  } else {
    auto points = scan->getPointCloudPtr();
    auto points_attr = scan->getPointCloudAttr();
    int  points_attr_dim = scan->getAttrDim();

    data["timestamp"] = scan->getTimeStamp();
    data["points"] = py::array_t<float>(py::array::ShapeContainer({(long) points->size() / 4, 4}), points->data());
    if (points_attr_dim == 0 || points_attr == nullptr) {
      data["points_attr"] = py::array_t<float>(py::array::ShapeContainer({0, 0}));
    } else {
      data["points_attr"] = py::array_t<float>(py::array::ShapeContainer({(long) points_attr->size() / points_attr_dim, points_attr_dim}), points_attr->data());
    }
    LOG_DEBUG("lidar {}:{} output {} points, stamp {}", token_name, lidar_name, points->size() / 4, scan->getTimeStamp());
  }
  return data;
}

PYBIND11_MODULE(lidar_driver_ext, m) {
    m.doc() = "lidar driver python interface";

    m.def("create_lidar", &create_lidar, "create lidar",
          py::arg("token_name"), py::arg("lidar_name")
    );

    m.def("set_external_param", &set_external_param, "set external param",
          py::arg("token_name"), py::arg("lidar_name"),
          py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("yaw"), py::arg("pitch"), py::arg("roll")
    );

    m.def("set_range_filter", &set_range_filter, "set range filter",
          py::arg("token_name"), py::arg("lidar_name"),
          py::arg("x_min"), py::arg("y_min"), py::arg("z_min"),
          py::arg("x_max"), py::arg("y_max"), py::arg("z_max")
    );

    m.def("set_exclude", &set_exclude, "set exclude",
          py::arg("token_name"), py::arg("lidar_name"),
          py::arg("x_min"), py::arg("y_min"), py::arg("z_min"),
          py::arg("x_max"), py::arg("y_max"), py::arg("z_max")
    );

    m.def("destory_lidar", &destory_lidar, "destory lidar",
          py::arg("token_name"), py::arg("lidar_name")
    );

    m.def("start_capture", &start_capture, "start capture",
          py::arg("token_name"), py::arg("lidar_name"), py::arg("port")
    );

    m.def("stop_capture", &stop_capture, "stop capture",
          py::arg("token_name"), py::arg("lidar_name")
    );

    m.def("start_package_transfer", &start_package_transfer, "start package transfer",
          py::arg("token_name"), py::arg("lidar_name"), py::arg("dest")
    );

    m.def("stop_package_transfer", &stop_package_transfer, "stop package transfer",
          py::arg("token_name"), py::arg("lidar_name")
    );

    m.def("get_points_online", &get_points_online, "get points online",
          py::arg("token_name"), py::arg("lidar_name"), py::arg("timeout")
    );
}