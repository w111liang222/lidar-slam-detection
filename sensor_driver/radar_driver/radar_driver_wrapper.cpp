#include "radar_driver_wrapper.h"

#include <map>

#include <iostream>
#include <stdint.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "Logger.h"
#include "radar_driver.h"
#include "Transform.h"

namespace py=pybind11;

static std::map<std::string, std::shared_ptr<RadarDriver>> radars;
static std::map<std::string, uint64_t> radarsTimestamp;

void create_radar(const std::string token_name, const std::string radar_name) {
  if (radars.find(token_name) != radars.end()) {
    LOG_WARN("radar {}:{} is already opened", token_name, radar_name);
    return;
  }

  // new a radar driver
  RadarDriver::radarType type = RadarDriver::getRadarTypeByName(radar_name);
  if (type == RadarDriver::radarType::None) {
    LOG_ERROR("unsupport radar: {}", radar_name);
    return;
  }
  std::shared_ptr<RadarDriver> radar = std::make_shared<RadarDriver>(RadarDriver::modeType::online, type);
  radars[token_name] = radar;
  LOG_INFO("radar {}:{} open success", token_name, radar_name);
}

void set_external_param(const std::string token_name, const std::string radar_name,
    double x, double y, double z, double yaw, double pitch, double roll) {
  std::shared_ptr<RadarDriver> radar = radars[token_name];
  Transform externTrans = getTransformFromRPYT(x, y, z, yaw, pitch, roll);
  radar->setExternalParameter(externTrans);
  LOG_INFO("set radar {}:{} external parameter [{}, {}, {}, {}, {}, {}]", token_name, radar_name, x, y, z, yaw, pitch, roll);
}

void destory_radar(const std::string token_name, const std::string radar_name) {
  if (radars.find(token_name) == radars.end()) {
    LOG_WARN("radar {}:{} is not opened", token_name, radar_name);
    return;
  }
  auto it = radars.find(token_name);
  radars.erase(it);
  LOG_INFO("radar {}:{} close success", token_name, radar_name);
}

void start_capture(const std::string token_name, const std::string radar_name, std::string port) {
  if (radars.find(token_name) == radars.end()) {
    LOG_ERROR("radar {}:{} is not opened", token_name, radar_name);
    return;
  }

  std::shared_ptr<RadarDriver> radar = radars[token_name];
  radar->startRun(port);
  LOG_INFO("radar {}:{} start success", token_name, radar_name);
}

void stop_capture(const std::string token_name, const std::string radar_name) {
  if (radars.find(token_name) == radars.end()) {
    LOG_ERROR("radar {}:{} is not opened", token_name, radar_name);
    return;
  }
  std::shared_ptr<RadarDriver> radar = radars[token_name];
  radar->stopRun();
  LOG_INFO("radar {}:{} stop success", token_name, radar_name);
}

std::vector<float> get_radar_impl(const std::string token_name, const std::string radar_name, int timeout) {
  radarsTimestamp[token_name] = 0;
  if (radars.find(token_name) == radars.end()) {
    LOG_ERROR("radar {}:{} is not opened", token_name, radar_name);
    return std::vector<float>();
  }
  std::shared_ptr<RadarDriver> tmp_radar = radars[token_name];
  RadarFrame *scan = nullptr;
  bool hasData = tmp_radar->scanQueue.wait_dequeue_timed(scan, timeout);
  if (!hasData) {
    LOG_WARN("{}:{} No radar data comes", token_name, radar_name);
    return std::vector<float>();
  }
  size_t queueSize = tmp_radar->scanQueue.size_approx();
  if (queueSize >= 10) {
    LOG_WARN("radar {}:{}'s queue has {} scans, consumer is too slow", token_name, radar_name, queueSize);
  }

  std::vector<float> object_result;
  std::shared_ptr<RadarFrame> scanPtr =  std::shared_ptr<RadarFrame>(scan);
  auto radar_frame = scanPtr->getRadarFrame();
  for (auto r : radar_frame) {
    object_result.emplace_back(r.second.x);
    object_result.emplace_back(r.second.y);
    object_result.emplace_back(r.second.z); // z-axis value of center point
    object_result.emplace_back(r.second.length);
    object_result.emplace_back(r.second.width);
    object_result.emplace_back(1.0); // the height of object box
    object_result.emplace_back(r.second.ang); // the heading of object box

    object_result.emplace_back(float(r.second.id));
    object_result.emplace_back(float(r.second.type));
    object_result.emplace_back(r.second.vx);
    object_result.emplace_back(r.second.vy);
  }
  radarsTimestamp[token_name] = scanPtr->getTimeStamp();
  return object_result;
}

uint64_t get_timestamp(const std::string token_name, const std::string radar_name) {
  if (radars.find(token_name) == radars.end()) {
    LOG_ERROR("radar {}:{} is not opened", token_name, radar_name);
    return 0;
  }
  return radarsTimestamp[token_name];
}

py::array_t<float> get_radar_online(const std::string token_name, const std::string radar_name, int timeout) {
  py::gil_scoped_release release;
  auto radar_frame = get_radar_impl(token_name, radar_name, timeout);
  py::gil_scoped_acquire acquire;
  if (radar_frame.empty())
    return py::array_t<float>(py::array::ShapeContainer({0, 11}));
  else
    return py::array_t<float>(py::array::ShapeContainer({(long) radar_frame.size() / 11, 11}), radar_frame.data());
}

PYBIND11_MODULE(radar_driver_ext, m) {
    m.doc() = "radar driver python interface";

    m.def("create_radar", &create_radar, "create radar",
          py::arg("token_name"), py::arg("radar_name")
    );

    m.def("set_external_param", &set_external_param, "set external param",
          py::arg("token_name"), py::arg("radar_name"),
          py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("yaw"), py::arg("pitch"), py::arg("roll")
    );

    m.def("destory_radar", &destory_radar, "destory radar",
          py::arg("token_name"), py::arg("radar_name")
    );

    m.def("start_capture", &start_capture, "start capture",
          py::arg("token_name"), py::arg("radar_name"), py::arg("port")
    );

    m.def("stop_capture", &stop_capture, "stop capture",
          py::arg("token_name"), py::arg("radar_name")
    );

    m.def("get_timestamp", &get_timestamp, "get timestamp",
          py::arg("token_name"), py::arg("radar_name")
    );

    m.def("get_radar_online", &get_radar_online, "get radar online",
          py::arg("token_name"), py::arg("radar_name"), py::arg("timeout")
    );
}