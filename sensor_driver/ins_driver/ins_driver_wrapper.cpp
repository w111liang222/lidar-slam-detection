#include "ins_driver_wrapper.h"

#include <stdint.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "Logger.h"
#include "Transform.h"
#include "ins_driver.h"

namespace py=pybind11;

static std::unique_ptr<InsDriver> ins(nullptr);

void create_ins(std::string ins_type) {
  // new a ins driver
  ins.reset(new InsDriver(ins_type));
  LOG_INFO("ins open success");
}

void destory_ins() {
  ins.reset(nullptr);
  LOG_INFO("ins close success");
}

void set_external_param(double x, double y, double z, double yaw, double pitch, double roll) {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return;
  }
  Transform externTrans = getTransformFromRPYT(x, y, z, yaw, pitch, roll);
  ins->setExternalParameter(externTrans);
  LOG_INFO("set ins external parameter [{}, {}, {}, {}, {}, {}]", x, y, z, yaw, pitch, roll);
}

void start_capture(int port, std::string device) {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return;
  }
  ins->startRun(port, device);
  LOG_INFO("ins start success");
}

void stop_capture() {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return;
  }
  ins->stopRun();
  LOG_INFO("ins stop success");
}

void start_transfer(const std::string dest) {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return;
  }
  ins->startPackageTransfer(dest);
  LOG_INFO("ins start package transfer {}", dest);
}

void stop_transfer() {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return;
  }
  ins->stopPackageTransfer();
  LOG_INFO("ins stop package transfer");
}

uint64_t get_valid_message_count() {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return 0;
  }
  return ins->getValidMessageCount();
}

uint64_t get_receive_message_count() {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return 0;
  }
  return ins->getReceiveMessageCount();
}

void set_offline_mode() {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return;
  }
  ins->setOfflineMode();
}

void set_offline_data(py::dict &data_dict) {
  if (ins == nullptr) {
    LOG_ERROR("INS is not opened");
    return;
  }
  InsDataType data;
  data.gps_week = data_dict["gps_week"].cast<int>();
  data.gps_time = data_dict["gps_time"].cast<double>();
  data.gps_timestamp = data_dict["timestamp"].cast<uint64_t>();
  data.heading = data_dict["heading"].cast<double>();
  data.pitch = data_dict["pitch"].cast<double>();
  data.roll = data_dict["roll"].cast<double>();
  data.gyro_x = data_dict["gyro_x"].cast<double>();
  data.gyro_y = data_dict["gyro_y"].cast<double>();
  data.gyro_z = data_dict["gyro_z"].cast<double>();
  data.acc_x = data_dict["acc_x"].cast<double>();
  data.acc_y = data_dict["acc_y"].cast<double>();
  data.acc_z = data_dict["acc_z"].cast<double>();
  data.latitude = data_dict["latitude"].cast<double>();
  data.longitude = data_dict["longitude"].cast<double>();
  data.altitude = data_dict["altitude"].cast<double>();
  data.Ve = data_dict["Ve"].cast<double>();
  data.Vn = data_dict["Vn"].cast<double>();
  data.Vu = data_dict["Vu"].cast<double>();
  data.Status = data_dict["Status"].cast<int>();
  ins->setData(data, data.gps_timestamp);
}

py::dict trigger(uint64_t timestamp) {
  std::vector<double> motionT = {1, 0, 0, 0, // T
                                 0, 1, 0, 0,
                                 0, 0, 1, 0,
                                 0, 0, 0, 1};
  double motionR = 0; // Yaw
  InsDataType gps;
  std::vector<InsDataType> imu;
  bool ins_valid = false;
  bool motion_valid = false;

  if (ins != nullptr) {
    ins_valid = ins->trigger(timestamp, motion_valid, motionT, motionR, gps, imu);
  } else {
    LOG_ERROR("INS is not opened");
  }

  py::dict data_dict;
  data_dict["motion_t"] = py::array_t<double>(py::array::ShapeContainer({(long) 4, 4}), motionT.data());
  data_dict["motion_heading"] = motionR;
  data_dict["ins_valid"] = ins_valid;
  data_dict["motion_valid"] = motion_valid;

  if (ins_valid) {
    py::dict ins_data;
    ins_data["timestamp"] = gps.gps_timestamp;
    ins_data["gps_week"] = gps.gps_week;
    ins_data["gps_time"] = gps.gps_time;
    ins_data["heading"] = gps.heading;
    ins_data["pitch"] = gps.pitch;
    ins_data["roll"] = gps.roll;
    ins_data["gyro_x"] = gps.gyro_x;
    ins_data["gyro_y"] = gps.gyro_y;
    ins_data["gyro_z"] = gps.gyro_z;
    ins_data["acc_x"] = gps.acc_x;
    ins_data["acc_y"] = gps.acc_y;
    ins_data["acc_z"] = gps.acc_z;
    ins_data["latitude"] = gps.latitude;
    ins_data["longitude"] = gps.longitude;
    ins_data["altitude"] = gps.altitude;
    ins_data["Ve"] = gps.Ve;
    ins_data["Vn"] = gps.Vn;
    ins_data["Vu"] = gps.Vu;
    ins_data["Status"] = gps.Status;

    py::list imu_data;
    for (auto &data : imu) {
      py::list l;
      l.append(data.gps_timestamp);
      l.append(data.gyro_x);
      l.append(data.gyro_y);
      l.append(data.gyro_z);
      l.append(data.acc_x);
      l.append(data.acc_y);
      l.append(data.acc_z);
      imu_data.append(l);
    }

    data_dict["ins_data"] = ins_data;
    data_dict["imu_data"] = imu_data;
  }

  return data_dict;
}

PYBIND11_MODULE(ins_driver_ext, m) {
    m.doc() = "ins driver python interface";

    m.def("create_ins", &create_ins, "create ins",
          py::arg("ins_type")
    );

    m.def("set_external_param", &set_external_param, "set external param",
          py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("yaw"), py::arg("pitch"), py::arg("roll")
    );

    m.def("destory_ins", &destory_ins, "destory ins");

    m.def("start_capture", &start_capture, "start capture",
          py::arg("port"), py::arg("device")
    );

    m.def("stop_capture", &stop_capture, "stop capture");

    m.def("start_transfer", &start_transfer, "start transfer",
          py::arg("dest")
    );

    m.def("stop_transfer", &stop_transfer, "stop transfer");

    m.def("get_valid_message_count", &get_valid_message_count, "successful parsed message count");

    m.def("get_receive_message_count", &get_receive_message_count, "received message count");

    m.def("set_offline_mode", &set_offline_mode, "set offline mode");

    m.def("set_offline_data", &set_offline_data, "set offline data",
          py::arg("data_dict"));

    m.def("trigger", &trigger, "trigger",
          py::arg("timestamp")
    );

}