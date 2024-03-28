#include "ins_driver_wrapper.h"

#include <stdint.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "Logger.h"
#include "Transform.h"
#include "ins_driver.h"

namespace py=pybind11;

static std::unique_ptr<InsDriver> ins_driver(nullptr);

void create_ins(std::string ins_type, std::string mode) {
  ins_driver.reset(new InsDriver(ins_type, mode));
  LOG_INFO("INS open success");
}

void destory_ins() {
  ins_driver.reset(nullptr);
  LOG_INFO("INS close success");
}

void set_extrinsic_param(double x, double y, double z, double yaw, double pitch, double roll) {
  Transform extrinsic = getTransformFromRPYT(x, y, z, yaw, pitch, roll);
  ins_driver->setExtrinsicParameter(extrinsic);
  LOG_INFO("set INS extrinsic parameter [{}, {}, {}, {}, {}, {}]", x, y, z, yaw, pitch, roll);
}

void start_capture(int port, std::string device) {
  ins_driver->startRun(port, device);
  LOG_INFO("INS start success");
}

void stop_capture() {
  ins_driver->stopRun();
  LOG_INFO("INS stop success");
}

void start_transfer(const std::string dest) {
  ins_driver->startPackageTransfer(dest);
  LOG_INFO("INS start package transfer {}", dest);
}

void stop_transfer() {
  ins_driver->stopPackageTransfer();
  LOG_INFO("INS stop package transfer");
}

void set_offline_data(py::dict &ins_data, py::array_t<double> imu_data) {
  InsDataType data;
  auto imu_ref = imu_data.unchecked<2>();
  for (size_t i = 0; i < (imu_ref.shape(0) - 1); i++) {
    data.gps_timestamp = imu_ref(i, 0);
    data.gyro_x        = imu_ref(i, 1);
    data.gyro_y        = imu_ref(i, 2);
    data.gyro_z        = imu_ref(i, 3);
    data.acc_x         = imu_ref(i, 4);
    data.acc_y         = imu_ref(i, 5);
    data.acc_z         = imu_ref(i, 6);
    data.Status        = ins_data["Status"].cast<int>();
    ins_driver->setData(data, data.gps_timestamp, true);
  }

  data.gps_timestamp = imu_ref(imu_ref.shape(0) - 1, 0);
  data.gps_week      = ins_data["gps_week"].cast<int>();
  data.gps_time      = ins_data["gps_time"].cast<double>();
  data.heading       = ins_data["heading"].cast<double>();
  data.pitch         = ins_data["pitch"].cast<double>();
  data.roll          = ins_data["roll"].cast<double>();
  data.gyro_x        = ins_data["gyro_x"].cast<double>();
  data.gyro_y        = ins_data["gyro_y"].cast<double>();
  data.gyro_z        = ins_data["gyro_z"].cast<double>();
  data.acc_x         = ins_data["acc_x"].cast<double>();
  data.acc_y         = ins_data["acc_y"].cast<double>();
  data.acc_z         = ins_data["acc_z"].cast<double>();
  data.latitude      = ins_data["latitude"].cast<double>();
  data.longitude     = ins_data["longitude"].cast<double>();
  data.altitude      = ins_data["altitude"].cast<double>();
  data.Ve            = ins_data["Ve"].cast<double>();
  data.Vn            = ins_data["Vn"].cast<double>();
  data.Vu            = ins_data["Vu"].cast<double>();
  data.Status        = ins_data["Status"].cast<int>();
  ins_driver->setData(data, data.gps_timestamp, false);
  usleep(10000);
}

py::dict trigger(uint64_t timestamp) {
  bool ins_valid = false;
  bool motion_valid = false;
  std::vector<double> ins_pose = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::vector<double> motion_t = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double motion_heading = 0;
  InsDataType ins;
  std::vector<InsDataType> imu;
  ins_valid = ins_driver->trigger(timestamp, motion_valid, ins_pose, motion_t, motion_heading, ins, imu);

  py::dict data_dict;
  data_dict["ins_valid"]      = ins_valid;
  data_dict["motion_valid"]   = motion_valid;
  data_dict["motion_t"]       = py::array_t<double>(py::array::ShapeContainer({(long) 4, 4}), motion_t.data());
  data_dict["motion_heading"] = motion_heading;

  py::dict ins_data;
  ins_data["timestamp"] = ins.gps_timestamp;
  ins_data["gps_week"]  = ins.gps_week;
  ins_data["gps_time"]  = ins.gps_time;
  ins_data["heading"]   = ins.heading;
  ins_data["pitch"]     = ins.pitch;
  ins_data["roll"]      = ins.roll;
  ins_data["gyro_x"]    = ins.gyro_x;
  ins_data["gyro_y"]    = ins.gyro_y;
  ins_data["gyro_z"]    = ins.gyro_z;
  ins_data["acc_x"]     = ins.acc_x;
  ins_data["acc_y"]     = ins.acc_y;
  ins_data["acc_z"]     = ins.acc_z;
  ins_data["latitude"]  = ins.latitude;
  ins_data["longitude"] = ins.longitude;
  ins_data["altitude"]  = ins.altitude;
  ins_data["Ve"]        = ins.Ve;
  ins_data["Vn"]        = ins.Vn;
  ins_data["Vu"]        = ins.Vu;
  ins_data["Status"]    = ins.Status;
  ins_data["pose"]      = py::array_t<double>(py::array::ShapeContainer({(long) 4, 4}), ins_pose.data());

  std::vector<double> imu_data(imu.size() * 7);
  for (size_t i = 0; i < imu.size(); i++) {
    imu_data[7 * i + 0] = imu[i].gps_timestamp;
    imu_data[7 * i + 1] = imu[i].gyro_x;
    imu_data[7 * i + 2] = imu[i].gyro_y;
    imu_data[7 * i + 3] = imu[i].gyro_z;
    imu_data[7 * i + 4] = imu[i].acc_x;
    imu_data[7 * i + 5] = imu[i].acc_y;
    imu_data[7 * i + 6] = imu[i].acc_z;
  }

  data_dict["ins_data"] = ins_data;
  data_dict["imu_data"] = py::array_t<double>(py::array::ShapeContainer({(long) imu.size(), 7}), imu_data.data());
  return data_dict;
}

PYBIND11_MODULE(ins_driver_ext, m) {
    m.doc() = "ins driver python interface";

    m.def("create_ins", &create_ins, "create ins",
          py::arg("ins_type"), py::arg("mode")
    );

    m.def("destory_ins", &destory_ins, "destory ins");

    m.def("set_extrinsic_param", &set_extrinsic_param, "set extrinsic param",
          py::arg("x"), py::arg("y"), py::arg("z"),
          py::arg("yaw"), py::arg("pitch"), py::arg("roll")
    );

    m.def("start_capture", &start_capture, "start capture",
          py::arg("port"), py::arg("device")
    );

    m.def("stop_capture", &stop_capture, "stop capture");

    m.def("start_transfer", &start_transfer, "start transfer",
          py::arg("dest")
    );

    m.def("stop_transfer", &stop_transfer, "stop transfer");

    m.def("set_offline_data", &set_offline_data, "set offline data",
          py::arg("ins_data"), py::arg("imu_data"));

    m.def("trigger", &trigger, "trigger",
          py::arg("timestamp")
    );
}