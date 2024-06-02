#include "RosbagReader.h"
#include "Utils.h"

#include <string>
#include <vector>
#include <cmath>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
namespace py = pybind11;

void signalHandler(int s)
{
  printf("exit by user interrupt\n");
  exit(1);
}

int main(int argc, char *argv[])
{
  py::scoped_interpreter python;

  struct sigaction sig_handler;
  sig_handler.sa_handler = signalHandler;
  sigemptyset(&sig_handler.sa_mask);
  sig_handler.sa_flags = 0;
  sigaction(SIGINT, &sig_handler, NULL);

  std::string config_path;
  if (argc == 1) {
    config_path = "../config.yaml";
  } else {
    config_path = argv[1];
  }
  printf("use config: %s\n", config_path.c_str());

  // parse config yaml
  cv::FileStorage config(config_path, cv::FileStorage::READ);
  std::string rosbag_path      = config["rosbag_path"];
  std::string pickle_path      = config["pickle_path"];
  std::string pointcloud_topic = config["pointcloud_topic"];
  std::string imu_topic        = config["imu_topic"];
  std::string gps_topic        = config["gps_topic"];

  cv::Mat cv_Tvl;
  Eigen::Matrix4d Tvl;
  config["extrinsic_lidar"] >> cv_Tvl;
  cv::cv2eigen(cv_Tvl, Tvl);

  printf("create directory \"%s\" to store pkl", pickle_path.c_str());
  create_directory(pickle_path);

  // parse sensor config to cfg.yaml
  py::dict sensor_config = generate_sensor_config(config);
  write_config(pickle_path + "/" + "cfg.yaml", sensor_config);

  // parse rosbag
  RosbagReader rosbag(rosbag_path, config_path);
  std::vector<Imu_t> imus = rosbag.readImu(imu_topic);
  std::vector<Ins_t> gpss = rosbag.readGps(gps_topic);

  // iterate pointcloud scan
  uint32_t frame_size = rosbag.getFrameSize(pointcloud_topic);
  printf("\nstart to convert LiDAR topics, total num: %u\n", frame_size);
  py::dict data;
  uint64_t prev_scan_stamp = 0, curr_scan_stamp = 0;
  rosbag.startScanIter(pointcloud_topic);
  for (uint32_t i = 0; i < frame_size; i++)
  {
    // parse pointcloud
    PointCloud::Ptr scan(new PointCloud());
    curr_scan_stamp = rosbag.readScan(scan);
    transformPointCloud(scan, Tvl);

    if (i > 0) // write synced data to disk
    {
      // sync imu
      std::vector<Imu_t> sync_imus = sync_data(curr_scan_stamp, imus);
      if (sync_imus.size() > 0)
      {
        data["ins_valid"] = true;
        data["imu_data"]  = imu_to_numpy(sync_imus);
      }
      // sync gps
      std::vector<Ins_t> sync_gpss = sync_data(curr_scan_stamp, gpss);
      if (sync_gpss.size() > 0)
      {
        data["ins_data"] = ins_to_dict(sync_gpss.back());
      }

      write_pickle(pickle_path, i - 1, data);
      data = py::dict();
    } else {
      std::vector<Imu_t> sync_imus = sync_data(curr_scan_stamp, imus);
      std::vector<Ins_t> sync_gpss = sync_data(curr_scan_stamp, gpss);
    }

    // initialize the data dict
    data["lidar_valid"]    = true;
    data["image_valid"]    = false;
    data["radar_valid"]    = false;
    data["ins_valid"]      = false;
    data["points"]         = py::dict();
    data["points_attr"]    = py::dict();
    data["image"]          = py::dict();
    data["image_param"]    = py::dict();
    data["radar"]          = py::dict();
    data["ins_data"]       = ins_to_dict(Ins_t());
    data["imu_data"]       = imu_to_numpy(std::vector<Imu_t>());
    data["motion_t"]       = generate_identy_matrix();
    data["motion_heading"] = 0;
    data["motion_valid"]   = false;

    // timestamp
    uint64_t timestep = prev_scan_stamp == 0 ? 100000 : (curr_scan_stamp - prev_scan_stamp);
    prev_scan_stamp = curr_scan_stamp;
    data["frame_start_timestamp"]                  = curr_scan_stamp;
    data["frame_timestamp_monotonic"]              = curr_scan_stamp;
    data["timestep"]                               = timestep;

    // convert LiDAR
    data["points"]["0-Custom"]                     = scan_to_numpy_points(scan);
    data["points_attr"]["0-Custom"]                = py::dict();
    data["points_attr"]["0-Custom"]["timestamp"]   = curr_scan_stamp;
    data["points_attr"]["0-Custom"]["points_attr"] = scan_to_numpy_stamp (scan);

    printProgress(double(i) / frame_size);
  }
  rosbag.StopScanIter();

  printf("\nDone, convert success\n");
  return 0;
}
