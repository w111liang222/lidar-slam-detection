#include "RosbagWritter.h"
#include "GetPklData.h"

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
#include <getopt.h>
namespace py = pybind11;

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 80

void printProgress(double percentage)
{
  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);
}

void signalHandler(int s)
{
  printf("exit by user interrupt\n");
  exit(1);
}

int main(int argc, char *argv[])
{
  py::scoped_interpreter python;
  std::vector<std::string> filelists;

  struct sigaction sig_handler;
  sig_handler.sa_handler = signalHandler;
  sigemptyset(&sig_handler.sa_mask);
  sig_handler.sa_flags = 0;
  sigaction(SIGINT, &sig_handler, NULL);

  int opt;
  int option_index = 0;

  std::string input_p;
  std::string output_p;
  struct option long_options[] = {
      {"input_path", optional_argument, NULL, 'i'},
      {"output_path", optional_argument, NULL, 'o'},
      {0, 0, 0, 0}};

  while ((opt = getopt_long(argc, argv, "i:o:", long_options, &option_index)) != -1)
  {
    switch (opt)
    {
    case 'i':
      input_p = optarg;
      break;
    case 'o':
      output_p = optarg;
      break;
    }
  }

  if (input_p.length() > 0 && input_p.back() != '/')
  {
    input_p = input_p + '/';
  }
  filelists = getFiles(input_p);
  if (filelists.empty())
  {
    printf("[ERROR] No .pkl file found!\n");
    return 0;
  }

  char ch = output_p.back();
  std::string bag_name;
  if (output_p == "" || ch == '/')
  {
    bag_name = output_p + std::string("rosbag.bag");
  }
  else
  {
    bag_name = output_p;
  }

  RosbagWritter wbag(bag_name);
  int file_index = 0;
  for (auto c : filelists)
  {
    auto file_path = input_p + c;
    py::dict data_dict = getPklData(file_path);

    // TimeStamp
    uint64_t timestamp = data_dict["frame_start_timestamp"].cast<uint64_t>();
    wbag.writeTimeStamp("frame_start_timestamp", timestamp, timestamp);
    wbag.writeTimeStamp("timestep", timestamp, data_dict["timestep"].cast<uint64_t>());

    // Lidar data
    for (auto it : data_dict["points"].attr("keys")())
    {
      std::string lidarname = it.cast<std::string>();
      py::dict points_obj = data_dict["points"];
      pcl::PointCloud<pcl::PointXYZI>::Ptr points_cloud = toPclPointCloud(points_obj[lidarname.c_str()].cast<py::array_t<float>>());

      int pos = 0;
      lidarname = "lidar" + lidarname;
      while (std::string::npos != (pos = lidarname.find("-")))
      {
        lidarname.erase(pos, 1);
      }
      wbag.writeScan(lidarname, "base_link", timestamp, points_cloud);
    }

    // Camera data
    for (auto it : data_dict["image"].attr("keys")())
    {
      std::string imagename = it.cast<std::string>();
      py::dict image_obj = data_dict["image"];
      py::bytes image_bytes = image_obj[imagename.c_str()].cast<py::bytes>();
      cv::Mat image_compressed = toCvMatImage(image_bytes);

      int pos = 0;
      imagename = "image" + imagename;
      while (std::string::npos != (pos = imagename.find(":")))
      {
        imagename.erase(pos, 1);
      }
      // wbag.writeCompressedImage(imagename, "base_link", timestamp, image_compressed);
#if (CV_VERSION_MAJOR >= 4)
      cv::Mat image_bgr = cv::imdecode(image_compressed, cv::IMREAD_UNCHANGED);
#else
      cv::Mat image_bgr = cv::imdecode(image_compressed, CV_LOAD_IMAGE_UNCHANGED);
#endif
      wbag.writeImage(imagename, "base_link", timestamp, image_bgr);
    }

    // INS data
    if (data_dict.contains("ins_data") && data_dict["ins_valid"].cast<bool>() && data_dict["ins_data"]["timestamp"].cast<uint64_t>() != 0)
    {
      Ins_t ins = toIns(data_dict["ins_data"]);
      wbag.writeIns("ins_raw", "base_link", ins);
    }

    // SLAM Localization data
    if (data_dict.contains("pose") && data_dict["slam_valid"].cast<bool>())
    {
      Ins_t ins = toIns(data_dict["pose"]);
      wbag.writeIns("localization", "base_link", ins);
    }

    // IMU data
    if (data_dict.contains("imu_data"))
    {
      auto imu_data = data_dict["imu_data"].cast<py::array_t<double>>();
      auto imu_ref = imu_data.unchecked<2>();
      for (size_t i = 0; i < imu_ref.shape(0); i++)
      {
        Imu_t imu;
        imu.gyro_x = imu_ref(i, 1) / 180.0 * M_PI;
        imu.gyro_y = imu_ref(i, 2) / 180.0 * M_PI;
        imu.gyro_z = imu_ref(i, 3) / 180.0 * M_PI;
        imu.acc_x  = imu_ref(i, 4) * 9.81;
        imu.acc_y  = imu_ref(i, 5) * 9.81;
        imu.acc_z  = imu_ref(i, 6) * 9.81;
        imu.timestamp = uint64_t(imu_ref(i, 0));
        wbag.writeImu("imu_raw", "base_link", imu);
      }
    }

    printProgress(double(file_index++) / filelists.size());
  }
  printf("\nDone, convert %lu pickle files\n", filelists.size());
  return 0;
}
