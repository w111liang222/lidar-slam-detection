#include "RosbagReader.h"

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "cloud_convert.h"
#include "nmea_converter.hpp"

static CloudConvert g_cloud_convert;
static rosbag::View::iterator g_iter;

RosbagReader::RosbagReader(std::string file, std::string config_path)
{
  pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ERROR);
  mConfig.open(config_path, cv::FileStorage::READ);
  mConfig["dataset"] >> mDataSet;
  g_cloud_convert.init(mConfig["lidar_type"], mConfig["scan_line"], mConfig["time_scale"]);

  mBag = new rosbag::Bag();
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->open(file, rosbag::bagmode::Read);
}
RosbagReader::~RosbagReader()
{
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->close();
  delete mBagPtr;
}

uint32_t RosbagReader::getFrameSize(std::string topic)
{
  std::vector<std::string> topics{topic};
  auto mBagPtr = (rosbag::Bag *)mBag;
  rosbag::View view(*mBagPtr, rosbag::TopicQuery(topics));
  return view.size();
}

std::vector<Imu_t> RosbagReader::readImu(std::string topic)
{
  std::vector<std::string> topics{topic};
  auto mBagPtr = (rosbag::Bag *)mBag;
  rosbag::View view(*mBagPtr, rosbag::TopicQuery(topics));
  printf("\nstart to convert IMU topics, total num: %u\n", view.size());

  int have_gravity = mConfig["have_gravity"];
  int acc_unit     = mConfig["acc_unit"];
  std::vector<Imu_t> imus;
  size_t message_counter = 0;
  for (const rosbag::MessageInstance& m : view)
  {
    printProgress(double(++message_counter) / view.size());
    // parse & convert
    sensor_msgs::Imu imuMsg = *(m.instantiate<sensor_msgs::Imu>());
    Imu_t imu;
    imu.timestamp = imuMsg.header.stamp.sec * 1000000ULL + imuMsg.header.stamp.nsec / 1000ULL;
    imu.gyro_x    = imuMsg.angular_velocity.x;
    imu.gyro_y    = imuMsg.angular_velocity.y;
    imu.gyro_z    = imuMsg.angular_velocity.z;
    imu.acc_x     = imuMsg.linear_acceleration.x * (acc_unit == 0 ? 1.0 : 9.81);
    imu.acc_y     = imuMsg.linear_acceleration.y * (acc_unit == 0 ? 1.0 : 9.81);
    imu.acc_z     = imuMsg.linear_acceleration.z * (acc_unit == 0 ? 1.0 : 9.81) + (have_gravity == 0 ? 9.81 : 0.0);
    imus.push_back(imu);
  }
  return imus;
}

std::vector<Ins_t> RosbagReader::readGps(std::string topic)
{
  std::vector<std::string> topics{topic};
  auto mBagPtr = (rosbag::Bag *)mBag;
  rosbag::View view(*mBagPtr, rosbag::TopicQuery(topics));
  printf("\nstart to convert GPS topics, total num: %u\n", view.size());

  std::vector<Ins_t> gpss;
  size_t message_counter = 0;
  for (const rosbag::MessageInstance& m : view)
  {
    printProgress(double(++message_counter) / view.size());
    // convert
    sensor_msgs::NavSatFix gpsMsg;
    if (mDataSet.compare("UTBM") == 0) {
      static nmea_msgs::Sentence sentence;
      nmea_msgs::Gpgga gga;
      nmea_msgs::Gprmc rmc;
      auto msg = m.instantiate<nmea_msgs::Sentence>();
      sentence.header = msg->header;
      sentence.sentence = msg->sentence;
      nmea_converter(sentence, &gpsMsg, &gga, &rmc);
      if (gpsMsg.header.stamp.toSec() == 0) {
        continue;
      }
    } else {
      gpsMsg = *(m.instantiate<sensor_msgs::NavSatFix>());
      gpsMsg.status.status = 0; // Fix
    }

    Ins_t gps;
    gps.timestamp = gpsMsg.header.stamp.sec * 1000000ULL + gpsMsg.header.stamp.nsec / 1000ULL;
    gps.latitude  = gpsMsg.latitude;
    gps.longitude = gpsMsg.longitude;
    gps.altitude  = gpsMsg.altitude;
    gps.status    = (gpsMsg.status.status == 0) ? 1 : 0;
    gpss.push_back(gps);
  }
  return gpss;
}

void RosbagReader::startScanIter(std::string topic)
{
  std::vector<std::string> topics{topic};
  auto mBagPtr = (rosbag::Bag *)mBag;
  mScanView = new rosbag::View(*mBagPtr, rosbag::TopicQuery(topics));
  g_iter = ((rosbag::View* )mScanView)->begin();
}

void RosbagReader::StopScanIter()
{
  rosbag::View* view = (rosbag::View* )mScanView;
  delete view;
}

uint64_t RosbagReader::readScan(PointCloud::Ptr &scan)
{
  auto cloudMsg = g_iter++->instantiate<sensor_msgs::PointCloud2>();
  g_cloud_convert.Process(cloudMsg, scan);
  uint64_t timestamp = cloudMsg->header.stamp.sec * 1000000ULL + cloudMsg->header.stamp.nsec / 1000ULL;
  scan->header.stamp = timestamp;
  return timestamp;
}
