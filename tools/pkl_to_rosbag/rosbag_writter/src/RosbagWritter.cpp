#include "RosbagWritter.h"

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>

RosbagWritter::RosbagWritter(std::string file)
{
  mBag = new rosbag::Bag();
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->open(file, rosbag::bagmode::Write);
}
RosbagWritter::~RosbagWritter()
{
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->close();
}

void RosbagWritter::writeScan(std::string topic, const std::string frame, uint64_t timestamp,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*input, cloudMsg);
  cloudMsg.header.stamp.sec = timestamp / 1000000;
  cloudMsg.header.stamp.nsec = (timestamp % 1000000) * 1000;
  cloudMsg.header.frame_id = frame;
  ros::Time topicTime;
  topicTime.sec = timestamp / 1000000;
  topicTime.nsec = (timestamp % 1000000) * 1000;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, cloudMsg);
}

void RosbagWritter::writeImage(std::string topic, const std::string frame, uint64_t timestamp,
                               cv::Mat input)
{
  // const cv::Mat image = input;
  cv_bridge::CvImage cvi;
  cvi.header.stamp.sec = timestamp / 1000000;
  cvi.header.stamp.nsec = (timestamp % 1000000) * 1000;
  cvi.header.frame_id = frame;
  cvi.encoding = "bgr8";
  cvi.image = input;

  sensor_msgs::Image im;
  cvi.toImageMsg(im);
  ros::Time topicTime;
  topicTime.sec = timestamp / 1000000;
  topicTime.nsec = (timestamp % 1000000) * 1000;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, im);
}

void RosbagWritter::writeCompressedImage(std::string topic, const std::string frame, uint64_t timestamp,
                                         cv::Mat input)
{
  sensor_msgs::CompressedImage im;
  im.header.stamp.sec = timestamp / 1000000;
  im.header.stamp.nsec = (timestamp % 1000000) * 1000;
  im.header.frame_id = frame;
  im.format = "jpeg";

  size_t size = input.cols * input.rows * input.elemSize();
  im.data.resize(size);
  memcpy(reinterpret_cast<char *>(&im.data[0]), input.data, size);
  ros::Time topicTime;
  topicTime.sec = timestamp / 1000000;
  topicTime.nsec = (timestamp % 1000000) * 1000;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, im);
}

void RosbagWritter::writeImu(std::string topic, const std::string frame, Imu_t &imu)
{
  sensor_msgs::Imu imuMsg;
  ros::Time topicTime;
  topicTime.sec = imu.timestamp / 1000000;
  topicTime.nsec = (imu.timestamp % 1000000) * 1000;

  imuMsg.header.stamp = topicTime;
  imuMsg.angular_velocity.x = imu.gyro_x;
  imuMsg.angular_velocity.y = imu.gyro_y;
  imuMsg.angular_velocity.z = imu.gyro_z;
  imuMsg.linear_acceleration.x = imu.acc_x;
  imuMsg.linear_acceleration.y = imu.acc_y;
  imuMsg.linear_acceleration.z = imu.acc_z;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, imuMsg);
}

void RosbagWritter::writeIns(std::string topic, const std::string frame, Ins_t &ins)
{
  sensor_msgs::NavSatFix insMsg;
  ros::Time topicTime;
  topicTime.sec = ins.timestamp / 1000000;
  topicTime.nsec = (ins.timestamp % 1000000) * 1000;

  insMsg.header.stamp = topicTime;
  insMsg.latitude = ins.latitude;
  insMsg.longitude = ins.longitude;
  insMsg.altitude = ins.altitude;
  insMsg.position_covariance[0] = ins.heading;
  insMsg.position_covariance[1] = ins.pitch;
  insMsg.position_covariance[2] = ins.roll;
  insMsg.position_covariance[3] = ins.Ve;
  insMsg.position_covariance[4] = ins.Vn;
  insMsg.position_covariance[5] = ins.Vu;
  insMsg.status.status = ins.status;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, insMsg);
}

void RosbagWritter::writeTimeStamp(std::string topic, uint64_t timestamp, uint64_t data)
{
  std_msgs::UInt64 stdMsg;
  ros::Time topicTime;
  topicTime.sec = timestamp / 1000000;
  topicTime.nsec = (timestamp % 1000000) * 1000;

  stdMsg.data = data;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, stdMsg);
}