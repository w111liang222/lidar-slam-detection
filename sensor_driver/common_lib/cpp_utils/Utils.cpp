#include "Utils.h"

#include <pcl/conversions.h>

void toPCL(sensor_msgs::PointCloud &msg, pcl::PointCloud<pcl::PointXYZI> &pcl) {
  pcl::PCLPointCloud2 pcl2;
  toPCL(msg, pcl2, true);
  pcl::fromPCLPointCloud2(pcl2, pcl);
}

void toPCL(sensor_msgs::PointCloud &msg, pcl::PointCloud<pcl::PointXYZRGB> &pcl) {
  pcl::PCLPointCloud2 pcl2;
  toPCL(msg, pcl2, true);
  pcl::fromPCLPointCloud2(pcl2, pcl);
}

void toPCL(sensor_msgs::PointCloud &msg, pcl::PointCloud<pcl::PointXYZINormal> &pcl) {
  pcl::PCLPointCloud2 pcl2;
  toPCL(msg, pcl2, true);
  pcl::fromPCLPointCloud2(pcl2, pcl);
}

void toPCL(sensor_msgs::PointCloud &msg, pcl::PCLPointCloud2 &pcl, bool move) {
  pcl.header.stamp = msg.header.stamp;
  pcl.height = msg.height;
  pcl.width = msg.width;

  // point field
  pcl.fields.resize(msg.fields.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = msg.fields.begin();
  size_t i = 0;
  for(; it != msg.fields.end(); ++it, ++i) {
    pcl.fields[i].name = it->name;
    pcl.fields[i].offset = it->offset;
    pcl.fields[i].datatype = it->datatype;
    pcl.fields[i].count = it->count;
  }

  pcl.is_bigendian = msg.is_bigendian;
  pcl.point_step = msg.point_step;
  pcl.row_step = msg.row_step;
  pcl.is_dense = msg.is_dense;

  if (move) {
    pcl.data.swap(msg.data);
  } else {
    pcl.data = msg.data;
  }
}

void fromPCL(pcl::PointCloud<pcl::PointXYZI> &pcl, sensor_msgs::PointCloud &msg) {
  pcl::PCLPointCloud2 pcl2;
  pcl::toPCLPointCloud2(pcl, pcl2);
  fromPCL(pcl2, msg, true);
}

void fromPCL(pcl::PointCloud<pcl::PointXYZRGB> &pcl, sensor_msgs::PointCloud &msg) {
  pcl::PCLPointCloud2 pcl2;
  pcl::toPCLPointCloud2(pcl, pcl2);
  fromPCL(pcl2, msg, true);
}

void fromPCL(pcl::PointCloud<pcl::PointXYZINormal> &pcl, sensor_msgs::PointCloud &msg) {
  pcl::PCLPointCloud2 pcl2;
  pcl::toPCLPointCloud2(pcl, pcl2);
  fromPCL(pcl2, msg, true);
}

void fromPCL(pcl::PCLPointCloud2 &pcl, sensor_msgs::PointCloud &msg, bool move) {
  msg.header.stamp = pcl.header.stamp;
  msg.height = pcl.height;
  msg.width = pcl.width;

  // point field
  msg.fields_num = pcl.fields.size();
  msg.fields.resize(msg.fields_num);
  std::vector<pcl::PCLPointField>::const_iterator it = pcl.fields.begin();
  size_t i = 0;
  for(; it != pcl.fields.end(); ++it, ++i) {
    msg.fields[i].name = it->name;
    msg.fields[i].offset = it->offset;
    msg.fields[i].datatype = it->datatype;
    msg.fields[i].count = it->count;
  }

  msg.is_bigendian = pcl.is_bigendian;
  msg.point_step = pcl.point_step;
  msg.row_step = pcl.row_step;
  msg.is_dense = pcl.is_dense;

  msg.data_num = pcl.data.size();
  if (move) {
    msg.data.swap(pcl.data);
  } else {
    msg.data = pcl.data;
  }
}

void fromOdometry(const Eigen::Matrix4d &odometry, nav_msgs::Odometry &msg) {
  Eigen::Quaterniond q = Eigen::Quaterniond(odometry.block<3, 3>(0, 0));

  msg.pose.pose.position.x = odometry(0, 3);
  msg.pose.pose.position.y = odometry(1, 3);
  msg.pose.pose.position.z = odometry(2, 3);
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
}

void toOdometry(const nav_msgs::Odometry &msg, Eigen::Matrix4d &odometry) {
  odometry = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
  q.w() = msg.pose.pose.orientation.w;

  odometry.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  odometry(0, 3) = msg.pose.pose.position.x;
  odometry(1, 3) = msg.pose.pose.position.y;
  odometry(2, 3) = msg.pose.pose.position.z;
}

void fromPath(const std::vector<Eigen::Matrix4d> &path, nav_msgs::Path &msg) {
  msg.size = path.size();
  msg.poses.resize(msg.size);

  for (size_t i = 0; i < msg.size; i++) {
    Eigen::Quaterniond q = Eigen::Quaterniond(path[i].block<3, 3>(0, 0));
    msg.poses[i].pose.position.x = path[i](0, 3);
    msg.poses[i].pose.position.y = path[i](1, 3);
    msg.poses[i].pose.position.z = path[i](2, 3);
    msg.poses[i].pose.orientation.x = q.x();
    msg.poses[i].pose.orientation.y = q.y();
    msg.poses[i].pose.orientation.z = q.z();
    msg.poses[i].pose.orientation.w = q.w();
  }
}

void toPath(const nav_msgs::Path &msg, std::vector<Eigen::Matrix4d> &path) {
  path.resize(msg.size);
  for (size_t i = 0; i < msg.size; i++) {
    path[i] = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q;
    q.x() = msg.poses[i].pose.orientation.x;
    q.y() = msg.poses[i].pose.orientation.y;
    q.z() = msg.poses[i].pose.orientation.z;
    q.w() = msg.poses[i].pose.orientation.w;

    path[i].block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    path[i](0, 3) = msg.poses[i].pose.position.x;
    path[i](1, 3) = msg.poses[i].pose.position.y;
    path[i](2, 3) = msg.poses[i].pose.position.z;
  }
}

void toCv(const sensor_msgs::CompressedImage &msg, cv::Mat &img) {
  if (msg.format.compare("jpeg") == 0) {
    img = cv::imdecode(cv::Mat(1, msg.size, CV_8UC1, const_cast<uint8_t*>(&msg.data[0])), cv::IMREAD_UNCHANGED);
  }
}

void toCv(const sensor_msgs::Image &msg, cv::Mat &img) {
  int type = 0;
  if (msg.encoding.compare("BGR8") == 0) {
    type = CV_8UC3;
  } else if (msg.encoding.compare("YUV_I420") == 0) {
    type = CV_8UC1;
  } else {
    return;
  }

  img = cv::Mat((int)msg.height, (int)msg.width, type, const_cast<uint8_t*>(&msg.data[0]), (size_t)msg.step);
}

void fromCv(const cv::Mat &img, sensor_msgs::CompressedImage &msg) {
  cv::Mat BGR;
  if (img.type() == CV_8UC3) {
    BGR = img;
  } else if (img.type() == CV_8UC1) {
    cv::cvtColor(img, BGR, cv::COLOR_YUV2BGR_I420);
  }

  msg.format = "jpeg";
  cv::imencode(".jpg", BGR, msg.data);
  msg.size = msg.data.size();
}

void fromCv(const cv::Mat &img, sensor_msgs::Image &msg) {
  msg.height = img.rows;
  msg.width = img.cols;
  if (img.type() == CV_8UC3) {
    msg.encoding = "BGR8";
  } else if (img.type() == CV_8UC1) {
    msg.encoding = "YUV_I420";
  }
  msg.is_bigendian = false;
  msg.step = img.cols * img.elemSize();

  msg.size = msg.step * img.rows;
  msg.data.resize(msg.size);
  if (img.isContinuous()) {
    memcpy((char*)(&msg.data[0]), img.data, msg.size);
  } else {
    // Copy by row
    unsigned char* msg_data_ptr = (unsigned char*)(&msg.data[0]);
    unsigned char* cv_data_ptr = img.data;
    for (int i = 0; i < img.rows; ++i) {
      memcpy(msg_data_ptr, cv_data_ptr, msg.step);
      msg_data_ptr += msg.step;
      cv_data_ptr += img.step;
    }
  }
}