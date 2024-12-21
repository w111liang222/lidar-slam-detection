#ifndef __UTILS__H
#define __UTILS__H

#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <opencv2/opencv.hpp>

#include "nav_msgs/Odometry.hpp"
#include "nav_msgs/Path.hpp"
#include "sensor_msgs/CompressedImage.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/PointCloud.hpp"

// sensor_msgs::PointCloud --> pcl::PointCloud
void toPCL(sensor_msgs::PointCloud &msg, pcl::PointCloud<pcl::PointXYZI> &pcl);
void toPCL(sensor_msgs::PointCloud &msg, pcl::PointCloud<pcl::PointXYZRGB> &pcl);
void toPCL(sensor_msgs::PointCloud &msg, pcl::PointCloud<pcl::PointXYZINormal> &pcl);
void toPCL(sensor_msgs::PointCloud &msg, pcl::PCLPointCloud2 &pcl, bool move = false);

// pcl::PointCloud --> sensor_msgs::PointCloud
void fromPCL(pcl::PointCloud<pcl::PointXYZI> &pcl, sensor_msgs::PointCloud &msg);
void fromPCL(pcl::PointCloud<pcl::PointXYZRGB> &pcl, sensor_msgs::PointCloud &msg);
void fromPCL(pcl::PointCloud<pcl::PointXYZINormal> &pcl, sensor_msgs::PointCloud &msg);
void fromPCL(pcl::PCLPointCloud2 &pcl, sensor_msgs::PointCloud &msg, bool move = false);

// nav_msgs::Odometry --> Eigen::Matrix4d
void toOdometry(const nav_msgs::Odometry &msg, Eigen::Matrix4d &odometry);

// Eigen::Matrix4d --> nav_msgs::Odometry
void fromOdometry(const Eigen::Matrix4d &odometry, nav_msgs::Odometry &msg);

// nav_msgs::Path --> std::vector<Eigen::Matrix4d>
void toPath(const nav_msgs::Path &msg, std::vector<Eigen::Matrix4d> &path);

// std::vector<Eigen::Matrix4d> --> nav_msgs::Path
void fromPath(const std::vector<Eigen::Matrix4d> &path, nav_msgs::Path &msg);

// sensor_msgs::Image(CompressedImage) --> Cv::Mat
void toCv(const sensor_msgs::CompressedImage &msg, cv::Mat &img);
void toCv(const sensor_msgs::Image &msg, cv::Mat &img);

// Cv::Mat --> sensor_msgs::Image(CompressedImage)
void fromCv(const cv::Mat &img, sensor_msgs::CompressedImage &msg);
void fromCv(const cv::Mat &img, sensor_msgs::Image &msg);

#endif  //__UTILS__H
