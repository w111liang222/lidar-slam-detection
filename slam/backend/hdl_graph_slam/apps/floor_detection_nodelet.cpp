// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <iostream>

#include <boost/optional.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "Logger.h"
#include "mapping_types.h"

namespace hdl_graph_slam {

class FloorDetectionNodelet {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FloorDetectionNodelet() {}
  virtual ~FloorDetectionNodelet() {}

  virtual void onInit() {
    initialize_params();
  }

  virtual void deInit() {}

  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    sensor_height = 0.0;          // approximate sensor height [m]
    height_clip_range_low  = 2.0;
    height_clip_range_high = 1.0; // points with heights in [sensor_height - height_clip_range_low, sensor_height + height_clip_range_high] will be used for floor detection
    floor_pts_thresh = 1024;       // minimum number of support points of RANSAC to accept a detected floor plane
    floor_normal_thresh = 10.0;   // verticality check thresold for the detected floor plane [deg]
    use_normal_filtering = true;  // if true, points with "non-"vertical normals will be filtered before RANSAC
    normal_filter_thresh = 20.0;  // "non-"verticality check threshold [deg]

    points_topic = "/velodyne_points";
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  FloorCoeffs cloud_callback(const PointCloud::Ptr& cloud) {
    FloorCoeffs coeffs;
    if(cloud->empty()) {
      return coeffs;
    }

    // floor detection
    boost::optional<Eigen::Vector4f> floor = detect(cloud);

    // publish the detected floor coefficients
    coeffs.timestamp = cloud->header.stamp;
    if(floor) {
      coeffs.coeffs.resize(4);
      for(int i = 0; i < 4; i++) {
        coeffs.coeffs[i] = (*floor)[i];
      }
    }
    return coeffs;
  }

  /**
   * @brief detect the floor plane from a point cloud
   * @param cloud  input cloud
   * @return detected floor plane coefficients
   */
  boost::optional<Eigen::Vector4f> detect(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    // filtering before RANSAC (height and normal filtering)
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    filtered = cloud;
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range_low), false);
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range_high), true);

    if(use_normal_filtering) {
      filtered = normal_filtering(filtered);
    }

    // too few points for RANSAC
    if(filtered->size() < floor_pts_thresh) {
      return boost::none;
    }

    // RANSAC
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // too few inliers
    if(inliers->indices.size() < floor_pts_thresh) {
      return boost::none;
    }

    // verticality check of the detected floor's normal
    Eigen::Vector4f reference = Eigen::Vector4f::UnitZ();

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

    double dot = coeffs.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
      // the normal is not vertical
      return boost::none;
    }

    // make the normal upward
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
      coeffs *= -1.0f;
    }

    // pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(filtered);
    // extract.setIndices(inliers);
    // extract.filter(*inlier_cloud);
    // inlier_cloud->header = cloud->header;
    // floor_points_pub.publish(*inlier_cloud);

    return Eigen::Vector4f(coeffs);
  }

  /**
   * @brief plane_clip
   * @param src_cloud
   * @param plane
   * @param negative
   * @return
   */
  pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) const {
    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);

    return dst_cloud;
  }

  /**
   * @brief filter points with non-vertical normals
   * @param cloud  input cloud
   * @return filtered cloud
   */
  pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(10);
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    ne.compute(*normals);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    filtered->reserve(cloud->size());

    for(int i = 0; i < cloud->size(); i++) {
      float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
      if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
        filtered->push_back(cloud->at(i));
      }
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
  }

private:
  std::string points_topic;

  // floor detection parameters
  // see initialize_params() for the details
  double sensor_height;
  double height_clip_range_high;
  double height_clip_range_low;

  int floor_pts_thresh;
  double floor_normal_thresh;

  bool use_normal_filtering;
  double normal_filter_thresh;
};

}  // namespace hdl_graph_slam

hdl_graph_slam::FloorDetectionNodelet floorNode;

void init_floor_node() {
  floorNode.onInit();
}

void deinit_floor_node() {
  floorNode.deInit();
}

FloorCoeffs enqueue_floor(PointCloud::Ptr &points) {
  return floorNode.cloud_callback(points);
}