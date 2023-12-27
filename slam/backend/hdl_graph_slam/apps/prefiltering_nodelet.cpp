// SPDX-License-Identifier: BSD-2-Clause

#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "slam_base.h"
#include "mapping_types.h"

namespace hdl_graph_slam {

class PrefilteringNodelet {
public:
  typedef pcl::PointXYZI PointT;

  PrefilteringNodelet() {}
  virtual ~PrefilteringNodelet() {}

  virtual void onInit(InitParameter &param) {
    initialize_params(param);
  }

  virtual void deInit() {
    downsample_filter = nullptr;
    outlier_removal_filter = nullptr;
  }

  void initialize_params(InitParameter &param) {
    std::string downsample_method = "VOXELGRID";
    double downsample_resolution = param.resolution;

    if(downsample_method == "VOXELGRID") {
      LOG_INFO("downsample: VOXELGRID {}", downsample_resolution);
      auto voxelgrid = new pcl::VoxelGrid<PointT>();
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter.reset(voxelgrid);
    } else if(downsample_method == "APPROX_VOXELGRID") {
      LOG_INFO("downsample: APPROX_VOXELGRID {}", downsample_resolution);
      pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        LOG_WARN("warning: unknown downsampling type ({})", downsample_method);
        LOG_WARN("       : use passthrough filter");
      }
      downsample_filter = nullptr;
      LOG_INFO("downsample: NONE");
    }

    std::string outlier_removal_method = "NONE";
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = 20;
      double stddev_mul_thresh = 1.0;
      LOG_INFO("outlier_removal: STATISTICAL {} - {}", mean_k, stddev_mul_thresh);

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = 0.5;
      int min_neighbors = 2;
      LOG_INFO("outlier_removal: RADIUS {} - {}", radius, min_neighbors);

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } else {
      outlier_removal_filter = nullptr;
      LOG_INFO("outlier_removal: NONE");
    }

    use_distance_filter = true;
    distance_near_thresh = 0.1;
    distance_far_thresh = 100.0;

    base_link_frame = "";
  }

  // void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
  //   imu_queue.push_back(imu_msg);
  // }

  PointCloud::Ptr cloud_callback(const PointCloud::Ptr& src_cloud) {
    if(src_cloud->empty()) {
      return PointCloud::Ptr(new PointCloud());
    }

    // src_cloud = deskewing(src_cloud);

    // if base_link_frame is defined, transform the input cloud to the frame
    // if(!base_link_frame.empty()) {
    //   if(!tf_listener.canTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0))) {
    //     std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
    //   }

    //   tf::StampedTransform transform;
    //   tf_listener.waitForTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
    //   tf_listener.lookupTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), transform);

    //   pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
    //   pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
    //   transformed->header.frame_id = base_link_frame;
    //   transformed->header.stamp = src_cloud->header.stamp;
    //   src_cloud = transformed;
    // }

    pcl::PointCloud<PointT>::Ptr filtered = downsample(src_cloud);
    filtered = distance_filter(filtered);
    filtered = outlier_removal(filtered);
    return filtered;
  }

  pcl::PointCloud<PointT>::Ptr downsample(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::Ptr outlier_removal(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::Ptr distance_filter(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
      double d = p.getVector3fMap().norm();
      return d > distance_near_thresh && d < distance_far_thresh;
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

  // pcl::PointCloud<PointT>::ConstPtr deskewing(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  //   uint64_t stamp = cloud->header.stamp;
  //   if(imu_queue.empty()) {
  //     return cloud;
  //   }

  //   // the color encodes the point number in the point sequence
  //   if(colored_pub.getNumSubscribers()) {
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
  //     colored->header = cloud->header;
  //     colored->is_dense = cloud->is_dense;
  //     colored->width = cloud->width;
  //     colored->height = cloud->height;
  //     colored->resize(cloud->size());

  //     for(int i = 0; i < cloud->size(); i++) {
  //       double t = static_cast<double>(i) / cloud->size();
  //       colored->at(i).getVector4fMap() = cloud->at(i).getVector4fMap();
  //       colored->at(i).r = 255 * t;
  //       colored->at(i).g = 128;
  //       colored->at(i).b = 255 * (1 - t);
  //     }
  //     colored_pub.publish(*colored);
  //   }

  //   sensor_msgs::ImuConstPtr imu_msg = imu_queue.front();

  //   auto loc = imu_queue.begin();
  //   for(; loc != imu_queue.end(); loc++) {
  //     imu_msg = (*loc);
  //     if((*loc)->header.stamp > stamp) {
  //       break;
  //     }
  //   }

  //   imu_queue.erase(imu_queue.begin(), loc);

  //   Eigen::Vector3f ang_v(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
  //   ang_v *= -1;

  //   pcl::PointCloud<PointT>::Ptr deskewed(new pcl::PointCloud<PointT>());
  //   deskewed->header = cloud->header;
  //   deskewed->is_dense = cloud->is_dense;
  //   deskewed->width = cloud->width;
  //   deskewed->height = cloud->height;
  //   deskewed->resize(cloud->size());

  //   double scan_period = 0.1;
  //   for(int i = 0; i < cloud->size(); i++) {
  //     const auto& pt = cloud->at(i);

  //     // TODO: transform IMU data into the LIDAR frame
  //     double delta_t = scan_period * static_cast<double>(i) / cloud->size();
  //     Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
  //     Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();

  //     deskewed->at(i) = cloud->at(i);
  //     deskewed->at(i).getVector3fMap() = pt_;
  //   }

  //   return deskewed;
  // }

private:
  // std::vector<sensor_msgs::ImuConstPtr> imu_queue;

  std::string base_link_frame;

  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;
};

}  // namespace hdl_graph_slam

hdl_graph_slam::PrefilteringNodelet filterNode;

void init_filter_node(InitParameter &param) {
  filterNode.onInit(param);
}

void deinit_filter_node() {
  filterNode.deInit();
}

PointCloud::Ptr enqueue_filter(PointCloud::Ptr &points) {
  return filterNode.cloud_callback(points);
}