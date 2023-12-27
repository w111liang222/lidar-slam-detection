// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <thread>
#include <iostream>
#include <atomic>
#include <sys/time.h>
#include <sys/prctl.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <hdl_graph_slam/registrations.hpp>

#include "slam_base.h"
#include "mapping_types.h"

namespace hdl_graph_slam {

const int MAX_LOCAL_MAP_POINTS = 100000;

typedef pcl::PointXYZI PointT;

struct RegistrationType {
  RegistrationType(const pcl::Registration<PointT, PointT>::Ptr& in) {
    registration = in;
  }
  pcl::Registration<PointT, PointT>::Ptr registration;
};

class ScanMatchingOdometryNodelet {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanMatchingOdometryNodelet() {}
  virtual ~ScanMatchingOdometryNodelet() {}

  virtual void onInit(InitParameter &param) {
    LOG_INFO("initializing scan_matching_odometry_nodelet...");

    initialize_params(param);
    thread_start = true;
    map_update_thread.reset(new std::thread(&ScanMatchingOdometryNodelet::map_update_loop, this));
  }

  virtual void deInit() {
    LOG_INFO("releasing scan_matching_odometry_nodelet...");
    if (map_update_thread != nullptr) {
      thread_start = false;
      map_update_thread->join();
      map_update_thread.reset(nullptr);
    }

    PointCloudType *points = nullptr;
    while (keyframe_queue.try_dequeue(points)) {
        delete points;
    }

    RegistrationType *registration_ptr = nullptr;
    while (registration_queue.try_dequeue(registration_ptr)) {
        delete registration_ptr;
    }

    keyframe = nullptr;

    downsample_filter = nullptr;
    registration = nullptr;
    for (int i = 0; i < 2; i++) {
      registrations[i] = nullptr;
    }
  }

  /**
   * @brief initialize parameters
   */
  void initialize_params(InitParameter &param) {
    points_topic = "/velodyne_points";
    odom_frame_id = "odom";
    robot_odom_frame_id = "robot_odom";

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = param.key_frame_distance / 2.0;
    keyframe_delta_angle = 1.0;
    keyframe_delta_time = 2.0;

    // Registration validation by thresholding
    transform_thresholding = false;
    max_acceptable_trans = 1.0;
    max_acceptable_angle = 1.0;

    keyframe = nullptr;

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
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
      LOG_INFO("downsample: NONE");
      pcl::PassThrough<PointT>::Ptr passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }

    pingpong_idx = 0;
    last_pingpong_idx = 1;
    for (int i = 0; i < 2; i++) {
#ifdef HAVE_CUDA_ENABLE
      registrations[i] = select_registration_method("FAST_VGICP_CUDA");
#else
      registrations[i] = select_registration_method("FAST_GICP");
#endif
    }
    registration = registrations[last_pingpong_idx];
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  Eigen::Matrix4d cloud_callback(const PointCloud::Ptr& cloud, const Eigen::Matrix4d& init_guess) {
    Eigen::Matrix4d pose = matching(cloud->header.stamp, cloud, init_guess.cast<float>());
    // publish_odometry(cloud->header.stamp, pose);
    return pose;
  }

  // void msf_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg, bool after_update) {
  //   if(after_update) {
  //     msf_pose_after_update = pose_msg;
  //   } else {
  //     msf_pose = pose_msg;
  //   }
  // }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::Ptr downsample(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  void map_update_loop() {
    prctl(PR_SET_NAME, "GICP Map", 0, 0, 0);

    PointCloud::Ptr local_map(new PointCloud());
    while (thread_start) {
      PointCloudType *points = nullptr;
      if (false == keyframe_queue.wait_dequeue_timed(points, 100000)) {
        continue;
      }
      std::shared_ptr<PointCloudType> pointPtr = std::shared_ptr<PointCloudType>(points);

      // accumulate points
      *local_map += *(pointPtr->points);
      if (local_map->points.size() > MAX_LOCAL_MAP_POINTS) {
        local_map->erase(local_map->begin(), local_map->begin() + (local_map->points.size() - MAX_LOCAL_MAP_POINTS));
      }

      if (pingpong_idx == last_pingpong_idx) {
        LOG_WARN("GICPM: previous local map has not been processed");
        continue;
      }

      auto filtered = downsample(local_map);
      registrations[pingpong_idx]->setInputTarget(filtered);
      registration_queue.enqueue(new RegistrationType(registrations[pingpong_idx]));
      pingpong_idx.store(last_pingpong_idx);
    }
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the keyframe cloud
   */
  Eigen::Matrix4d matching(const uint64_t& stamp, const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Matrix4f& init_guess) {
    if(!keyframe) {
      prev_trans.setIdentity();
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = cloud;
      registration->setInputTarget(keyframe);
      return Eigen::Matrix4d::Identity();
    }

    // update local map registration
    RegistrationType* registration_ptr;
    if (registration_queue.try_dequeue(registration_ptr)) {
      registration = registration_ptr->registration;
      last_pingpong_idx.store((last_pingpong_idx + 1) % 2);
      delete registration_ptr;
    }

    registration->setInputSource(cloud);

    // std::string msf_source;
    // Eigen::Isometry3f msf_delta = Eigen::Isometry3f::Identity();

    // if(false) {
      // if(msf_pose && msf_pose->header.stamp > keyframe_stamp && msf_pose_after_update && msf_pose_after_update->header.stamp > keyframe_stamp) {
      //   Eigen::Isometry3d pose0 = pose2isometry(msf_pose_after_update->pose.pose);
      //   Eigen::Isometry3d pose1 = pose2isometry(msf_pose->pose.pose);
      //   Eigen::Isometry3d delta = pose0.inverse() * pose1;

      //   msf_source = "imu";
      //   msf_delta = delta.cast<float>();
      // } else {
      //   std::cerr << "msf data is too old" << std::endl;
      // }
    // } else if(false && !prev_time) {
      // tf::StampedTransform transform;
      // if(tf_listener.waitForTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id, ros::Duration(0))) {
      //   tf_listener.lookupTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, prev_time, robot_odom_frame_id, transform);
      // } else if(tf_listener.waitForTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, prev_time, robot_odom_frame_id, ros::Duration(0))) {
      //   tf_listener.lookupTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, prev_time, robot_odom_frame_id, transform);
      // }

      // if(transform.stamp_.isZero()) {
      //   NODELET_WARN_STREAM("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      // } else {
      //   msf_source = "odometry";
      //   msf_delta = tf2isometry(transform).cast<float>();
      // }
    // }

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned, prev_trans * init_guess);

    // publish_scan_matching_status(stamp, cloud->header.frame_id, aligned, msf_source, msf_delta);

    if(!registration->hasConverged()) {
      LOG_WARN("scan matching has not converged, ignore this frame({})", stamp);
      return prev_trans.cast<double>();
    }

    Eigen::Matrix4f odom = registration->getFinalTransformation();
    Eigen::Matrix4f trans = keyframe_pose.inverse() * odom;

    // if(transform_thresholding) {
    //   Eigen::Matrix4f delta = prev_trans.inverse() * trans;
    //   double dx = delta.block<3, 1>(0, 3).norm();
    //   double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

    //   if(dx > max_acceptable_trans || da > max_acceptable_angle) {
    //     std::cout << "too large transform!!  " << dx << "[m] " << da << "[rad]" << std::endl;
    //     std::cout << "ignore this frame(" << stamp << ")" << std::endl;
    //     return (keyframe_pose * prev_trans).cast<double>();
    //   }
    // }

    prev_trans = odom;

    // auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    // keyframe_broadcaster.sendTransform(keyframe_trans);

    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp) / 1000000.0;

    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
      keyframe = aligned;
      keyframe_pose = odom;
      keyframe_stamp = stamp;
      keyframe_queue.enqueue(new PointCloudType(keyframe));
    }


    // pcl::transformPointCloud (*cloud, *aligned, odom);
    // aligned->header.frame_id=odom_frame_id;
    // aligned_points_pub.publish(*aligned);

    return odom.cast<double>();
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const uint64_t& stamp, const Eigen::Matrix4d& pose) {
    // publish transform stamped for IMU integration
    // geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    // trans_pub.publish(odom_trans);

    // broadcast the transform over tf
    // odom_broadcaster.sendTransform(odom_trans);

    // publish the transform
    // nav_msgs::Odometry odom;
    // odom.header.stamp = stamp;
    // odom.header.frame_id = odom_frame_id;

    // odom.pose.pose.position.x = pose(0, 3);
    // odom.pose.pose.position.y = pose(1, 3);
    // odom.pose.pose.position.z = pose(2, 3);
    // odom.pose.pose.orientation = odom_trans.transform.rotation;

    // odom.child_frame_id = base_frame_id;
    // odom.twist.twist.linear.x = 0.0;
    // odom.twist.twist.linear.y = 0.0;
    // odom.twist.twist.angular.z = 0.0;

    // odom_pub.publish(odom);
  }

  /**
   * @brief publish scan matching status
   */
  void publish_scan_matching_status(const uint64_t& stamp, const std::string& frame_id, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned, const std::string& msf_source, const Eigen::Isometry3f& msf_delta) {
    // ScanMatchingStatus status;
    // status.header.frame_id = frame_id;
    // status.header.stamp = stamp;
    // status.has_converged = registration->hasConverged();
    // status.matching_error = registration->getFitnessScore();

    // const double max_correspondence_dist = 0.5;

    // int num_inliers = 0;
    // std::vector<int> k_indices;
    // std::vector<float> k_sq_dists;
    // for(int i=0; i<aligned->size(); i++) {
    //   const auto& pt = aligned->at(i);
    //   registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
    //   if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
    //     num_inliers++;
    //   }
    // }
    // status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();

    // status.relative_pose = isometry2pose(Eigen::Isometry3f(registration->getFinalTransformation()).cast<double>());

    // if(!msf_source.empty()) {
    //   status.prediction_labels.resize(1);
    //   status.prediction_labels[0].data = msf_source;

    //   status.prediction_errors.resize(1);
    //   Eigen::Isometry3f error = Eigen::Isometry3f(registration->getFinalTransformation()).inverse() * msf_delta;
    //   status.prediction_errors[0] = isometry2pose(error.cast<double>());
    // }

    // status_pub.publish(status);
  }

private:
  std::string points_topic;
  std::string odom_frame_id;
  std::string robot_odom_frame_id;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  uint64_t prev_time;
  Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;               // keyframe pose
  uint64_t keyframe_stamp;                    // keyframe time
  pcl::PointCloud<PointT>::Ptr keyframe;  // keyframe point cloud
  RWQueue<PointCloudType*> keyframe_queue;

  bool thread_start;
  std::unique_ptr<std::thread> map_update_thread{nullptr};

  //
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::Registration<PointT, PointT>::Ptr registrations[2];
  RWQueue<RegistrationType*> registration_queue;
  std::atomic<int> pingpong_idx;
  std::atomic<int> last_pingpong_idx;
};

}  // namespace hdl_graph_slam

hdl_graph_slam::ScanMatchingOdometryNodelet scanMatchNode;

void init_scan_match_node(InitParameter &param) {
  scanMatchNode.onInit(param);
}

void deinit_scan_match_node() {
  scanMatchNode.deInit();
}

Eigen::Matrix4d enqueue_scan_match(PointCloud::Ptr &points, Eigen::Matrix4d& init_guess) {
  return scanMatchNode.cloud_callback(points, init_guess);
}