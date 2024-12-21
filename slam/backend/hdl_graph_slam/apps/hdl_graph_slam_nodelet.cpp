// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>

#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_se3_gnss.hpp>

#include "slam_base.h"
#include "mapping_types.h"
#include "UTMProjector.h"

#define ENABLE_ROBUST_GRAPH_OPTIMIZATION

namespace hdl_graph_slam {

const double ODOMETRY_LOCAL_MAP_DIST = 2.0;
const int ODOMETRY_LOCAL_MAP_NUM     = 100000;
const int NORMAL_VERTEX_ID_OFFSET    = 100000;
const int MOMENT_VERTEX_ID_OFFSET    = 1000000;

class HdlGraphSlamNodelet {
public:
  typedef pcl::PointXYZI PointT;

  HdlGraphSlamNodelet() {}
  virtual ~HdlGraphSlamNodelet() {}

  virtual void onInit(InitParameter &param) {
    LOG_INFO("initializing hdl graph nodelet...");
    init_config = param;

    // keyframe selector
    best_score = std::numeric_limits<double>::max();
    best_inlier_ratio = 0.0;
    average_score = 0.0;
    map_keyframe_count = 0;
    map_keyframe_step = std::max(1, int(std::round(ODOMETRY_LOCAL_MAP_DIST / param.key_frame_distance)));
    best_frame = PointCloudAttrImagePose();
    best_frame_cloud = PointCloud::Ptr(new PointCloud());
    odom_local_map = PointCloud::Ptr(new PointCloud());

    trans_odom2map.setIdentity();
    keyframe_queue.clear();
    floor_coeffs_queue.clear();
    gps_queue.clear();
    imu_queue.clear();
    keyframes_snapshot.clear();
    new_keyframes.clear();

    //
    fix_first_node = true;
    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    floor_enable = false;
    loop_enable = false;
    zero_utm = boost::optional<Eigen::Vector3d>();

    keyframes.clear();
    keyframe_hash.clear();

    graph_slam.reset(new GraphSLAM("lm_var"));
    keyframe_updater.reset(new KeyframeUpdater(param.key_frame_distance, param.key_frame_degree));
    loop_detector.reset(new LoopDetector());
    projector.reset(new UTMProjector());
    inf_calclator.reset(new InformationMatrixCalculator());

    gps_last_update_pose << 0, 0, 0;
    floor_edge_stddev = 10.0;
    floor_node_id = NORMAL_VERTEX_ID_OFFSET; // floor node id start from 100000 -
    floor_height = 0;
    floor_last_update_distance = 0;

    graph_updated = false;
    graph_loop_detected = false;
    double graph_update_interval = 3.0;
    double map_cloud_update_interval = 10.0;

    auto voxelgrid = new pcl::VoxelGrid<PointT>();
    voxelgrid->setLeafSize(param.resolution, param.resolution, param.resolution);
    downsample_filter.reset(voxelgrid);
  }

  virtual void deInit() {
    LOG_INFO("releasing hdl graph nodelet...");
    best_frame = PointCloudAttrImagePose();
    best_frame_cloud = nullptr;
    odom_local_map = nullptr;

    keyframe_queue.clear();
    floor_coeffs_queue.clear();
    gps_queue.clear();
    imu_queue.clear();
    keyframes_snapshot.clear();
    new_keyframes.clear();

    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;

    keyframes.clear();
    keyframe_hash.clear();

    graph_slam = nullptr;
    keyframe_updater = nullptr;
    loop_detector = nullptr;
    projector = nullptr;
    inf_calclator = nullptr;

    downsample_filter = nullptr;
  }

  void set_origin(RTKType &rtk) {
    if (!keyframes.empty()) {
      rtk.altitude -= keyframes.back()->node->estimate()(2, 3);
    }

    double easting, northing, altitude;
    projector->FromGlobalToLocal(rtk.latitude, rtk.longitude, easting, northing);
    altitude = rtk.altitude;
    Eigen::Vector3d xyz(easting, northing, altitude);
    zero_utm = xyz;
    LOG_INFO("Graph: origin is set to  \n \
              lat: {}, lon: {}, alt: {}\n \
              x  : {}, y  : {}, z  : {}",
              rtk.latitude, rtk.longitude, rtk.altitude,
              easting, northing, altitude);
  }

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  bool cloud_callback(PointCloudAttrImagePose &frame, PointCloudAttrImagePose &keyframe) {
    Eigen::Isometry3d odom = frame.T;
    if (frame.points->cloud->empty()) {
      return false;
    }

    if (keyframe_updater->is_first_frame()) {
      keyframe_updater->update(odom);
      frame.points->cloud = downsample(frame.points->cloud);
      // frame.points->cloud = filter(frame.points->cloud, 0.5);
      pcl::transformPointCloud(*frame.points->cloud, *odom_local_map, odom.matrix());
      InformationMatrixCalculator::rebuild_kd_tree(odom_local_map);
      return false;
    }

    float dx, da;
    bool need_update, must_update;
    keyframe_updater->is_update(odom, need_update, must_update, dx, da);
    if (!need_update) {
      return false;
    }

    // undistortion pointcloud
    if (frame.imu_poses.empty()) {
      Eigen::Matrix4f delta_odom = (frame.points->T).cast<float>();
      undistortPoints(delta_odom, frame.points, init_config.scan_period);
    } else{
      undistortPoints(frame.imu_poses, frame.points);
    }

    // downsample pointcloud
    frame.points->cloud = downsample(frame.points->cloud);

    // filter the ground points
    // PointCloud::Ptr frame_cloud = filter(frame.points->cloud, 0.5);
	PointCloud::Ptr frame_cloud = frame.points->cloud;

    // calculate fitness score and find best frame
    int nr = 0;
    double fitness_score = InformationMatrixCalculator::calc_fitness_score(odom_local_map, frame_cloud, odom, nr, 1.0);
    float inlier_ratio = float(nr) / frame_cloud->points.size();
    if ((fitness_score * 0.8 + (1.0 - inlier_ratio) * 0.2)  <= (best_score * 0.8 + (1.0 - best_inlier_ratio) * 0.2)) {
      best_score = fitness_score;
      best_inlier_ratio = inlier_ratio;
      best_frame = frame;
      best_frame_cloud = frame_cloud;
    }

    if (must_update) {
      LOG_INFO("map update, average score {}, best score {}, inlier ratio {}, dx {}, da {}", average_score, best_score, best_inlier_ratio, dx, da);
      keyframe_updater->update(odom);
      double accum_d = keyframe_updater->get_accum_distance();
      KeyFrame::Ptr keyframe_ptr(new KeyFrame(best_frame.points->cloud->header.stamp, best_frame.T, accum_d, best_frame.points->cloud));
      {
        std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
        keyframe_queue.push_back(keyframe_ptr);
      }

      // update mean score
      map_keyframe_count += 1;
      average_score = (average_score * (map_keyframe_count - 1) + best_score) / map_keyframe_count;
      best_score = std::numeric_limits<double>::max();

      // update odometry local map
      if ((map_keyframe_count % map_keyframe_step) == 0) {
        PointCloud::Ptr best_align_frame(new PointCloud());
        pcl::transformPointCloud(*best_frame_cloud , *best_align_frame, best_frame.T.matrix());
        *odom_local_map += *best_align_frame;
        if (odom_local_map->points.size() > ODOMETRY_LOCAL_MAP_NUM) {
          odom_local_map->erase(odom_local_map->begin(), odom_local_map->begin() + (odom_local_map->points.size() - ODOMETRY_LOCAL_MAP_NUM));
        }
        InformationMatrixCalculator::rebuild_kd_tree(odom_local_map);
      }

      // assign the best frame to keyframe
      trans_odom2map_mutex.lock();
      Eigen::Isometry3d odometry2map = Eigen::Isometry3d(trans_odom2map.cast<double>());
      trans_odom2map_mutex.unlock();
      keyframe = best_frame;
      keyframe.T = odometry2map * keyframe.T;
    }

    return must_update;
  }

  void wait_for_flush() {
    // LOG_INFO("start wait for flush keyframe queue");
    // wait for flush keyframe queue
    while(true) {
      keyframe_queue_mutex.lock();
      if(keyframe_queue.empty()) {
        keyframe_queue_mutex.unlock();
        break;
      }
      keyframe_queue_mutex.unlock();
      usleep(100000);
    }

    // wait for flush new_keyframes
    while(true) {
      main_thread_mutex.lock();
      if(new_keyframes.empty()) {
        main_thread_mutex.unlock();
        break;
      }
      main_thread_mutex.unlock();
      usleep(100000);
    }

    // LOG_INFO("flush keyframe queue done");
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    keyframe_queue_mutex.lock();
    std::deque<KeyFrame::Ptr> keyframe_queue_snapshot = keyframe_queue;
    keyframe_queue_mutex.unlock();
    if(keyframe_queue_snapshot.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    auto clock = std::chrono::steady_clock::now();
    int num_processed = 0;
    for(int i = 0; i < keyframe_queue_snapshot.size(); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue_snapshot[i];
      // new_keyframes will be tested later for loop closure
      new_keyframes.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      // fix the first node
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(fix_first_node) {
          keyframe->node->setFixed(true);
      //     Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
      //     std::stringstream sst("1 1 1 1 1 1");
      //     for(int i = 0; i < 6; i++) {
      //       double stddev = 1.0;
      //       sst >> stddev;
      //       inf(i, i) = 1.0 / stddev;
      //     }

      //     anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
      //     anchor_node->setFixed(true);
      //     anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
        }
      }

      if(i == 0 && keyframes.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue_snapshot[i - 1];

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      graph_slam->add_robust_kernel(edge, "NONE", 1.0);
    }

    keyframe_queue_mutex.lock();
    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
    keyframe_queue_mutex.unlock();

    auto elapseMs = since(clock).count();
    LOG_INFO("Graph: flush keyframe queue costs {} ms", elapseMs);
    return true;
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(std::shared_ptr<RTKType>& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > gps_queue.back()->timestamp) {
        break;
      }

      if(keyframe->stamp < (*gps_cursor)->timestamp || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      auto first_gps = gps_cursor;
      auto second_gps = gps_cursor;
      for(; second_gps != gps_queue.end(); second_gps++) {
        auto dt = ((*first_gps)->timestamp - double(keyframe->stamp)) / 1000000.0;
        auto dt2 = ((*second_gps)->timestamp - double(keyframe->stamp)) / 1000000.0;
        if(dt <= 0 && dt2 >= 0 && (dt2 - dt) < 2.0) {
          break;
        }
        first_gps = second_gps;
      }
      if (second_gps == gps_queue.end()) {
        break;
      }
      gps_cursor = first_gps;

      if ((*first_gps)->precision != (*second_gps)->precision) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      int gps_dim = (*first_gps)->dimension;
      double gps_xyz_stddev = (*first_gps)->precision;
      double gps_rot_stddev = (*first_gps)->precision * 10.0;
      Eigen::Matrix4d utm_transform = interpolated_transform(*first_gps, *second_gps, keyframe->stamp);
      Eigen::Vector3d xyz = utm_transform.block<3, 1>(0, 3);
      xyz[2] = (gps_dim == 2) ? 0 : xyz[2];

      double update_threshold;
      if ((*first_gps)->precision < 100.0) {
        update_threshold = 10.0;
      } else if ((*first_gps)->precision < 1000.0) {
        update_threshold = 20.0;
      } else {
        update_threshold = 50.0;
      }

      if (gps_last_update_pose.norm() > 0 && (gps_last_update_pose - xyz).norm() < update_threshold) {
        continue;
      }

      keyframe->utm_coord = xyz;
      gps_last_update_pose = xyz;

      g2o::OptimizableGraph::Edge* edge;
      if(gps_dim == 2) {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_xyz_stddev;
        information_matrix(2, 2) = 1.0 / (xyz.norm() * xyz.norm());
        edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      } else {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_xyz_stddev;
        information_matrix(2, 2) /= gps_xyz_stddev;
        edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
        // rotation constraints
        if (gps_dim == 6) {
          Eigen::Quaterniond orientation = Eigen::Quaterniond(utm_transform.block<3, 3>(0, 0));
          orientation.normalize();
          keyframe->orientation = orientation;
          Eigen::MatrixXd rotation_information_matrix = Eigen::MatrixXd::Identity(3, 3) / gps_rot_stddev;
          auto rotation_edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, rotation_information_matrix);
          graph_slam->add_robust_kernel(rotation_edge, "Huber", 1.0);
        }
      }
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      LOG_INFO("Add GNSS constraint, ID: {}, meassurement: ({}, {}, {})", edge->id(), xyz(0), xyz(1), xyz(2));

      updated = true;
    }

    if(updated && fix_first_node) {
      LOG_WARN("Release the first node after GNSS factor received");
      fix_first_node = false;
      if (graph_slam->graph->vertices().find(0) != graph_slam->graph->vertices().end()) {
        auto first_node = dynamic_cast<g2o::VertexSE3*>(graph_slam->graph->vertices()[0]);
        first_node->setFixed(false);
      }
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp, [=](const uint64_t& stamp, const std::shared_ptr<RTKType>& geopoint) { return stamp < geopoint->timestamp; });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }

  void imu_callback(const uint64_t &timestamp, Eigen::Vector3d &gyro, Eigen::Vector3d &acc) {
    return;

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    ImuType imu;
    imu.stamp = timestamp;
    imu.gyr = gyro;
    imu.acc = acc / acc.norm();
    imu_queue.push_back(imu);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);

    if(keyframes.empty() || imu_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > imu_queue.back().stamp) {
        break;
      }

      if(keyframe->stamp < imu_cursor->stamp || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt = (closest_imu->stamp - double(keyframe->stamp)) / 1000000.0;
        auto dt2 = (imu->stamp - double(keyframe->stamp)) / 1000000.0;
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs((closest_imu->stamp - double(keyframe->stamp)) / 1000000.0)) {
        continue;
      }

      keyframe->acceleration = closest_imu->acc;
      updated = true;
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const uint64_t& stamp, const ImuType& imu) { return stamp < imu.stamp; });
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(FloorCoeffs& floor_coeffs_msg) {
    if(floor_coeffs_msg.coeffs.empty() || !floor_enable) {
      return;
    }

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() {
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if(keyframes.empty()) {
      return false;
    }

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue) {
      if(floor_coeffs.timestamp > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash.find(floor_coeffs.timestamp);
      if(found == keyframe_hash.end()) {
        continue;
      }

      if (floor_plane_node && (found->second->accum_distance - floor_last_update_distance) < 100.0) {
        continue;
      }

      if(!floor_plane_node) {
        floor_height = fix_first_node ? 0 : found->second->node->estimate()(2, 3);
        floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, -floor_height), floor_node_id);
        floor_plane_node->setFixed(true);
        LOG_INFO("Add Floor plane node, height: {}", floor_height);
      }

      const auto& keyframe = found->second;
      floor_last_update_distance = keyframe->accum_distance;

      Eigen::Vector4d coeffs(floor_coeffs.coeffs[0], floor_coeffs.coeffs[1], floor_coeffs.coeffs[2], floor_coeffs.coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      LOG_INFO("Add Floor plane edge, {}, {}, {}, {}", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    if(updated && fix_first_node) {
      LOG_INFO("Add Floor constraint, setFix to false for first node");
      fix_first_node = false;
      if (graph_slam->graph->vertices().find(0) != graph_slam->graph->vertices().end()) {
        auto first_node = dynamic_cast<g2o::VertexSE3*>(graph_slam->graph->vertices()[0]);
        first_node->setFixed(false);
      }
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp, [=](const uint64_t& stamp, const FloorCoeffs& coeffs) { return stamp < coeffs.timestamp; });
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(bool &is_running) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() & !flush_imu_queue()) {
      return;
    }

    // loop detection
    std::vector<Loop::Ptr> loops;
    if (loop_enable) {
      loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam, is_running);
    }
    for(const auto& loop : loops) {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
      auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
      graph_slam->add_robust_kernel(edge, "Huber", 1.0);
      graph_loop_detected = true;
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // move the first node anchor position to the current estimate of the first node pose
    // so the first node moves freely while trying to stay around the origin
    if(anchor_node && true) {
      Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
      anchor_node->setEstimate(anchor_target);
    }

    // optimize the pose graph
    int num_iterations = 1024;
    graph_slam->optimize(num_iterations);

    // publish tf
    const auto& keyframe = keyframes.back();
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
    graph_updated = true;
  }

  Eigen::Matrix4d interpolated_transform(std::shared_ptr<RTKType> &pre, std::shared_ptr<RTKType> &next, uint64_t timestamp) {
    double t_diff_ratio   = static_cast<double>(timestamp - pre->timestamp) /
                            static_cast<double>(next->timestamp - pre->timestamp + 1);
    Eigen::Matrix4d preT  = computeRTKTransform(*projector, pre,  *zero_utm);
    Eigen::Matrix4d nextT = computeRTKTransform(*projector, next, *zero_utm);

    return interpolateTransform(preT, nextT, t_diff_ratio);
  }

  pcl::PointCloud<PointT>::Ptr downsample(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  pcl::PointCloud<PointT>::Ptr filter(const pcl::PointCloud<PointT>::Ptr& cloud, float floor_height) {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const Point& p) {
        return (p.z > floor_height);
    });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;
    filtered->header = cloud->header;
    return filtered;
  }

  InitParameter init_config;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;

  // keyframe selector
  double best_score;
  float  best_inlier_ratio;
  double average_score = 0.0;
  int    map_keyframe_count = 0;
  int    map_keyframe_step = 0;
  PointCloudAttrImagePose best_frame;
  PointCloud::Ptr best_frame_cloud;
  PointCloud::Ptr odom_local_map;

  // keyframe queue
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  boost::optional<Eigen::Vector3d> zero_utm;
  Eigen::Vector3d gps_last_update_pose;
  std::mutex gps_queue_mutex;
  std::deque<std::shared_ptr<RTKType>> gps_queue;

  // imu queue
  std::mutex imu_queue_mutex;
  std::deque<ImuType> imu_queue;

  // floor_coeffs queue
  double floor_edge_stddev;
  bool floor_enable;
  double floor_height;
  double floor_last_update_distance;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<FloorCoeffs> floor_coeffs_queue;
  int floor_node_id;

  // for map cloud generation
  std::atomic_bool graph_updated;
  std::atomic_bool graph_loop_detected;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  bool fix_first_node;
  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<uint64_t, KeyFrame::Ptr> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  bool loop_enable;

  std::unique_ptr<UTMProjector> projector;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  pcl::Filter<PointT>::Ptr downsample_filter;
};

}  // namespace hdl_graph_slam

hdl_graph_slam::HdlGraphSlamNodelet graphNode;

void init_graph_node(InitParameter &param) {
  std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);
  graphNode.onInit(param);
}

void deinit_graph_node() {
  std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);
  graphNode.deInit();
}

void graph_set_origin(RTKType &rtk) {
  graphNode.set_origin(rtk);
}

void set_ground_constaint(bool enable) {
  std::lock_guard<std::mutex> lock(graphNode.floor_coeffs_queue_mutex);
  graphNode.floor_enable = enable;
  graphNode.floor_coeffs_queue.clear();
  if(graphNode.floor_plane_node) {
    graphNode.floor_plane_node = nullptr;
    graphNode.floor_node_id++;
  }
}

bool get_ground_constaint() {
  return graphNode.floor_enable;
}

void set_constraint(bool loop_closure, bool gravity_constraint) {
  std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);
  graphNode.loop_enable = loop_closure;
}

bool enqueue_graph(PointCloudAttrImagePose &frame, PointCloudAttrImagePose &keyframe) {
  return graphNode.cloud_callback(frame, keyframe);
}

void enqueue_graph_floor(FloorCoeffs& floor_coeffs) {
  graphNode.floor_coeffs_callback(floor_coeffs);
}

void enqueue_graph_gps(bool rtk_valid, std::shared_ptr<RTKType>& rtk) {
  if (rtk_valid) {
    graphNode.gps_callback(rtk);
  }
}

void enqueue_graph_imu(const uint64_t &timestamp, Eigen::Vector3d &gyro, Eigen::Vector3d &acc) {
  graphNode.imu_callback(timestamp, gyro, acc);
}

void graph_optimization(bool &is_running) {
  graphNode.optimization_timer_callback(is_running);
}

Eigen::Isometry3d get_odom2map() {
  graphNode.trans_odom2map_mutex.lock();
  Eigen::Isometry3d odom2map(graphNode.trans_odom2map.cast<double>());
  graphNode.trans_odom2map_mutex.unlock();
  return odom2map;
}

bool graph_update_odom(std::map<int, Eigen::Matrix4d> &odoms) {
  if (graphNode.graph_updated) {
    graphNode.keyframes_snapshot_mutex.lock();
    for (auto &keyframe : graphNode.keyframes_snapshot) {
      odoms[keyframe->id] = keyframe->pose.matrix();
    }
    graphNode.keyframes_snapshot_mutex.unlock();
    graphNode.graph_updated = false;
    return true;
  } else {
    return false;
  }
}

void graph_get_status(bool &loop_detected) {
  loop_detected = graphNode.graph_loop_detected;

  // reset status
  graphNode.graph_loop_detected = false;
}

void graph_get_edges(std::vector<EdgeType> &edges) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }

  graphNode.wait_for_flush();
  for(const auto& edge : graphNode.graph_slam->graph->edges()) {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(edge);
    if (e == nullptr) {
      continue;
    }
    std::vector<g2o::HyperGraph::Vertex*> &vertices = e->vertices();
    EdgeType et;
    et.id   = e->id();
    et.prev = vertices[0]->id();
    et.next = vertices[1]->id();
    edges.push_back(et);
  }
}

void graph_add_vertex(int id) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }
}

void graph_del_vertex(int id) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }

  graphNode.wait_for_flush();
  if(graphNode.graph_slam->graph->vertices().find(id) == graphNode.graph_slam->graph->vertices().end()) {
    LOG_WARN("Vertex ID {} does not exist!!", id);
    return;
  }

  g2o::VertexSE3* node = dynamic_cast<g2o::VertexSE3*>(graphNode.graph_slam->graph->vertices()[id]);

  std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);
  for (auto it = graphNode.keyframes.begin(); it != graphNode.keyframes.end(); it++) {
    if ((*it)->node->id() == id) {
      graphNode.keyframe_hash.erase((*it)->stamp);
      graphNode.keyframes.erase(it);
      break;
    }
  }
  graphNode.graph_slam->del_se3_node(node);
  graphNode.graph_updated = false;
}

void graph_add_edge(const PointCloud::Ptr& prev, int prev_id,
                    const PointCloud::Ptr& next, int next_id,
                    Eigen::Isometry3d &relative_pose,
                    double score) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }

  graphNode.wait_for_flush();
  Eigen::MatrixXd information;
  if (score <= 0.0) {
    information = graphNode.inf_calclator->calc_information_matrix(prev, next, relative_pose);
  } else {
    information = graphNode.inf_calclator->calc_information_matrix(score);
  }
  g2o::VertexSE3* prev_node = dynamic_cast<g2o::VertexSE3*>(graphNode.graph_slam->graph->vertices()[prev_id]);
  g2o::VertexSE3* next_node = dynamic_cast<g2o::VertexSE3*>(graphNode.graph_slam->graph->vertices()[next_id]);
  auto edge = graphNode.graph_slam->add_se3_edge(prev_node, next_node, relative_pose, information);
  graphNode.graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

void graph_del_edge(int id) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }

  graphNode.wait_for_flush();
  for(const auto& edge : graphNode.graph_slam->graph->edges()) {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(edge);
    if (e == nullptr || e->id() != id) {
      continue;
    }
    graphNode.graph_slam->del_se3_edge(e);
    break;
  }
}

void graph_set_vertex_fix(int id, bool fix) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }

  graphNode.wait_for_flush();
  for (auto v : graphNode.graph_slam->graph->vertices()) {
    auto node = dynamic_cast<g2o::VertexSE3*>(v.second);
    if (node == nullptr || node->id() != id) {
      continue;
    }
    node->setFixed(fix);
    break;
  }
}

void graph_get_info(GraphInfo &info) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }

  graphNode.wait_for_flush();
  for (auto v : graphNode.graph_slam->graph->vertices()) {
    auto node = dynamic_cast<g2o::VertexSE3*>(v.second);
    if (node != nullptr) {
      info.vertex[node->id()] = VertexInfo();
      info.vertex[node->id()].fix = node->fixed();
      info.vertex[node->id()].edge_num = node->edges().size();
    }
  }
  for(const auto& edge : graphNode.graph_slam->graph->edges()) {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(edge);
    if (e != nullptr) {
      info.edge[e->id()] = EdgeInfo();
      info.edge[e->id()].vertex_num = e->vertices().size();
    }
  }
}

void graph_optimize() {
  if (graphNode.graph_slam == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);
  int num_iterations = 1024;
  graphNode.graph_slam->optimize(num_iterations);
}

void graph_optimize(std::map<int, Eigen::Matrix4d> &odoms) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);
  int num_iterations = 1024;
  graphNode.graph_slam->optimize(num_iterations);
  for (auto v : graphNode.graph_slam->graph->vertices()) {
    auto node = dynamic_cast<g2o::VertexSE3*>(v.second);
    if (node != nullptr) {
      odoms[node->id()] = node->estimate().matrix();
    }
  }
}

void robust_graph_optimize(std::string mode, std::map<int, Eigen::Matrix4d> &odoms) {
  if (graphNode.graph_slam == nullptr) {
    return;
  }

  LOG_INFO("Robust pose graph optimization, SLAM mode: {}", mode);

#ifdef ENABLE_ROBUST_GRAPH_OPTIMIZATION
  {
    std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);

    LOG_INFO("GNSS outlier detection and removal...");
    // use DCS to identify the GNSS outlier
    const double max_distance_error2 = 1.0 * 1.0; // max error: 1.0 m
    for(const auto& edge_ : graphNode.graph_slam->graph->edges()) {
      g2o::EdgeSE3PriorXYZ* edge = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge_);
      if (edge != nullptr) {
        Eigen::MatrixXd information = edge->information();
        graphNode.graph_slam->add_robust_kernel(edge, "DCS2", max_distance_error2 * information(0, 0));
      }
    }

    // optimize
    int num_iterations = 1024;
    graphNode.graph_slam->optimize(num_iterations);

    // check the loss scale
    std::vector<g2o::EdgeSE3PriorXYZ*> outlier_edges;
    for(const auto& edge_ : graphNode.graph_slam->graph->edges()) {
      g2o::EdgeSE3PriorXYZ* edge = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge_);
      if (edge != nullptr) {
        Eigen::MatrixXd information = edge->information();
        double phi = max_distance_error2 * information(0, 0);
        double scale = (2.0 * phi) / (phi + edge->chi2());
        if (scale < 0.1) {
          outlier_edges.push_back(edge);
        }
      }
    }
    for (auto &edge : outlier_edges) {
      LOG_WARN("GNSS edge({}) is detected as an outlier", edge->id());
      graphNode.graph_slam->graph->removeEdge(edge);
    }

    if (mode.compare("mapping") == 0) {
      graphNode.graph_slam->optimize(num_iterations);
      LOG_INFO("GNSS moment estimation...");
      // new gnss moment node and prior (0, 0, 0) loose constraint
      Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
      information_matrix.block<2, 2>(0, 0) /= 2000.0;
      information_matrix(2, 2) /= 200.0;
      g2o::VertexPointXYZ* gnss_moment_node = graphNode.graph_slam->add_point_xyz_node(Eigen::Vector3d(0.0, 0.0, 0.0), hdl_graph_slam::MOMENT_VERTEX_ID_OFFSET);
      g2o::EdgeXYZPrior*   gnss_moment_edge = graphNode.graph_slam->add_xyz_prior_edge(gnss_moment_node, Eigen::Vector3d(0.0, 0.0, 0.0), information_matrix, hdl_graph_slam::MOMENT_VERTEX_ID_OFFSET + 1);

      // replace the g2o::EdgeSE3PriorXYZ with g2o::EdgeSE3GNSS*
      std::vector<g2o::EdgeSE3GNSS*> gnss_moment_edges;
      std::vector<g2o::EdgeSE3PriorXYZ*> gnss_prior_edges;
      for(const auto& edge_ : graphNode.graph_slam->graph->edges()) {
        g2o::EdgeSE3PriorXYZ* prior_edge = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge_);
        if (prior_edge != nullptr) {
          g2o::VertexSE3* node = dynamic_cast<g2o::VertexSE3*>(prior_edge->vertices()[0]);
          g2o::EdgeSE3GNSS* moment_edge = graphNode.graph_slam->add_se3_gnss_edge(node, gnss_moment_node, prior_edge->measurement(), prior_edge->information());
          graphNode.graph_slam->add_robust_kernel(moment_edge, "DCS2", max_distance_error2 * prior_edge->information()(0, 0));

          gnss_prior_edges.push_back(prior_edge);
          gnss_moment_edges.push_back(moment_edge);
        }
      }
      for (auto &edge : gnss_prior_edges) {
        graphNode.graph_slam->graph->removeEdge(edge);
      }

      // optimize
      graphNode.graph_slam->optimize(num_iterations);
      LOG_INFO("GNSS moment: ({}, {}, {})", gnss_moment_node->estimate()[0], gnss_moment_node->estimate()[1], gnss_moment_node->estimate()[2]);

      // add the estimation of moment to GNSS measurement
      for (auto &moment_edge : gnss_moment_edges) {
        g2o::VertexSE3* node = dynamic_cast<g2o::VertexSE3*>(moment_edge->vertices()[0]);
        Eigen::Isometry3d estimate = node->estimate();
        Eigen::Vector3d measurement = moment_edge->measurement() + estimate.linear() * gnss_moment_node->estimate();
        g2o::EdgeSE3PriorXYZ* prior_edge = graphNode.graph_slam->add_se3_prior_xyz_edge(node, measurement, moment_edge->information());
        graphNode.graph_slam->add_robust_kernel(prior_edge, "DCS2", max_distance_error2 * moment_edge->information()(0, 0));

        graphNode.graph_slam->graph->removeEdge(moment_edge);
      }
      graphNode.graph_slam->graph->removeEdge(gnss_moment_edge);
      graphNode.graph_slam->graph->removeVertex(gnss_moment_node);
    }
  }
#endif // ENABLE_ROBUST_GRAPH_OPTIMIZATION

  graph_optimize(odoms);
}

std::vector<std::string> graph_save(const std::string& directory) {
  std::vector<std::string> vertexes;
  if (graphNode.graph_slam == nullptr) {
    return vertexes;
  }

  std::lock_guard<std::mutex> lock(graphNode.main_thread_mutex);
  graphNode.graph_slam->save(directory + "/graph.g2o");

  std::ofstream ofs(directory + "/special_nodes.csv");
  ofs << "anchor_node " << (graphNode.anchor_node != nullptr ? graphNode.anchor_node->id() : -1) << std::endl;
  ofs << "anchor_edge " << -1 << std::endl;
  ofs << "floor_node " << (graphNode.floor_plane_node != nullptr ? graphNode.floor_plane_node->id() : -1) << std::endl;

  for (auto v : graphNode.graph_slam->graph->vertices()) {
    auto node = dynamic_cast<g2o::VertexSE3*>(v.second);
    if (node != nullptr) {
      vertexes.push_back(std::to_string(node->id()));
    }
  }

  return vertexes;
}

bool graph_load_impl(hdl_graph_slam::HdlGraphSlamNodelet &graph_, const std::string& directory, std::vector<std::shared_ptr<KeyFrame>> &kfs) {
  if (graph_.graph_slam == nullptr) {
    return false;
  }
  graph_.graph_slam->load(directory + "/graph.g2o");

  int edge_id_gen = 0;
  for(auto& edge : graph_.graph_slam->graph->edges()) {
    edge->setId(edge_id_gen++);
  }
  graph_.graph_slam->max_edge_id = edge_id_gen;

  // check the consistent between graph and keyframes
  std::set<int> keyframe_id_map;
  for (auto &kf : kfs) {
    keyframe_id_map.insert(kf->mId);
    if(graph_.graph_slam->graph->vertices().find(kf->mId) == graph_.graph_slam->graph->vertices().end()) {
      LOG_WARN("Vertex ID {} does not exist in graph.g2o", kf->mId);
      return false;
    }

    auto node = dynamic_cast<g2o::VertexSE3*>(graph_.graph_slam->graph->vertices()[kf->mId]);
    if (node == nullptr) {
      LOG_ERROR("Vertex ID {} is not g2o::VertexSE3 type in graph.g2o", kf->mId);
      return false;
    }
    node->setEstimate(kf->mOdom);
  }

  auto graph_vertices = graph_.graph_slam->graph->vertices();
  for (auto &v : graph_vertices) {
    auto node = dynamic_cast<g2o::VertexSE3*>(v.second);
    if (node != nullptr && keyframe_id_map.find(v.first) == keyframe_id_map.end()) {
      LOG_WARN("Vertex ID {} does not exist in keyframe", v.first);
      graph_.graph_slam->del_se3_node(node);
    }
  }

  // load special nodes from file
  std::ifstream ifs(directory + "/special_nodes.csv");
  if(ifs) {
    while(!ifs.eof()) {
      std::string line;
      std::getline(ifs, line);

      if(line.empty()) {
        continue;
      }

      std::stringstream sst(line);
      std::string tag;
      sst >> tag;

      // load anchor node
      if(tag == "anchor_node") {
        long anchor_node_id = -1;
        sst >> anchor_node_id;

        if(anchor_node_id < 0) {
          continue;
        }

        graph_.anchor_node = dynamic_cast<g2o::VertexSE3*>(graph_.graph_slam->graph->vertex(anchor_node_id));
        if(graph_.anchor_node == nullptr) {
          LOG_ERROR("failed to cast anchor node to VertexSE3!!");
          return false;
        }
        if(graph_.anchor_node->edges().empty()) {
          LOG_ERROR("anchor node is not connected with any edges!!");
          return false;
        }

        graph_.anchor_edge = dynamic_cast<g2o::EdgeSE3*>(*graph_.anchor_node->edges().begin());
        if(graph_.anchor_edge == nullptr) {
          LOG_ERROR("failed to cast anchor edge to EdgeSE3!!");
          return false;
        }

      }
      // load floor node
      else if(tag == "floor_node") {
        long floor_node_id = -1;
        sst >> floor_node_id;

        if(floor_node_id < 0) {
          continue;
        }

        graph_.floor_plane_node = dynamic_cast<g2o::VertexPlane*>(graph_.graph_slam->graph->vertex(floor_node_id));
        if(graph_.floor_plane_node == nullptr) {
          LOG_ERROR("failed to cast floor node to VertexPlane!!");
          return false;
        }
      }
    }
  }

  // compute the current error for calculating the real chi2
  for(auto& edge : graph_.graph_slam->graph->edges()) {
    auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
    e->computeError();
  }

  return true;
}

bool graph_load(const std::string& directory, std::vector<std::shared_ptr<KeyFrame>> &kfs) {
  return graph_load_impl(graphNode, directory, kfs);
}

bool graph_merge(const std::string& directory, std::vector<std::shared_ptr<KeyFrame>> &kfs) {
  hdl_graph_slam::HdlGraphSlamNodelet new_graph;
  InitParameter init_param;
  new_graph.onInit(init_param);
  if (!graph_load_impl(new_graph, directory, kfs)) {
    return false;
  }

  int max_se3_vertex_id = 0;
  int max_normal_vertex_id = hdl_graph_slam::NORMAL_VERTEX_ID_OFFSET;
  int max_edge_id = 0;

  for(const auto& vertex : graphNode.graph_slam->graph->vertices()) {
    g2o::VertexSE3* node = dynamic_cast<g2o::VertexSE3*>(vertex.second);
    if (node != nullptr) {
      max_se3_vertex_id = std::max<int>(max_se3_vertex_id, vertex.first);
    } else {
      max_normal_vertex_id = std::max<int>(max_normal_vertex_id, vertex.first);
    }
  }
  for(const auto& edge : graphNode.graph_slam->graph->edges()) {
    max_edge_id = std::max<long>(max_edge_id, edge->id());
  }
  LOG_INFO("Graph: max_se3_vertex_id {}, max_normal_vertex_id {}, max_edge_id {}", max_se3_vertex_id, max_normal_vertex_id, max_edge_id);

  g2o::Factory* factory = g2o::Factory::instance();
  std::unordered_map<int, g2o::HyperGraph::Vertex*> new_vertices_map;

  if(new_graph.anchor_node) {
    new_graph.graph_slam->graph->removeEdge(new_graph.anchor_edge);
    new_graph.graph_slam->graph->detachVertex(new_graph.anchor_node);
  }

  // clone vertices, increase with the origin order
  for (auto &kf : kfs) {
    g2o::VertexSE3* node = dynamic_cast<g2o::VertexSE3*>(new_graph.graph_slam->graph->vertices()[kf->mId]);
    int new_vertex_id = -1;
    if (node != nullptr) {
      new_vertex_id = ++max_se3_vertex_id;
    } else {
      new_vertex_id = ++max_normal_vertex_id;
    }
    auto v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(new_graph.graph_slam->graph->vertices()[kf->mId]);

    // copy params via g2o::Factory
    std::stringstream sst;
    if(!v->write(sst)) {
      LOG_ERROR("error: failed to write vertex data");
      return false;
    }

    auto new_v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(factory->construct(factory->tag(v)));
    if(!new_v->read(sst)) {
      LOG_ERROR("error: failed to read vertex data");
      return false;
    }
    new_v->setFixed(v->fixed());
    new_v->setId(new_vertex_id);
    graphNode.graph_slam->graph->addVertex(new_v);

    // for remapping
    new_vertices_map[v->id()] = new_v;
  }

  // clone edges
  for(const auto& edge : new_graph.graph_slam->graph->edges()) {
    int new_edge_id = ++max_edge_id;
    auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);

    // copy params via g2o::Factory
    std::stringstream sst;
    if(!e->write(sst)) {
      LOG_ERROR("error: failed to write edge data");
      return false;
    }

    auto new_e = dynamic_cast<g2o::OptimizableGraph::Edge*>(factory->construct(factory->tag(e)));
    if(!new_e->read(sst)) {
      LOG_ERROR("error: failed to read edge data");
      return false;
    }
    new_e->setId(new_edge_id);

    // remap vertices with new ones
    for(int i = 0; i < new_e->vertices().size(); i++) {
      new_e->vertices()[i] = new_vertices_map[e->vertices()[i]->id()];
    }

    // copy robust kernel
    if(e->robustKernel()) {
      g2o::RobustKernel* kernel = nullptr;

      if(dynamic_cast<g2o::RobustKernelHuber*>(e->robustKernel())) {
        kernel = new g2o::RobustKernelHuber();
      }

      if(kernel == nullptr) {
        LOG_WARN("warning: unknown kernel type!!");
      } else {
        kernel->setDelta(e->robustKernel()->delta());
        new_e->setRobustKernel(kernel);
      }
    }

    edge->setId(new_edge_id);
    graphNode.graph_slam->graph->addEdge(new_e);
  }
  graphNode.graph_slam->max_edge_id = max_edge_id + 1;

  for(auto &kf : kfs) {
    kf->mId = new_vertices_map[kf->mId]->id();
  }

  return true;
}

// below functions are only called by "MapLoader"

void graph_sync_pose(std::vector<std::shared_ptr<KeyFrame>> &kfs, int direction) {
  std::map<int, std::shared_ptr<KeyFrame>> kfs_map;
  std::transform(kfs.begin(), kfs.end(), std::inserter(kfs_map, kfs_map.end()),
                 [](const std::shared_ptr<KeyFrame> &kf) { return std::make_pair(kf->mId, kf); });

  for (auto v : graphNode.graph_slam->graph->vertices()) {
    auto node = dynamic_cast<g2o::VertexSE3*>(v.second);
    if (node != nullptr && kfs_map.find(node->id()) != kfs_map.end()) {
      if (direction == 0) {
        kfs_map[node->id()]->mOdom = node->estimate();
      } else if (direction == 1) {
        node->setEstimate(kfs_map[node->id()]->mOdom);
      }
    }
  }
}

void graph_add_prior_edge(int id, Eigen::Isometry3d &prior, double xyz_stddev, double rot_stddev) {
  g2o::VertexSE3* node = dynamic_cast<g2o::VertexSE3*>(graphNode.graph_slam->graph->vertices()[id]);
  // translation
  Eigen::Vector3d xyz = prior.matrix().block<3, 1>(0, 3);
  Eigen::Matrix3d xyz_information_matrix = Eigen::Matrix3d::Identity() / xyz_stddev;
  graphNode.graph_slam->add_se3_prior_xyz_edge(node, xyz, xyz_information_matrix);

  // rotation
  Eigen::Quaterniond orientation = Eigen::Quaterniond(prior.matrix().block<3, 3>(0, 0)).normalized();
  Eigen::MatrixXd rotation_information_matrix = Eigen::MatrixXd::Identity(3, 3) / rot_stddev;
  graphNode.graph_slam->add_se3_prior_quat_edge(node, orientation, rotation_information_matrix);
}

void graph_del_prior_edges() {
  // PriorXYZ
  std::vector<g2o::EdgeSE3PriorXYZ*> xyz_priors;
  for(const auto& edge : graphNode.graph_slam->graph->edges()) {
    g2o::EdgeSE3PriorXYZ* e = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
    if (e != nullptr) {
      xyz_priors.push_back(e);
    }
  }
  for (auto &e : xyz_priors) {
    graphNode.graph_slam->graph->removeEdge(e);
  }

  // PriorQuat
  std::vector<g2o::EdgeSE3PriorQuat*> quat_priors;
  for(const auto& edge : graphNode.graph_slam->graph->edges()) {
    g2o::EdgeSE3PriorQuat* e = dynamic_cast<g2o::EdgeSE3PriorQuat*>(edge);
    if (e != nullptr) {
      quat_priors.push_back(e);
    }
  }
  for (auto &e : quat_priors) {
    graphNode.graph_slam->graph->removeEdge(e);
  }
}

void graph_update_origin(const RTKType &from_origin, const RTKType &to_origin, std::vector<std::shared_ptr<KeyFrame>> &kfs) {
  UTMProjector projector_from, projector_to;
  Eigen::Vector3d zero_utm_from, zero_utm_to;
  projector_from.FromGlobalToLocal(from_origin.latitude, from_origin.longitude, zero_utm_from(0), zero_utm_from(1));
  projector_to.FromGlobalToLocal(to_origin.latitude, to_origin.longitude, zero_utm_to(0), zero_utm_to(1));

  std::map<int, std::shared_ptr<KeyFrame>> kfs_map;
  std::transform(kfs.begin(), kfs.end(), std::inserter(kfs_map, kfs_map.end()),
                 [](const std::shared_ptr<KeyFrame> &kf) { return std::make_pair(kf->mId, kf); });

  for(const auto& edge : graphNode.graph_slam->graph->edges()) {
    g2o::EdgeSE3PriorXYZ* e = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
    if (e != nullptr) {
      g2o::VertexSE3* node = dynamic_cast<g2o::VertexSE3*>(e->vertices()[0]);
      if (kfs_map.find(node->id()) == kfs_map.end()) {
        continue;
      }

      Eigen::Vector3d meassurement = e->measurement();
      // reverse project to LLA by previous origin
      double gnss_lat, gnss_lon;
      projector_from.FromLocalToGlobal(meassurement[0] + zero_utm_from(0), meassurement[1] + zero_utm_from(1), gnss_lat, gnss_lon);

      // project to local by new origin
      double new_local_x, new_local_y;
      projector_to.FromGlobalToLocal(gnss_lat, gnss_lon, new_local_x, new_local_y);
      meassurement[0] = new_local_x - zero_utm_to(0);
      meassurement[1] = new_local_y - zero_utm_to(1);
      meassurement[2] = meassurement[2] + from_origin.altitude - to_origin.altitude;
      e->setMeasurement(meassurement);
    }
  }

  for (const auto& node : graphNode.graph_slam->graph->vertices()) {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(node.second);
    if (v != nullptr) {
      if (kfs_map.find(v->id()) == kfs_map.end()) {
        continue;
      }

      Eigen::Isometry3d estimate = v->estimate();
      // reverse project to LLA by previous origin
      double est_lat, est_lon;
      projector_from.FromLocalToGlobal(estimate(0, 3) + zero_utm_from(0), estimate(1, 3) + zero_utm_from(1), est_lat, est_lon);

      // project to local by new origin
      double new_local_x, new_local_y;
      projector_to.FromGlobalToLocal(est_lat, est_lon, new_local_x, new_local_y);
      estimate(0, 3) = new_local_x - zero_utm_to(0);
      estimate(1, 3) = new_local_y - zero_utm_to(1);
      estimate(2, 3) = estimate(2, 3) + from_origin.altitude - to_origin.altitude;
      v->setEstimate(estimate);
    }
  }

  graph_optimize();
}