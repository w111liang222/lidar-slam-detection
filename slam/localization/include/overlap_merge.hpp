// SPDX-License-Identifier: BSD-2-Clause

#ifndef OVERLAP_MERGE_HPP
#define OVERLAP_MERGE_HPP

#include <queue>
#include <hdl_graph_slam/registrations.hpp>

#include "slam_base.h"
#include "backend_api.h"
#include "Logger.h"

struct Overlap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Overlap>;

  Overlap(const std::shared_ptr<KeyFrame>& key1, const std::shared_ptr<KeyFrame>& key2, const Eigen::Matrix4d& relpose, const double& score) : key1(key1), key2(key2), relative_pose(relpose), score(score) {}

public:
  std::shared_ptr<KeyFrame> key1;
  std::shared_ptr<KeyFrame> key2;
  Eigen::Isometry3d relative_pose;
  double score;
};

/**
 * @brief this class finds overlap by scan matching and adds them to the pose graph
 */
class OverlapDetector {
public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief constructor
   */
  OverlapDetector() {
    distance_thresh = 30.0;
    candidate_link_dist = 10;
    max_candidate_num = 3;
    fitness_score_max_range = 25.0; // 5m
    fitness_score_thresh = 1.5;

#ifdef HAVE_CUDA_ENABLE
    registration = hdl_graph_slam::select_registration_method("NDT_CUDA");
#else
    registration = hdl_graph_slam::select_registration_method("FAST_VGICP");
#endif
    registration_accurate = hdl_graph_slam::select_registration_method("FAST_GICP");
    registration_accurate->setMaxCorrespondenceDistance(0.5);
    registration_accurate->setTransformationEpsilon(0.001);
  }

  std::vector<Overlap::Ptr> detect(const std::vector<std::shared_ptr<KeyFrame>>& keyframes, const std::vector<std::shared_ptr<KeyFrame>>& new_keyframes) {
    std::vector<EdgeType> connection;
    graph_get_edges(connection);

    // build source keyframe pose KDTree
    pose_kdtree = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pose_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pose_cloud->width = static_cast<int>(keyframes.size());
    pose_cloud->height = 1;
    pose_cloud->points.resize(pose_cloud->width * pose_cloud->height);
    for (size_t i = 0; i < keyframes.size(); i++) {
      pose_cloud->points[i].x = keyframes[i]->mOdom(0, 3);
      pose_cloud->points[i].y = keyframes[i]->mOdom(1, 3);
      pose_cloud->points[i].z = keyframes[i]->mOdom(2, 3);
    }
    pose_kdtree->setInputCloud(pose_cloud);

    // build connection map
    connection_map = std::map<int, std::set<int>>();
    for(size_t i = 0; i < connection.size(); i++) {
      if (connection_map.find(connection[i].prev) == connection_map.end()) {
        connection_map[connection[i].prev] = std::set<int>();
      }
      if (connection_map.find(connection[i].next) == connection_map.end()) {
        connection_map[connection[i].next] = std::set<int>();
      }
      connection_map[connection[i].prev].insert(connection[i].next);
      connection_map[connection[i].next].insert(connection[i].prev);
    }

    // build keyframe id map
    keyframe_id_map = std::map<int, int>();
    for (size_t i = 0; i < keyframes.size(); i++) {
      keyframe_id_map[keyframes[i]->mId] = i;
    }

    std::vector<Overlap::Ptr> detected_overlaps;
    for(size_t i = 0; i < new_keyframes.size(); i++) {
      LOG_INFO("overlap detection, progress: {}/{}", i, new_keyframes.size());
      auto candidates = find_candidates(keyframes, new_keyframes[i]);
      auto overlap = matching(keyframes, candidates, new_keyframes[i]);
      if(overlap) {
        detected_overlaps.push_back(overlap);
      }
    }

    return detected_overlaps;
  }

private:
  std::vector<std::shared_ptr<KeyFrame>> find_candidates(const std::vector<std::shared_ptr<KeyFrame>>& keyframes, const std::shared_ptr<KeyFrame>& new_keyframe) {
    std::vector<std::shared_ptr<KeyFrame>> candidates;

    pcl::PointXYZ search_point;
    search_point.x = new_keyframe->mOdom(0, 3);
    search_point.y = new_keyframe->mOdom(1, 3);
    search_point.z = new_keyframe->mOdom(2, 3);
    std::vector<int> pointIdx(10);
    std::vector<float> pointDistance(10);
    if (pose_kdtree->nearestKSearch(search_point, 10, pointIdx, pointDistance) > 0) {
      for (size_t i = 0; i < pointIdx.size(); i++) {
        // same keyframe
        if (new_keyframe->mId == keyframes[pointIdx[i]]->mId) {
          continue;
        }
        // already connection
        if (connection_map[new_keyframe->mId].count(keyframes[pointIdx[i]]->mId) != 0) {
          continue;
        }
        if (pointDistance[i] < (distance_thresh * distance_thresh)) {
          int connection_count = get_connection_count(connection_map, new_keyframe->mId, keyframes[pointIdx[i]]->mId, candidate_link_dist);
          if (connection_count >= candidate_link_dist) {
            candidates.push_back(keyframes[pointIdx[i]]);
          }
        }
        if (candidates.size() >= max_candidate_num) {
          break;
        }
      }
    }

    return candidates;
  }

  Overlap::Ptr matching(const std::vector<std::shared_ptr<KeyFrame>>& keyframes, const std::vector<std::shared_ptr<KeyFrame>>& candidate_keyframes, const std::shared_ptr<KeyFrame>& new_keyframe) {
    if(candidate_keyframes.empty()) {
      return nullptr;
    }

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    double best_score = std::numeric_limits<double>::max();
    std::shared_ptr<KeyFrame> best_matched;
    Eigen::Matrix4f relative_pose;

    registration->setInputTarget(new_keyframe->mPoints);
    for(const auto& candidate : candidate_keyframes) {
      Eigen::Isometry3d new_keyframe_estimate = new_keyframe->mOdom;
      new_keyframe_estimate.linear() = Eigen::Quaterniond(new_keyframe_estimate.linear()).normalized().toRotationMatrix();
      Eigen::Isometry3d candidate_estimate = candidate->mOdom;
      candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear()).normalized().toRotationMatrix();
      Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate).matrix().cast<float>();
      // guess(2, 3) = 0.0;
      registration->setInputSource(candidate->mPoints);
      registration->align(*aligned, guess);

      if(!registration->hasConverged()) {
        continue;
      }
      double score = registration->getFitnessScore(fitness_score_max_range);
      if(score > best_score) {
        continue;
      }

      best_score = score;
      best_matched = candidate;
      relative_pose = registration->getFinalTransformation();
    }

    if(!best_matched.get()){
        LOG_INFO("best_matched is null!");
        return nullptr;
    }

    // finetune
    pcl::PointCloud<PointT>::Ptr accum_points(new pcl::PointCloud<PointT>());
    *accum_points += (*best_matched->mPoints);
    Eigen::Isometry3d reference_estimate = best_matched->mOdom;
    reference_estimate.linear() = Eigen::Quaterniond(reference_estimate.linear()).normalized().toRotationMatrix();
    for(auto &conn : connection_map[best_matched->mId]) {
      const std::shared_ptr<KeyFrame> &relative_frame = keyframes[keyframe_id_map[conn]];
      Eigen::Isometry3d relative_estimate = relative_frame->mOdom;
      relative_estimate.linear() = Eigen::Quaterniond(relative_estimate.linear()).normalized().toRotationMatrix();
      Eigen::Matrix4d relative = (reference_estimate.inverse() * relative_estimate).matrix();
      pcl::PointCloud<PointT>::Ptr retive_points(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*relative_frame->mPoints, *retive_points, relative);
      *accum_points += (*retive_points);
    }

    registration_accurate->setInputTarget(accum_points);
    registration_accurate->setInputSource(new_keyframe->mPoints);
    registration_accurate->align(*aligned, Eigen::Isometry3f(relative_pose).inverse().matrix());

    if(!registration_accurate->hasConverged()) {
      return nullptr;
    }
    relative_pose = registration_accurate->getFinalTransformation();
    best_score = calc_fitness_score(accum_points, new_keyframe->mPoints, relative_pose, fitness_score_max_range);
    if(best_score > fitness_score_thresh) {
      return nullptr;
    }

    LOG_INFO("overlap found, frame: {} <-> {}, score: {}", new_keyframe->mId, best_matched->mId, best_score);
    return std::make_shared<Overlap>(best_matched, new_keyframe, relative_pose.cast<double>(), best_score);
  }

  void filter(const pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::PointCloud<PointT>& filtered, float max_range, float floor_height) {
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered.points), [&](const Point& p) {
        float dist = std::sqrt(p.x * p.x + p.y * p.y);
        return (dist <  max_range && p.z > floor_height);
    });

    filtered.width = filtered.size();
    filtered.height = 1;
    filtered.is_dense = false;
    filtered.header = cloud->header;
  }

  double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Matrix4f& relpose, double max_range) {
    const double pointcloud_range = 100.0; // TODO: read from keyframe config
    const float  pointcloud_min_z = 0.5;

    pcl::PointCloud<PointT>::Ptr target_transformed(new pcl::PointCloud<PointT>());
    filter(cloud1, *target_transformed, pointcloud_range, pointcloud_min_z);
    pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
    tree_->setInputCloud(target_transformed);

    double fitness_score = 0.0;

    // Transform the input dataset using the final transformation
    pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT> input_transformed;
    pcl::transformPointCloud(*cloud2, *transformed, relpose);
    filter(transformed, input_transformed, pointcloud_range, pointcloud_min_z);

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    // For each point in the source dataset
    int nr = 0;
    for(size_t i = 0; i < input_transformed.points.size(); ++i) {
      // Find its nearest neighbor in the target
      tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

      // Deal with occlusions (incomplete targets)
      if(nn_dists[0] <= max_range) {
        // Add to the fitness score
        fitness_score += nn_dists[0];
        nr++;
      }
    }

    if(nr > 0)
      return (fitness_score / nr);
    else
      return (std::numeric_limits<double>::max());
  }

  int get_connection_count(std::map<int, std::set<int>>& connection, int source, int target, int max_count) {
    int count = 0;

    std::map<int, bool> visited;
    for (auto &conn : connection) {
      visited[conn.first] = false;
    }

    std::queue<int> q;
    q.push(source);
    while (!q.empty()) {
      int size = q.size();
      for (int i = 0; i < size; i++) {
        int curr = q.front();
        q.pop();
        if (curr == target) {
          return count;
        }
        visited[curr] = true;
        for (auto &adjacent : connection[curr]) {
          if (!visited[adjacent]) {
            q.push(adjacent);
          }
        }
      }
      count++;
      if (max_count > 0 && count >= max_count) {
        break;
      }
    }
    return count;
  }

private:
  double distance_thresh;
  int    candidate_link_dist;      // edges between two candidates
  int    max_candidate_num;
  double fitness_score_max_range;  // maximum allowable distance between corresponding points
  double fitness_score_thresh;     // threshold for scan matching

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pose_kdtree;
  std::map<int, std::set<int>> connection_map;
  std::map<int, int> keyframe_id_map;
  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::Registration<PointT, PointT>::Ptr registration_accurate;
};

#endif  // OVERLAP_MERGE_HPP
