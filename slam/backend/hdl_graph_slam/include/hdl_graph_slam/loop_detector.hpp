// SPDX-License-Identifier: BSD-2-Clause

#ifndef LOOP_DETECTOR_HPP
#define LOOP_DETECTOR_HPP

#include <boost/format.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/registrations.hpp>
#include <hdl_graph_slam/graph_slam.hpp>

#include <g2o/types/slam3d/vertex_se3.h>

#include "Logger.h"
#include "SystemUtils.h"

namespace hdl_graph_slam {

struct Loop {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  Loop(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Matrix4f& relpose) : key1(key1), key2(key2), relative_pose(relpose) {}

public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Matrix4f relative_pose;
};

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
public:
  typedef pcl::PointXYZI PointT;

  /**
   * @brief constructor
   * @param pnh
   */
  LoopDetector() {
    distance_thresh = 15.0;
    accum_distance_thresh = 25.0;
    distance_from_last_edge_thresh = 15.0;
    distance_new_keyframe_thresh = 2.0;
    distance_keyframe_thresh = 2.0;

    fitness_score_max_range = 25.0; // 5m
    fitness_score_thresh = 1.5;

    LOG_INFO("initializing loop detection...");
#ifdef HAVE_CUDA_ENABLE
    registration = select_registration_method("NDT_CUDA");
#else
    registration = select_registration_method("FAST_VGICP", 50000);
#endif
    registration_accurate = select_registration_method("FAST_GICP");
    registration_accurate->setMaxCorrespondenceDistance(0.5);
    last_edge_accum_distance = 0.0;
  }

  /**
   * @brief detect loops and add them to the pose graph
   * @param keyframes       keyframes
   * @param new_keyframes   newly registered keyframes
   * @param graph_slam      pose graph
   */
  std::vector<Loop::Ptr> detect(const std::vector<KeyFrame::Ptr>& keyframes, const std::deque<KeyFrame::Ptr>& new_keyframes, hdl_graph_slam::GraphSLAM& graph_slam, bool &is_running) {
    auto clock = std::chrono::steady_clock::now();

    std::vector<Loop::Ptr> detected_loops;
    double accum_distance_new_keyframe = 0;
    for(const auto& new_keyframe : new_keyframes) {
      if (!is_running) {
        break;
      }
      if ((new_keyframe->accum_distance - accum_distance_new_keyframe) < distance_new_keyframe_thresh) {
        continue;
      }

      accum_distance_new_keyframe = new_keyframe->accum_distance;
      auto candidates = find_candidates(keyframes, new_keyframe);
      auto loop = matching(candidates, new_keyframe, graph_slam);
      if(loop) {
        detected_loops.push_back(loop);
      }
    }

    auto elapseMs = since(clock).count();
    LOG_INFO("Graph: loop detection costs {} ms", elapseMs);
    return detected_loops;
  }

  double get_distance_thresh() const {
    return distance_thresh;
  }

private:
  /**
   * @brief find loop candidates. A detected loop begins at one of #keyframes and ends at #new_keyframe
   * @param keyframes      candidate keyframes of loop start
   * @param new_keyframe   loop end keyframe
   * @return loop candidates
   */
  std::vector<KeyFrame::Ptr> find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
    if(new_keyframe->accum_distance - last_edge_accum_distance < distance_from_last_edge_thresh) {
      return std::vector<KeyFrame::Ptr>();
    }

    std::vector<KeyFrame::Ptr> candidates;
    candidates.reserve(32);

    double accum_distance_keyframe = -100.0;
    for(const auto& k : keyframes) {
      // traveled distance between keyframes is too small
      if(new_keyframe->accum_distance - k->accum_distance < accum_distance_thresh) {
        continue;
      }

      if((k->accum_distance - accum_distance_keyframe) < distance_keyframe_thresh) {
        continue;
      }

      const auto& pos1 = k->node->estimate().translation();
      const auto& pos2 = new_keyframe->node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();
      if(dist > distance_thresh) {
        continue;
      }

      accum_distance_keyframe = k->accum_distance;
      candidates.push_back(k);
    }

    return candidates;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched well, the loop is added to the pose graph
   * @param candidate_keyframes  candidate keyframes of loop start
   * @param new_keyframe         loop end keyframe
   * @param graph_slam           graph slam
   */
  Loop::Ptr matching(const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe, hdl_graph_slam::GraphSLAM& graph_slam) {
    if(candidate_keyframes.empty()) {
      return nullptr;
    }

    registration->setInputTarget(new_keyframe->cloud);

    double best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    LOG_INFO("--- loop detection ---");
    // std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    // std::cout << "matching" << std::flush;

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    int candidate_idx = 0;
    for(const auto& candidate : candidate_keyframes) {
      candidate_idx++;
      registration->setInputSource(candidate->cloud);
      Eigen::Isometry3d new_keyframe_estimate = new_keyframe->node->estimate();
      new_keyframe_estimate.linear() = Eigen::Quaterniond(new_keyframe_estimate.linear()).normalized().toRotationMatrix();
      Eigen::Isometry3d candidate_estimate = candidate->node->estimate();
      candidate_estimate.linear() = Eigen::Quaterniond(candidate_estimate.linear()).normalized().toRotationMatrix();
      Eigen::Matrix4f guess = (new_keyframe_estimate.inverse() * candidate_estimate).matrix().cast<float>();
      guess(2, 3) = 0.0;
      registration->align(*aligned, guess);
      LOG_INFO("{} / {}", candidate_idx, candidate_keyframes.size());

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

    // std::cout << " done" << std::endl;
    // std::cout << "best_score: " << boost::format("%.3f") % best_score << "    time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

    if(best_score > (2.0 * fitness_score_thresh)) {
      LOG_INFO("score: {}, loop not found...", best_score);
      return nullptr;
    }

    registration_accurate->setInputTarget(new_keyframe->cloud);
    registration_accurate->setInputSource(best_matched->cloud);
    registration_accurate->align(*aligned, relative_pose);

    if(!registration_accurate->hasConverged()) {
      LOG_INFO("not converged, loop not found...");
      return nullptr;
    }
    best_score = registration_accurate->getFitnessScore(fitness_score_max_range);
    if(best_score > fitness_score_thresh) {
      LOG_INFO("score: {}, loop not found...", best_score);
      return nullptr;
    }

    relative_pose = registration_accurate->getFinalTransformation();
    LOG_INFO("score: {}, loop found!!", best_score);
    // std::cout << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - " << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;

    last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
  }

private:
  double distance_thresh;                 // estimated distance between keyframes consisting a loop must be less than this distance
  double accum_distance_thresh;           // traveled distance between ...
  double distance_from_last_edge_thresh;  // a new loop edge must far from the last one at least this distance
  double distance_new_keyframe_thresh;    // distance between new keyframes
  double distance_keyframe_thresh;        // distance between loop candidate keyframe

  double fitness_score_max_range;  // maximum allowable distance between corresponding points
  double fitness_score_thresh;     // threshold for scan matching

  double last_edge_accum_distance;

  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::Registration<PointT, PointT>::Ptr registration_accurate;
};

}  // namespace hdl_graph_slam

#endif  // LOOP_DETECTOR_HPP
