// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <Eigen/Dense>

namespace hdl_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater(double delta_trans, double delta_angle) : is_first(true), prev_keypose(Eigen::Isometry3d::Identity()) {
    keyframe_delta_trans = delta_trans;
    keyframe_delta_angle = delta_angle;

    accum_distance = 0.0;
  }

  bool is_first_frame() {
    return is_first;
  }

  void reset() {
    is_first = true;
    accum_distance = 0.0;
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose) {
    // first frame is always registered to the graph
    if(is_first) {
      is_first = false;
      prev_keypose = pose;
      return false;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = Eigen::AngleAxisd(delta.linear()).angle() / M_PI * 180;

    accum_distance += dx;
    prev_keypose = pose;
    return true;
  }

  void is_update(const Eigen::Isometry3d& pose, bool& need_update, bool& must_update, float& dx, float& da) {
    need_update = false;
    must_update = false;
    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    dx = delta.translation().norm();
    da = Eigen::AngleAxisd(delta.linear()).angle() / M_PI * 180;
    if(dx < (keyframe_delta_trans / 2.0) && da < (keyframe_delta_angle / 2.0)) {
      return;
    }

    need_update = true;
    if(dx >= (keyframe_delta_trans * 3.0 / 2.0) || da >= (keyframe_delta_angle * 3.0 / 2.0)) {
      must_update = true;
    }
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const {
    return accum_distance;
  }

public:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //

private:
  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_UPDATOR_HPP
